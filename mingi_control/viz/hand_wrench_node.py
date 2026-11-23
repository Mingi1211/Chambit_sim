#!/usr/bin/env python3
"""손끝 힘 시각화 + VLA 입력용 토픽 — 손목 F/T(시뮬 Gazebo ForceTorque) → 손이 박스에 주는 렌치, 박스 내력. 김민기 (11/22).

① 손목 센서 (gripper_{r,l}_joint, 측정 = 손목이 그리퍼에 주는 렌치 W_s, 센서 좌표)
② 그리퍼 무게 빼기 — 그리퍼 정지(준정적) 힘 평형: W_s + (그리퍼 무게) + (박스가 그리퍼에 주는 힘) = 0
     손이 박스에 주는 힘   f = F_s + m_g·g      (g = (0, 0, −9.81), 센서 좌표로 돌려서)
     손이 박스에 주는 모멘트 τ = T_s + r_c × (m_g·g)   (r_c = 그리퍼 무게중심, 센서 원점 기준)
     m_g, r_c: URDF 그리퍼 링크들 (Liu et al. 손목 F/T 중력 보정과 같은 생각) — 가감속 관성은 무시 (천천히 움직일 때)
③ 박스 내력 — 손 4개 렌치 h = [f_i; τ_i] (월드), 박스 기준점 p_o = 손 위치 평균
     박스에 걸리는 합 렌치 W_o = G h,  G = [[I, 0], [[p_i − p_o]×, I]] (i = 1..4)
     박스를 움직이는(무게·가속) 몫 G†W_o, 내력 h_int = (I − G†G) h   (Liu et al. 식 −G† h_o 와 같은 분해)
출력
  /robotN/hand_wrench_{r,l}  WrenchStamped (그리퍼 좌표 = RViz Wrench 표시용)
  /coop/hand_state           Float64MultiArray (VLA 입력): [손 4개 × (f 3, τ 3) 월드] + [손 4개 내력 f_int 3] + [‖f_int‖ 4]
                              순서: robot1_r, robot1_l, robot2_r, robot2_l
  /coop/hand_markers         MarkerArray (손 힘 = 파랑, 내력 = 빨강, 숫자)
사용: 깨끗한 셸 + scripts/environment.bash → python3 hand_wrench_node.py
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import rclpy
import rclpy.executors
from geometry_msgs.msg import Point, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ROBOTS, SIDES = ('robot1', 'robot2'), ('r', 'l')
G = np.array([0.0, 0.0, -9.81])
SCALE = 0.02                                                   # 화살표 길이 [m/N]
TAU_VIEW = 1.0                                                 # RViz 표시만 1 s 평균 (시간 그래프와 같은 창). 토픽 값은 원신호


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Robot:
    """한 로봇: URDF(pinocchio) + 관절 상태 → 그리퍼 센서 좌표 자세, 그리퍼 질량·무게중심"""

    def __init__(self, urdf):
        self.m = pin.buildModelFromXML(urdf)
        self.d = self.m.createData()
        self.q = pin.neutral(self.m)
        root = ET.fromstring(urdf)
        self.inertial = {}
        for link in root.findall('link'):
            i = link.find('inertial')
            if i is not None and i.find('mass') is not None:
                o = i.find('origin')
                c = np.array([float(v) for v in o.get('xyz', '0 0 0').split()]) if o is not None else np.zeros(3)
                self.inertial[link.get('name')] = (float(i.find('mass').get('value')), c)

    def set_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            n = n.split('::')[-1]
            if self.m.existJointName(n):
                jt = self.m.joints[self.m.getJointId(n)]
                if jt.nq == 1:
                    self.q[jt.idx_q] = p
        pin.framesForwardKinematics(self.m, self.d, self.q)

    def sensor(self, side):
        """센서(그리퍼 베이스) 자세 (base_link 기준), 그리퍼 질량, 무게중심(센서 좌표)"""
        oMs = self.d.oMf[self.m.getFrameId(f'gripper_{side}_rh_p12_rn_base')]
        mass, mc = 0.0, np.zeros(3)
        for name, (mi, ci) in self.inertial.items():
            if name.startswith(f'gripper_{side}_') and self.m.existFrame(name):
                p = self.d.oMf[self.m.getFrameId(name)].act(ci)
                mass += mi; mc += mi * p
        r_c = oMs.actInv(mc / mass) if mass > 0 else np.zeros(3)
        ee = self.d.oMf[self.m.getFrameId(f'end_effector_{side}_link')].translation
        return oMs, mass, r_c, ee


class HandWrench(Node):
    def __init__(self):
        super().__init__('hand_wrench')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.rob, self.ft = {}, {}
        self.tfb = Buffer(); TransformListener(self.tfb, self)
        self.pub_w, self.pub_state = {}, self.create_publisher(Float64MultiArray, '/coop/hand_state', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/coop/hand_markers', 10)
        for r in ROBOTS:
            self.create_subscription(String, f'/{r}/robot_description', lambda m, r=r: self.on_urdf(r, m), qos)
            self.create_subscription(JointState, f'/sim/{r}/joint_states', lambda m, r=r: self.on_js(r, m), qos_profile_sensor_data)
            for s in SIDES:
                self.create_subscription(WrenchStamped, f'/sim/{r}/ft_{s}', lambda m, r=r, s=s: self.ft.__setitem__((r, s), m),
                                         qos_profile_sensor_data)
                self.pub_w[(r, s)] = self.create_publisher(WrenchStamped, f'/{r}/hand_wrench_{s}', 10)
        self.create_timer(0.02, self.step)                     # 50 Hz 출력
        self.view = None                                       # 표시용 평균 (손 힘 4×3, 내력 4×3)

    def on_urdf(self, r, msg):
        if r not in self.rob:
            self.rob[r] = Robot(msg.data)

    def on_js(self, r, msg):
        if r in self.rob:
            self.rob[r].set_joints(msg)

    def step(self):
        if len(self.ft) < 4 or len(self.rob) < 2:
            return
        H, P = [], []
        now = self.ft[('robot1', 'r')].header.stamp          # 시뮬 시간 (TF와 같은 시계)
        for r in ROBOTS:
            try:
                tf = self.tfb.lookup_transform('world', f'{r}/base_link', rclpy.time.Time())
            except Exception:                                  # noqa: BLE001
                return
            Rw = quat_to_R(tf.transform.rotation)
            tw = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            for s in SIDES:
                oMs, mg, r_c, ee = self.rob[r].sensor(s)
                w = self.ft[(r, s)].wrench
                Fs = np.array([w.force.x, w.force.y, w.force.z]); Ts = np.array([w.torque.x, w.torque.y, w.torque.z])
                Rs = Rw @ oMs.rotation                         # 센서 → 월드 회전
                g_s = Rs.T @ G                                 # 중력 (센서 좌표)
                f_s = Fs + mg * g_s                            # 손 → 박스 힘 (센서 좌표)
                t_s = Ts + np.cross(r_c, mg * g_s)             # 손 → 박스 모멘트 (센서 원점)
                out = WrenchStamped()
                out.header.stamp, out.header.frame_id = now, f'{r}/gripper_{s}_rh_p12_rn_base'
                out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = map(float, f_s)
                out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = map(float, t_s)
                self.pub_w[(r, s)].publish(out)
                H.append(np.r_[Rs @ f_s, Rs @ t_s]); P.append(Rw @ ee + tw)
        H, P = np.array(H), np.array(P)
        po = P.mean(axis=0)
        Gm = np.hstack([np.block([[np.eye(3), np.zeros((3, 3))], [skew(p - po), np.eye(3)]]) for p in P])   # 6×24
        h = H.reshape(-1)
        h_int = (np.eye(24) - np.linalg.pinv(Gm) @ Gm) @ h
        f_int = h_int.reshape(4, 6)[:, :3]
        msg = Float64MultiArray()
        msg.data = [float(x) for x in np.r_[h, f_int.reshape(-1), np.linalg.norm(f_int, axis=1)]]
        self.pub_state.publish(msg)
        cur = np.hstack([H[:, :3], f_int])
        a = 0.02 / TAU_VIEW                                    # 지수 평균 (표시용)
        self.view = cur if self.view is None else (1 - a) * self.view + a * cur
        self.markers(P, self.view[:, :3], self.view[:, 3:], now)

    def markers(self, P, F, Fi, now):
        ma = MarkerArray()

        def arrow(i, p, f, col, ns):
            mk = Marker(); mk.header.frame_id, mk.header.stamp, mk.ns, mk.id = 'world', now, ns, i
            mk.type, mk.action = Marker.ARROW, Marker.ADD
            mk.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                         Point(x=float(p[0] + SCALE * f[0]), y=float(p[1] + SCALE * f[1]), z=float(p[2] + SCALE * f[2]))]
            mk.scale.x, mk.scale.y, mk.scale.z = 0.02, 0.04, 0.05
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = *col, 1.0
            return mk

        def text(i, p, s, dz=0.15):
            mk = Marker(); mk.header.frame_id, mk.header.stamp, mk.ns, mk.id = 'world', now, 'text', i
            mk.type, mk.action, mk.text = Marker.TEXT_VIEW_FACING, Marker.ADD, s
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(p[0]), float(p[1]), float(p[2] + dz)
            mk.scale.z = 0.045; mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
            return mk
        NAMES = ('R1', 'R1', 'R2', 'R2')                               # 로봇 번호만 (RViz 글꼴에 한글 없음)
        for i in range(4):
            ma.markers.append(arrow(i, P[i], F[i], (0.2, 0.45, 0.95), 'hand_force'))
            ma.markers.append(arrow(i, P[i], Fi[i], (0.9, 0.2, 0.2), 'internal_force'))
            ma.markers.append(text(i, P[i], f'{NAMES[i]}:{np.linalg.norm(F[i]):.1f}N'))
        ln = Marker(); ln.header.frame_id, ln.header.stamp, ln.ns, ln.id = 'world', now, 'grasp', 0   # 네 손을 잇는 선 (잡은 모양)
        ln.type, ln.action, ln.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.006
        ln.color.r = ln.color.g = ln.color.b = 0.8; ln.color.a = 0.8
        ln.points = [Point(x=float(P[k][0]), y=float(P[k][1]), z=float(P[k][2])) for k in (0, 1, 2, 3, 0)]   # 마주 보므로 R1 왼손 맞은편 = R2 오른손
        ma.markers.append(ln)
        self.pub_mk.publish(ma)


def main():
    rclpy.init()
    n = HandWrench()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
