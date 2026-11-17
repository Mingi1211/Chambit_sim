"""URDF로 팔 중력 토크 g(q)와 박스 토크 τ_box = J_vᵀ[0,0,m g] 계산 (numpy만 사용 — Windows·Ubuntu 공통).

부호 규약: 정지 상태에서 모터가 내야 하는 토크 (M q̈ + c + g = τ 의 g, Pinocchio computeGeneralizedGravity와 같음)
  g(q)     = Σ_링크 J_v(링크 질량중심)ᵀ · [0, 0, m_링크 · 9.81]      (MR 8.1 위치 에너지의 기울기)
  τ_box(q) = J_v(매단 점)ᵀ · [0, 0, m_box · 9.81]                    (MR 5.2 정역학 τ = JᵀF)
팔 관절(arm_<s>_joint1~7) 외 관절은 0 (리프트는 수직 이동이라 중력 토크에 영향 없음).
"""
import xml.etree.ElementTree as ET

import numpy as np

G0 = 9.81


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def _axis_rot(a, th):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


class Arm:
    def __init__(self, urdf_path, side):
        text = open(urdf_path, encoding='utf-8').read()
        # robot_description을 표준출력으로 받으면 앞에 다른 출력 줄이 섞일 수 있음 → XML 시작부터
        text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
        root = ET.fromstring(text)
        self.side = side
        self.links = {}
        for l in root.findall('link'):
            i = l.find('inertial')
            if i is not None and i.find('mass') is not None:
                self.links[l.get('name')] = (float(i.find('mass').get('value')), _origin(i))
        self.joints = {}
        for j in root.findall('joint'):
            ax = j.find('axis')
            lim = j.find('limit')
            self.joints[j.get('name')] = dict(
                type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=_origin(j), axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([1., 0, 0]),
                lim=(float(lim.get('lower', '-inf')), float(lim.get('upper', 'inf'))) if lim is not None else None)
        self.child2joint = {v['child']: k for k, v in self.joints.items()}
        self.arm = [f'arm_{side}_joint{i}' for i in range(1, 8)]
        self.ee = f'end_effector_{side}_link'
        if self.ee not in self.child2joint:
            raise SystemExit(f'URDF에 {self.ee} 없음')
        self._chain = {}

    def limits(self):
        return [self.joints[j]['lim'] for j in self.arm]

    def _path(self, link):
        if link not in self._chain:
            out, l = [], link
            while l in self.child2joint:
                jn = self.child2joint[l]
                out.append(jn)
                l = self.joints[jn]['parent']
            self._chain[link] = set(out)
        return self._chain[link]

    def _fk(self, q_arm):
        q = dict(zip(self.arm, q_arm))
        Tw = {}

        def world(link):
            if link in Tw:
                return Tw[link]
            if link not in self.child2joint:
                Tw[link] = np.eye(4)
                return Tw[link]
            jn = self.child2joint[link]
            j = self.joints[jn]
            M = np.eye(4)
            if j['type'] in ('revolute', 'continuous'):
                M[:3, :3] = _axis_rot(j['axis'], q.get(jn, 0.0))
            elif j['type'] == 'prismatic':
                M[:3, 3] = j['axis'] * q.get(jn, 0.0)
            Tw[link] = world(j['parent']) @ j['T'] @ M
            return Tw[link]
        return world

    def _jv(self, world, point, link):
        """point(월드)에서 선속도 자코비안 3×7 (link의 조상 관절만)"""
        Jv = np.zeros((3, 7))
        p = self._path(link)
        for k, jn in enumerate(self.arm):
            if jn in p:
                j = self.joints[jn]
                T = world(j['parent']) @ j['T']
                a = T[:3, :3] @ (j['axis'] / np.linalg.norm(j['axis']))
                Jv[:, k] = np.cross(a, point - T[:3, 3])
        return Jv

    def gravity(self, q_arm):
        world = self._fk(q_arm)
        g = np.zeros(7)
        for name, (m, Tc) in self.links.items():
            if not (self._path(name) & set(self.arm)):
                continue
            pc = (world(name) @ Tc)[:3, 3]
            g += self._jv(world, pc, name).T @ np.array([0, 0, m * G0])
        return g

    def hang_point(self, q_arm, offset=(0, 0, 0)):
        world = self._fk(q_arm)
        return (world(self.ee) @ np.r_[offset, 1.0])[:3]

    def box_torque(self, q_arm, m_box, offset=(0, 0, 0)):
        """end_effector 프레임 기준 offset[m] 점에 m_box[kg]을 매달았을 때 관절 토크 (끈으로 자유롭게 늘어뜨린 경우)"""
        world = self._fk(q_arm)
        p = (world(self.ee) @ np.r_[offset, 1.0])[:3]
        return self._jv(world, p, self.ee).T @ np.array([0, 0, m_box * G0])
