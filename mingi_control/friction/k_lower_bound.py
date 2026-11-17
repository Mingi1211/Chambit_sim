#!/usr/bin/env python3
"""남은 마찰 → 원하는 정상상태 오차 → 강성 하한. 김민기 (11/17).

남은 마찰: 피드포워드 b_static·sgn(θ̇_d) + b_viscous·θ̇_d 는 움직이는 동안만 상쇄. 정지 목표(θ̇_d = 0)에서는 0
           → 정지 마찰 b_static 전부가 남음 (MR 그림 8.13(b): 정지 시 마찰은 [−b_static, b_static] 아무 값)
관절 공간: 정지 시 K_p θ_e = 외란 (MR (11.27) 꼴) → |θ_e| ≤ b_static / K_p → K_p ≥ b_static / δθ_j,
           δθ_j = δx / ‖J_v,j‖ (그 관절만 움직일 때 손끝 변위 = J_v,j Δθ_j, MR 5장)
작업 공간: 손끝에서 버텨야 하는 힘 F = |J̄ᵀ| τ_s (관절 부호가 최악으로 겹친 경우), K ≥ F / δx   (김민기 게인 문서 3·4쪽)
           J̄ᵀ = Λ J_v M⁻¹,  Λ = (J_v M⁻¹ J_vᵀ)⁻¹  (Khatib 작업공간 동역학의 동역학적으로 일관된 역 — 7자유도 여유 팔. MR 8.6은 정방 J의 Λ만)
자세: 박스 파지 자세 Q_GRASP (gain_design.py, 11/4)

사용: source /opt/ros/jazzy/setup.bash; python3 k_lower_bound.py --urdf ../../real_logs/robot_description.urdf --dx 0.005
"""
import argparse
import os

import numpy as np
import pinocchio as pin
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument('--urdf', required=True)
ap.add_argument('--dx', type=float, default=0.005, help='원하는 손끝 정상상태 오차 [m]')
ap.add_argument('--yaml', default=os.path.join(HERE, '../../src/coop_carry_bringup/config/robots.yaml'))
a = ap.parse_args()
QG = {'r': [-0.3888, -0.7111, 1.9041, -1.37, -0.3074, 0.6082, -1.5823],
      'l': [0.5106, 0.1599, -0.0832, -1.266, 1.7479, -0.0301, -0.8224]}
FR = yaml.safe_load(open(a.yaml))['joint_friction']
t = open(a.urdf).read()
t = t[t.index('<?xml'):] if '<?xml' in t else t[t.index('<robot'):]
m = pin.buildModelFromXML(t); d = m.createData()
for s in ('r', 'l'):
    iq = [m.joints[m.getJointId(f'arm_{s}_joint{j}')].idx_q for j in range(1, 8)]
    iv = [m.joints[m.getJointId(f'arm_{s}_joint{j}')].idx_v for j in range(1, 8)]
    q = pin.neutral(m)
    for k, v in zip(iq, QG[s]):
        q[k] = v
    pin.computeJointJacobians(m, d, q); pin.framesForwardKinematics(m, d, q)
    fid = m.getFrameId(f'end_effector_{s}_link')
    Jv = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, iv]
    M = pin.crba(m, d, q)[np.ix_(iv, iv)]
    Mi = np.linalg.inv(M)
    Lam = np.linalg.inv(Jv @ Mi @ Jv.T)
    JbT = Lam @ Jv @ Mi                                                      # J̄ᵀ (3×7)
    ts = np.array(FR[s]['friction'])
    F = np.abs(JbT) @ ts
    print(f'\n{s}팔 (파지 자세) — 남은 마찰 = 정지 마찰 b_static {np.round(ts, 2)} N·m')
    dth = a.dx / np.linalg.norm(Jv, axis=0)
    print(f'  관절별 허용 δθ_j = δx/‖J_v,j‖ [mrad]: {np.round(dth * 1e3, 1)}')
    print(f'  관절 공간 K_p ≥ b_static/δθ_j: {np.round(ts / dth, 0)} N·m/rad')
    print(f'  손끝 마찰력 |J̄ᵀ|τ_s (x, y, z) = {np.round(F, 1)} N  →  K ≥ F/δx (δx = {a.dx * 1e3:.0f} mm) = {np.round(F / a.dx, 0)} N/m')
