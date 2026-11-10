#!/usr/bin/env python3
"""관절별 박스 Kt 측정 자세 찾기 — 실물 URDF로 박스 토크 신호가 크고 박스가 몸체에 안 닿는 기준 자세. 김민기 (11/10).

조건 (11/9 현장: 우체국 5호 박스는 손이 낮으면 베이스에 닿음)
  - 측정 관절 j를 기준 ±range 움직이는 동안 손끝 높이 z ≥ --zmin, 앞쪽 x ≥ --xmin (base_link 기준)
  - 관절 한계 안
  - 그 구간에서 박스가 관절 j에 주는 토크 |Δτ_j| = |J_vᵀ[0,0,m g]|_j 의 최솟값이 가장 큰 자세 (MR 5.4)
탐색: 11/9 자세 P = (0,0,0,−1.57,1.57,0,0)에서 관절 1·2·3·5·6·7 몇 값 격자

사용: python3 plan_kt_poses.py --urdf robot_description.urdf --arm l --mass 2.3
"""
import argparse
import itertools

import numpy as np
import pinocchio as pin

ap = argparse.ArgumentParser()
ap.add_argument('--urdf', required=True)
ap.add_argument('--arm', default='l')
ap.add_argument('--mass', type=float, default=2.3)
ap.add_argument('--range', type=float, default=0.2)
ap.add_argument('--zmin', type=float, default=1.0)
ap.add_argument('--xmin', type=float, default=0.45)
a = ap.parse_args()

text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
m = pin.buildModelFromXML(text)
d = m.createData()
iq = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_q for j in range(1, 8)]
iv = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_v for j in range(1, 8)]
lo = np.array([m.lowerPositionLimit[k] for k in iq])
hi = np.array([m.upperPositionLimit[k] for k in iq])
fid = m.getFrameId(f'end_effector_{a.arm}_link')
F = np.array([0.0, 0.0, a.mass * 9.81])
s = 1.0 if a.arm == 'l' else -1.0      # 오른팔은 관절2 부호 반대 (한계 −3.14 ~ 0)


def ev(q7):
    q = pin.neutral(m)
    for k, v in zip(iq, q7):
        q[k] = v
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    Jv = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, iv]
    return Jv.T @ F, d.oMf[fid].translation.copy()


P = np.array([0, 0, 0, -1.57, 1.57, 0, 0.0])
grid = {0: [0, -0.3, 0.3], 1: [0, s * 0.3], 2: [0, -0.6, 0.6], 4: [1.57, 0.0, 0.8], 5: [0, -0.6, 0.6], 6: [0, -0.6, 0.6]}
best = {}
for vals in itertools.product(*[grid[k] for k in sorted(grid)]):
    q = P.copy()
    for k, v in zip(sorted(grid), vals):
        q[k] = v
    if np.any(q < lo) or np.any(q > hi):
        continue
    for j in range(7):
        sig = []
        for dq in np.linspace(-a.range, a.range, 5):
            qq = q.copy()
            qq[j] += dq
            dt, p = ev(qq)
            if qq[j] < lo[j] or qq[j] > hi[j] or p[2] < a.zmin or p[0] < a.xmin:
                sig = None
                break
            sig.append(abs(dt[j]))
        if sig and (j not in best or min(sig) > best[j][0]):
            best[j] = (min(sig), q.copy())
print(f'조건: 손 z ≥ {a.zmin} m, x ≥ {a.xmin} m (base_link), 측정 관절 ±{a.range} rad, 박스 {a.mass} kg')
for j in range(7):
    if j not in best:
        print(f'j{j + 1}: 조건 만족 자세 없음')
        continue
    sig, q = best[j]
    dt, p = ev(q)
    print(f'j{j + 1}: --base {" ".join(f"{v:g}" for v in np.round(q, 2))}   |Δτ_j| ≥ {sig:.1f} N·m, 손 {np.round(p, 2)} m')
