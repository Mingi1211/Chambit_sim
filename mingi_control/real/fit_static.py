#!/usr/bin/env python3
"""static_current.py 결과 분석 — 정지 전류 vs URDF 중력 토크 → 환산 보정계수(Kt), 정지마찰 τ_s. 김민기 (11/8).

입력: sweep CSV(static_current.py sweep), robot_description.urdf(static_current.py urdf, 앞의 비XML 줄은 자동 제거)
  hold 구간(정지 1 s 뒤) 평균 → 자세·접근방향별 관절 j '전류' I_j
  ※ /dynamic_joint_states 'Present Current'는 ROBOTIS 모델 파일([unit info])이 raw에 공칭 환산을 곱한 [N·m] 값
     (YM070-210-R099: 0.04 N·m/raw, YM080-230-R099: 0.0769 N·m/raw)
  두 방향 평균  I_avg = s·g_j(q) + c   (g = Pinocchio RNEA(q,0,0), 실물 URDF)  → s: 공칭 환산 / URDF 중력 비
  (위 − 아래)/2 = 정지마찰 (공칭 단위) → /s 로 URDF 토크 단위 τ_s
  손끝 마찰 F = |J̄ᵀ|[:, j] τ_s (파지 자세, armature 손목 0.05) → K_min = F/δx (문서 3·4쪽)
  ※ s가 공칭 Kt 오차인지 URDF 질량 오차인지는 이 데이터로 못 가림 → 질량 아는 박스로 차이 측정 필요

사용: python3 fit_static.py sweep_l_j4.csv --urdf robot_description.urdf --arm l --joint 4
"""
import argparse
import collections
import csv

import numpy as np
import pinocchio as pin

ap = argparse.ArgumentParser()
ap.add_argument('csv')
ap.add_argument('--urdf', required=True)
ap.add_argument('--arm', default='l')
ap.add_argument('--joint', type=int, required=True)
ap.add_argument('--dx', type=float, default=0.005, help='목표 정밀도 δx [m]')
a = ap.parse_args()

text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
m = pin.buildModelFromXML(text)
d = m.createData()
J = a.joint
vi = lambda j: m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_v  # noqa: E731
qi = lambda j: m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_q  # noqa: E731

rows = [r for r in csv.DictReader(open(a.csv)) if r['phase'] == 'hold']
if not rows:
    raise SystemExit('hold 구간 없음')
q_all = np.array([float(r[f'q{J}']) for r in csv.DictReader(open(a.csv))])
print(f'관절{J} 실제 이동 범위 {q_all.min():+.3f} ~ {q_all.max():+.3f} rad')
groups = collections.defaultdict(list)
for r in rows:
    groups[float(r['target']), r['dir']].append(r)
res = collections.defaultdict(dict)
print('목표    방향        q[rad]    I(공칭 N·m)     g(URDF N·m)')
for (tg, dr), rs in sorted(groups.items()):
    q = np.array([[float(r[f'q{i}']) for i in range(1, 8)] for r in rs]).mean(0)
    I = np.array([float(r[f'I{J}']) for r in rs])
    qq = pin.neutral(m)
    for i in range(7):
        qq[qi(i + 1)] = q[i]
    g = pin.computeGeneralizedGravity(m, d, qq)[vi(J)]
    res[tg][dr] = (q[J - 1], I.mean(), I.std(), g)
    print(f'{tg:+.2f}  {dr:10s}  {q[J-1]:+.4f}  {I.mean():7.2f}±{I.std():.2f}   {g:+.3f}')
X, Y, F = [], [], []
for tg, v in sorted(res.items()):
    if len(v) == 2:
        up, dn = v['from_above'], v['from_below']
        X.append((up[3] + dn[3]) / 2)
        Y.append((up[1] + dn[1]) / 2)
        F.append((up[1] - dn[1]) / 2)
X, Y, F = map(np.array, (X, Y, F))
if len(X) >= 2:
    s, c = np.linalg.lstsq(np.vstack([X, np.ones_like(X)]).T, Y, rcond=None)[0]
    print(f'\n적합 I_avg = {s:.3f}·g + {c:.3f}  (잔차 {np.round(Y - (s * X + c), 2)})  → 공칭 환산이 URDF 중력의 {s:.2f}배')
else:
    s = 1.0
    print('\n자세 2개 이상 필요 — 적합 생략 (s = 1 가정)')
tau_s = np.abs(F) / s
print('정지마찰 τ_s (URDF 토크 단위):', dict(zip(np.round(X, 2), np.round(tau_s, 2))), '(키 = 중력 토크 g)')
try:
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from gain_design import Q_GRASP
    qq = pin.neutral(m)
    for i, v in enumerate(Q_GRASP[a.arm]):
        qq[qi(i + 1)] = v
    pin.computeJointJacobians(m, d, qq)
    pin.framesForwardKinematics(m, d, qq)
    M = pin.crba(m, d, qq)
    M = np.triu(M) + np.triu(M, 1).T
    idx = [vi(j) for j in range(1, 8)]
    Jac = pin.getFrameJacobian(m, d, m.getFrameId(f'end_effector_{a.arm}_link'), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, idx]
    Ma = M[np.ix_(idx, idx)] + np.diag([0, 0, 0, 0, 0.05, 0.05, 0.05])
    Mi = np.linalg.inv(Ma)
    L = np.linalg.inv(Jac @ Mi @ Jac.T + 1e-6 * np.eye(6))
    col = np.abs((L @ Jac @ Mi)[:3, J - 1])
    for t in tau_s:
        print(f'  파지 자세에서 관절{J} τ_s={t:.2f} N·m → 손끝 F_fric (x y z) {np.round(col * t, 2)} N → K_min {np.round(col * t / a.dx)} N/m')
except Exception as e:  # gain_design 없거나 자세 없음
    print('K_min 계산 생략:', e)
