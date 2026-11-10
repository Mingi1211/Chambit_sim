#!/usr/bin/env python3
"""등속 왕복(static_current.py vel) 박스 없음/있음 → 관절별 토크 환산 보정계수 c와 하중 의존 마찰. 김민기 (11/10).

모델 (등속 구간 q̈ = 0, 저속이라 코리올리 무시 — MR 8장 τ = M q̈ + c(q,q̇) + g(q) + Jᵀ F 에서)
  c·I = g(q) + Jᵀ F_box + τ_f
  τ_f = s·(f_c + f_l·|τ_L|) + f_v·q̇      s = sign(q̇), τ_L = g + JᵀF (하중 토크)
        쿨롱 f_c, 하중 비례 f_l (감속기 하중 의존 마찰), 점성 f_v — 마찰 모델 일반형 (논문 ② 형태와 대조 필요)
  같은 q에서 + 방향·− 방향을 지날 때:  평균 Ī = (I₊ + I₋)/2 → c·Ī = τ_L          (대칭 마찰이면 상쇄)
                                     반차 Ĩ = (I₊ − I₋)/2 → c·Ĩ = f_c + f_l|τ_L| + f_v|q̇|
  박스 있음 − 없음 (같은 q, 같은 속도):  c·ΔĪ = J_vᵀ[0,0,m g] = Δτ      (MR 5.4) → 링크 질량(URDF) 오차가 빠짐
  최소제곱 c = ΣΔτ·ΔĪ / ΣΔĪ²  (Drive 덱 24쪽 ①) — 구간 안의 q 칸(bin)마다 한 점 → 자세 1개보다 점이 많음
  한계 (11/10): 마찰이 방향에 따라 다르면(감속기 정·역 효율 차이, f_l₊ ≠ f_l₋) c·ΔĪ = Δτ + (f_l₊ − f_l₋)/2·Δ|τ_L| 이 되어
        c가 편향된다. 이 편향은 Δτ와 같은 방향으로 커서 이 데이터만으로는 c와 분리할 수 없음(식별 불가)
        → 마찰이 작은 관절(j1)과 비교, 전류 모드에서 알려진 전류로 교차 확인이 필요

사용: python3 fit_kt_vel.py nobox_vel_j4.csv box_vel_j4.csv --urdf robot_description.urdf --arm l --joint 4 --mass 2.3
"""
import argparse
import csv

import numpy as np
import pinocchio as pin

ap = argparse.ArgumentParser()
ap.add_argument('nobox')
ap.add_argument('box')
ap.add_argument('--urdf', required=True)
ap.add_argument('--arm', default='l')
ap.add_argument('--joint', type=int, required=True)
ap.add_argument('--mass', type=float, required=True)
ap.add_argument('--bin', type=float, default=0.01, help='q 칸 크기 [rad]')
ap.add_argument('--frame', default=None)
a = ap.parse_args()
J = a.joint - 1
G = 9.81

text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
m = pin.buildModelFromXML(text)
d = m.createData()
iq = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_q for j in range(1, 8)]
iv = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_v for j in range(1, 8)]
fid = m.getFrameId(a.frame or f'end_effector_{a.arm}_link')


def binned(path):
    """cruise 구간을 (속도, q 칸)별로 + / − 방향 평균 전류(관절 J)와 평균 자세"""
    out = {}
    for r in csv.DictReader(open(path)):
        if r['phase'] != 'cruise':
            continue
        q = np.array([float(r[f'q{i}']) for i in range(1, 8)])
        key = (float(r['target']), int(np.floor(q[J] / a.bin)))
        e = out.setdefault(key, {'pos': [], 'neg': [], 'q': []})
        e[r['dir']].append(float(r[f'I{J + 1}']))
        e['q'].append(q)
    return {k: (np.mean(v['pos']), np.mean(v['neg']), np.mean(v['q'], axis=0))
            for k, v in out.items() if v['pos'] and v['neg']}


def load_torques(q7):
    q = pin.neutral(m)
    for k, v in zip(iq, q7):
        q[k] = v
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    Jv = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, iv]
    return (Jv.T @ np.array([0.0, 0.0, a.mass * G]))[J], pin.computeGeneralizedGravity(m, d, q)[iv][J]


NB, BX = binned(a.nobox), binned(a.box)
keys = sorted(set(NB) & set(BX))
if len(keys) < 3:
    raise SystemExit(f'공통 (속도, q칸)이 {len(keys)}개 — 같은 --base --range --speeds로 측정했는지, cruise 구간이 있는지 확인')
rows = []
for k in keys:
    pn, nn, qn = NB[k]
    pb, nb, qb = BX[k]
    dtau, g = load_torques((qn + qb) / 2)
    rows.append(dict(v=k[0], q=(qn[J] + qb[J]) / 2, dtau=dtau, g=g,
                     avg_n=(pn + nn) / 2, avg_b=(pb + nb) / 2, half_n=(pn - nn) / 2, half_b=(pb - nb) / 2,
                     dpos=pb - pn, dneg=nb - nn))
R = {k: np.array([r[k] for r in rows]) for k in rows[0]}
dI = R['avg_b'] - R['avg_n']


def ls(x, y):
    return float(x @ y / (x @ x)), float(np.sqrt(np.mean((y - (x @ y / (x @ x)) * x) ** 2)))


c, rms = ls(dI, R['dtau'])
print(f'관절{J + 1}: 점 {len(rows)}개 (속도 {sorted(set(R["v"]))} rad/s, q {R["q"].min():+.3f} ~ {R["q"].max():+.3f})')
print(f'  박스 Δτ(URDF) 범위 {R["dtau"].min():+.2f} ~ {R["dtau"].max():+.2f} N·m')
print(f'  c (두 방향 평균)  = {c:.3f}   잔차 RMS {rms:.3f} N·m')

tauL = np.concatenate([c * R['avg_n'], c * R['avg_b']])
tauf = np.concatenate([c * R['half_n'], c * R['half_b']])
vv = np.concatenate([R['v'], R['v']])
A = np.vstack([np.ones_like(tauL), np.abs(tauL), vv]).T
(fc, fl, fv), *_ = np.linalg.lstsq(A, tauf, rcond=None)
print(f'  마찰 τ_f = f_c + f_l|τ_L| + f_v|q̇|:  f_c = {fc:.3f} N·m,  f_l = {fl:.3f},  f_v = {fv:.3f} N·m·s/rad'
      f'   (잔차 RMS {np.sqrt(np.mean((tauf - A @ np.array([fc, fl, fv])) ** 2)):.3f})')
ok = np.abs(R['g']) >= 0.5
if ok.any():
    print(f'  박스 없을 때 실제 중력/URDF 중력 = c·Ī/g: 평균 {np.mean(c * R["avg_n"][ok] / R["g"][ok]):.2f} '
          f'(범위 {np.min(c * R["avg_n"][ok] / R["g"][ok]):.2f} ~ {np.max(c * R["avg_n"][ok] / R["g"][ok]):.2f})')
print('  단위: c는 [N·m / Present Current 1단위]. j1~j6은 공칭 N·m라 c≈1이면 공칭이 맞음. j7(PH 계열, raw)은 c가 곧 N·m/raw')
