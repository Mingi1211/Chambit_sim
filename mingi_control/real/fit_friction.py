#!/usr/bin/env python3
"""관절 마찰 식별 — MR 식·개념만. 김민기 (11/17).

  운동방정식 (한 관절, 나머지는 위치 고정)      τ = M(θ)θ̈ + c(θ,θ̇) + g(θ) + τ_fric          MR 8장 + 식 (11.21)
     c: 움직이는 관절 자신의 항 Γ_jjj θ̇_j² = 0 (Γ_jjj = ½ ∂M_jj/∂θ_j, 직렬 팔은 M_jj가 θ_j에 무관) — MR 크리스토펠 식에서 유도
     → 측정 마찰  τ_fric = τ_d − g(θ) − M_jj(θ)·θ̈      τ_d = c_kt·I (11/10), g·M = URDF (Newton–Euler, MR 8.3)
  사용하는 샘플
     · 보낸 궤적이 등속인 구간: 궤적은 0.05 s 간격 경유점이고 첫·마지막 경유점 구간에서만 가감속 → 그 두 구간만 제외
       (나머지 실제 가속은 M θ̈ 항으로 이미 포함)
     · 관절이 지령 방향으로 움직이는 샘플만: θ̇ = 0이면 마찰은 [−b_static, b_static] 안의 아무 값 (MR 그림 8.13(b)) → 식에 못 씀
  모델 고르기 (MR 그림 8.13)
     속도별 마찰 F(v) = 평균[ sgn(θ̇)·τ_fric ] (지령 속도 v마다)
     (c) b_static·sgn(θ̇) + b_viscous·θ̇ 는 마찰이 속도에 따라 커지는 구간의 모델, (f)는 저속에서 줄어드는 구간
     F(v)는 + 방향과 − 방향을 같은 구간에서 평균한 값 → 방향과 무관한 오차(URDF 중력 오차 등)는 지워지고 (c) 모델의 b_static + b_viscous·v만 남음
     → 경계 = F(v)가 가장 작은 속도. 그 속도 이상의 F(v) 점에 직선 F = b_static + b_viscous·v 최소제곱
     → 맞춘 b_viscous ≤ 0 (MR 식 (11.20) b > 0 위반) 또는 최소가 가장 빠른 속도면: 커지는 구간 없음 → (b) b_static만 = 전체 F(v) 평균
  최소제곱 = pinv(Θ)·F, RSD·RMSE는 일반 통계
  시뮬 값 <friction> = b_static, <damping> = b_viscous

사용: source /opt/ros/jazzy/setup.bash
  python3 fit_friction.py --arm l --files l_visc_j*.csv l_viscfast_j*.csv --urdf ../robot_description.urdf --table ../kt_friction_table_1110.csv --out .
  (같은 관절 파일이 여러 개면 합쳐서 씀)
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib import font_manager

for f in ('NanumGothic', 'Noto Sans CJK KR'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f
        break
plt.rcParams['axes.unicode_minus'] = False

ap = argparse.ArgumentParser()
ap.add_argument('--arm', required=True)
ap.add_argument('--files', nargs='+', required=True)
ap.add_argument('--urdf', required=True)
ap.add_argument('--table', required=True)
ap.add_argument('--out', default='.')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
WP = 0.05                                                      # static_current.py go_const 경유점 간격 [s]

C = {int(r['joint']): float(r['c_all']) for r in csv.DictReader(open(a.table, encoding='utf-8')) if r['arm'] == a.arm}
text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
Mdl = pin.buildModelFromXML(text)
Dd = Mdl.createData()
IQ = [Mdl.joints[Mdl.getJointId(f'arm_{a.arm}_joint{j}')].idx_q for j in range(1, 8)]
IV = [Mdl.joints[Mdl.getJointId(f'arm_{a.arm}_joint{j}')].idx_v for j in range(1, 8)]


def load(path, j):
    R = list(csv.DictReader(open(path)))
    out = {k: [] for k in ('v', 'dq', 'tf')}
    i, n = 0, len(R)
    while i < n:                                               # 이동 구간 = 같은 (속도, 방향)의 ramp/cruise 연속 행
        if R[i]['phase'] not in ('ramp', 'cruise'):
            i += 1
            continue
        s = i
        while i < n and R[i]['phase'] in ('ramp', 'cruise') and R[i]['target'] == R[s]['target'] and R[i]['dir'] == R[s]['dir']:
            i += 1
        P = R[s:i]
        t = np.array([float(r['t']) for r in P])
        if len(t) < 5:
            continue
        v = float(P[0]['target']); sg = 1.0 if P[0]['dir'] == 'pos' else -1.0
        dq = np.array([float(r[f'dq{j}']) for r in P])
        ddq = np.gradient(dq, t)
        keep = (t - t[0] >= WP) & (t[-1] - t >= WP) & (np.sign(dq) == sg)
        for r, k_, a_, dq_ in zip(P, keep, ddq, dq):
            if not k_:
                continue
            q = pin.neutral(Mdl)
            for kk, idx in enumerate(IQ):
                q[idx] = float(r[f'q{kk + 1}'])
            g = pin.computeGeneralizedGravity(Mdl, Dd, q)[IV[j - 1]]
            Mjj = pin.crba(Mdl, Dd, q)[IV[j - 1], IV[j - 1]]
            out['v'].append(v); out['dq'].append(dq_)
            out['tf'].append(C[j] * float(r[f'I{j}']) - g - Mjj * a_)
    return {k: np.array(x) for k, x in out.items()}


def lsq(Th, y):
    z = np.linalg.pinv(Th) @ y
    p, n = Th.shape
    s2 = np.sum((y - Th @ z) ** 2) / (p - n)
    rsd = 100 * np.sqrt(np.diag(s2 * np.linalg.inv(Th.T @ Th))) / np.abs(z)
    return z, rsd, float(np.sqrt(np.mean((y - Th @ z) ** 2)))


groups = {}
for p in a.files:
    groups.setdefault(int(re.search(r'_j(\d)', os.path.basename(p)).group(1)), []).append(p)
res, rows = {}, []
print(f'\n{a.arm}팔 마찰 — τ_fric = τ_d − g − M θ̈ (MR 8장), 모델 MR 그림 8.13')
print(' 관절 | 속도별 마찰 F(v) [N·m]                                         | 모델 (사용 속도)  | b_static (RSD) | b_viscous (RSD) | RMSE')
for j in sorted(groups):
    D = [load(p, j) for p in groups[j]]
    d = {k: np.concatenate([x[k] for x in D]) for k in D[0]}
    V = np.array(sorted(set(d['v'])))
    F = np.array([np.mean(np.sign(d['dq'][d['v'] == v]) * d['tf'][d['v'] == v]) for v in V])
    i0 = int(np.argmin(F))
    z = None
    if len(V) - i0 >= 3:                                       # 커지는 구간 (점 3개 이상이어야 RSD 계산 가능) → (c)
        z, rsd, e = lsq(np.c_[np.ones(len(V) - i0), V[i0:]], F[i0:])
        model = f'(c) {V[i0]:g}~{V[-1]:g}'
    if z is None or z[1] <= 0:                                 # 커지는 구간 없음 → (b)
        i0 = len(V) - 1
        z1, r1, e = lsq(np.ones((len(V), 1)), F)
        z, rsd = np.r_[z1, 0.0], np.r_[r1, np.nan]
        model = f'(b) {V[0]:g}~{V[-1]:g}'
    res[j] = dict(V=V, F=F, z=z, i0=i0)
    fs = ' '.join(f'{v:g}:{f:.2f}' for v, f in zip(V, F))
    print(f'  j{j}  | {fs:62s} | {model:16s} | {z[0]:5.2f} ({rsd[0]:4.1f}%) | {z[1]:6.2f} ({rsd[1]:5.1f}%)  | {e:.2f}')
    rows.append(dict(arm=a.arm, joint=j, model=model, b_static=round(z[0], 3), b_viscous=round(z[1], 3),
                     rsd_static=round(rsd[0], 1), rsd_viscous=round(rsd[1], 1) if np.isfinite(rsd[1]) else '', rmse=round(e, 3),
                     speeds=' '.join(f'{v:g}' for v in V), F_per_speed=' '.join(f'{f:.3f}' for f in F)))
with open(os.path.join(a.out, f'friction_{a.arm}.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
print('(c) = 마찰이 속도에 따라 커지는 구간에서 쿨롱+점성, (b) = 커지는 구간 없음 → 쿨롱만 (MR 그림 8.13)')
print('시뮬 <friction> = b_static, <damping> = b_viscous')

J = sorted(res)
fig, ax = plt.subplots(2, 4, figsize=(15, 7))
for n_, j in enumerate(J):
    x, R = ax.flat[n_], res[j]
    x.plot(np.r_[-R['V'][::-1], R['V']], np.r_[-R['F'][::-1], R['F']], 'o', color='C0', label='측정 F(v) (지령 속도별 평균)')
    v0 = R['V'][R['i0']] if R['i0'] < len(R['V']) - 1 else 0.0
    vv = np.linspace(v0, R['V'][-1], 50)
    x.plot(vv, R['z'][0] + R['z'][1] * vv, 'r-', lw=1.2); x.plot(-vv, -(R['z'][0] + R['z'][1] * vv), 'r-', lw=1.2, label='모델 (MR 그림 8.13)')
    if 0 < R['i0'] < len(R['V']) - 1:
        x.axvspan(-v0, v0, color='orange', alpha=0.12, label='마찰이 줄어드는 저속 구간 (그림 8.13(f))')
    x.axhline(0, color='gray', lw=0.5); x.axvline(0, color='gray', lw=0.5)
    x.set_title(f'j{j}: b_static {R["z"][0]:.2f} N·m, b_viscous {R["z"][1]:.2f} N·m·s/rad', fontsize=10)
    x.set_xlabel('관절 속도 [rad/s]'); x.set_ylabel('마찰 토크 [N·m]')
ax.flat[0].legend(fontsize=7)
for x in ax.flat[len(J):]:
    x.axis('off')
fig.suptitle(f'{a.arm}팔 관절 마찰 실측 (박스 없음, 등속 왕복)')
fig.tight_layout(); p1 = os.path.join(a.out, f'friction_curve_{a.arm}.png'); fig.savefig(p1, dpi=130)
print('그림:', p1)
