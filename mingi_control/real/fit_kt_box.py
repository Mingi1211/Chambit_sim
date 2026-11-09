#!/usr/bin/env python3
"""박스 질량 차이로 토크 환산(Kt) 보정계수 식별 — 정지 자세, 위치 모드. 김민기 (11/9).

근거
  MR 8장 동역학에서 q̇ = q̈ = 0 (정지) → τ = g(q) + Jᵀ(q) F_tip           (F_tip: 로봇이 환경에 가하는 렌치)
  MR 5.4 정역학 τ = Jᵀ F: 손에 매단 박스(질량 m)를 버티려면 로봇이 박스에 위로 m g를 가함
      → 박스 때문에 늘어나는 관절 토크  Δτ = J_vᵀ(q) [0, 0, m g]ᵀ      (base_link 기준, z 위쪽)
      ※ 박스가 손잡이(테이프 고리)에 자유롭게 매달려 있으면 힘의 작용선이 매단 점을 지나므로 모멘트 항 없음 (가정)
  측정 모델: 실제 토크 τ = c · I   (I = 'Present Current', ROBOTIS 공칭 환산 N·m, c = 보정계수, 1이면 공칭이 맞음)
      정지마찰은 위·아래 두 방향 접근 평균으로 상쇄 (대칭 쿨롱 가정, 11/8 static_current 방식)
      같은 자세에서 (박스 있음 − 없음) 하면 링크 질량(URDF g)이 빠짐 → c를 URDF 질량 오차와 분리
  최소제곱 (Drive 덱 24쪽 ①): c_j = Σ Δτ_j·ΔI_j / Σ ΔI_j²   (원점을 지나는 직선, 관절 j마다)

같이 출력: 박스 없을 때 c·I_avg / g_URDF = 실제 중력 / URDF 중력 (1이 아니면 URDF 링크 질량·COM 오차)

입력: static_current.py sweep 결과 두 개 (같은 --joint, --poses, --delta로 박스 없이/있게), 실물 URDF
사용: python3 fit_kt_box.py nobox.csv box.csv --urdf robot_description.urdf --arm l --mass 2.3
"""
import argparse
import collections
import csv

import numpy as np
import pinocchio as pin

ap = argparse.ArgumentParser()
ap.add_argument('nobox')
ap.add_argument('box')
ap.add_argument('--urdf', required=True)
ap.add_argument('--arm', default='l')
ap.add_argument('--mass', type=float, required=True, help='박스 질량 [kg] (저울)')
ap.add_argument('--frame', default=None, help='박스를 매단 점의 frame (기본 end_effector_<arm>_link)')
ap.add_argument('--min_dtau', type=float, default=0.3, help='이보다 작은 Δτ [N·m] 자세는 그 관절 적합에서 제외 (신호 < 잡음)')
a = ap.parse_args()

text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
m = pin.buildModelFromXML(text)
d = m.createData()
idx_q = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_q for j in range(1, 8)]
idx_v = [m.joints[m.getJointId(f'arm_{a.arm}_joint{j}')].idx_v for j in range(1, 8)]
fid = m.getFrameId(a.frame or f'end_effector_{a.arm}_link')
G = 9.81


def hold_means(path):
    """자세(target)마다 위·아래 접근 각각의 정지 평균 → 두 방향 평균 (q 7개, I 7개)"""
    g = collections.defaultdict(list)
    for r in csv.DictReader(open(path)):
        if r['phase'] == 'hold':
            g[float(r['target']), r['dir']].append(r)
    out = {}
    if not g:   # 정지만 기록(static_current noise) — 접근 방향 없음 → 정지마찰이 상쇄되지 않음 (오차로 남음)
        rs = [r for r in csv.DictReader(open(path)) if r['phase'] == 'noise']
        if rs:
            print(f'  [{path}] 정지 기록만 있음 — 위·아래 평균 없이 사용 (정지마찰 미상쇄)')
            out['static'] = (np.mean([[float(r[f'q{i}']) for i in range(1, 8)] for r in rs], axis=0),
                             np.mean([[float(r[f'I{i}']) for i in range(1, 8)] for r in rs], axis=0))
        return out
    for tg in sorted({k[0] for k in g}):
        dirs = [g[tg, dr] for dr in ('from_above', 'from_below') if (tg, dr) in g]
        if len(dirs) < 2:
            continue
        q = np.mean([[float(r[f'q{i}']) for i in range(1, 8)] for rs in dirs for r in rs], axis=0)
        I = np.mean([np.mean([[float(r[f'I{i}']) for i in range(1, 8)] for r in rs], axis=0) for rs in dirs], axis=0)
        out[tg] = (q, I)
    return out


def model_at(q7):
    q = pin.neutral(m)
    for k, v in zip(idx_q, q7):
        q[k] = v
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    Jv = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, idx_v]   # base_link 축 방향
    g = pin.computeGeneralizedGravity(m, d, q)[idx_v]
    return Jv, g


NB, BX = hold_means(a.nobox), hold_means(a.box)
common = sorted(set(NB) & set(BX))
if not common:
    raise SystemExit('두 파일에 공통 자세 없음 — 같은 --poses로 측정했는지 확인')
dtau_all, dI_all, rows = [], [], []
print(f'박스 m = {a.mass} kg → 손에 가하는 힘 [0, 0, {a.mass * G:.2f}] N (MR 5.4: Δτ = J_vᵀ F)')
print('자세   관절: Δτ(J_vᵀF)  ΔI(박스−없음)')
for tg in common:
    qb, Ib = BX[tg]
    qn, In = NB[tg]
    Jv, g = model_at(qb)
    dtau = Jv.T @ np.array([0.0, 0.0, a.mass * G])
    dI = Ib - In
    dtau_all.append(dtau)
    dI_all.append(dI)
    rows.append((tg, qb, qn, In, g))
    print(f'{tg if isinstance(tg, str) else f"{tg:+.2f}"}  ' + '  '.join(f'j{j+1}:{dtau[j]:+.2f}/{dI[j]:+.2f}' for j in range(7))
          + f'   |q박스−q없음| 최대 {np.abs(qb - qn).max() * 1000:.1f} mrad')
dtau_all, dI_all = np.array(dtau_all), np.array(dI_all)

print('\n관절별 보정계수 c_j = ΣΔτΔI/ΣΔI² (Δτ가 작은 자세 제외)')
c = {}
for j in range(7):
    use = np.abs(dtau_all[:, j]) >= a.min_dtau
    if use.sum() == 0:
        print(f'  j{j+1}: 신호 부족 (|Δτ| < {a.min_dtau} N·m)')
        continue
    x, y = dI_all[use, j], dtau_all[use, j]
    c[j] = float(x @ y / (x @ x))
    res = y - c[j] * x
    print(f'  j{j+1}: c = {c[j]:.3f}  (자세 {use.sum()}개, 잔차 RMS {np.sqrt(np.mean(res ** 2)):.3f} N·m)')

print('\n박스 없을 때 실제 중력 / URDF 중력 = c·I_avg / g_URDF (|g| ≥ 0.5 N·m 자세만)')
for j, cj in c.items():
    ratios = [cj * In[j] / g[j] for tg, qb, qn, In, g in rows if abs(g[j]) >= 0.5]
    if ratios:
        print(f'  j{j+1}: {np.round(ratios, 2)}  평균 {np.mean(ratios):.2f}')
print('\n해석: c ≈ 1 → ROBOTIS 공칭 환산이 맞음 (어제 1.66배는 URDF 질량 쪽 문제)'
      '\n      c ≈ 1/1.66 ≈ 0.60 → 공칭 환산이 과대 (URDF는 맞음)')
