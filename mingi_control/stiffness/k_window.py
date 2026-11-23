#!/usr/bin/env python3
"""수평 강성 창 — "넣을 수 있는 상한" 안에서 x·y 허용 오차와 팔로워 강성 범위를 다시 산정. 김민기 (11/23).

11/17 문제: 마찰 하한 K ≥ F_fric/δx (δx = 5 mm) = x 13,400 / y 6,490 N/m → 실물에 넣을 수 있는 상한(아래 ①)보다 훨씬 큼.
순서
  ① 상한 K_max: 실물 100 Hz 이산 폐루프 스펙트럼 반경 ρ(Φ) < 1 (gain_design ⑦, 문서 4쪽 두 이산 조건의 관절 공간판).
     MR 11.7.1: 큰 K는 지연·양자화와 겹쳐 진동·불안정 → 넣을 수 있는 K에 상한이 있음. 판정은 ρ < 1 (0.99 여유는 아래 k_max 설명)
     새로: 플랜트 관절 점성을 URDF 0.1 대신 11/17 실측 b_viscous로 (MR (11.20) b > 0, (11.26) 감쇠에 더해짐)
  ② 손끝 마찰력 F_fric,a = |J̄ᵀ| b_static (문서 3·4쪽, J̄ᵀ = Λ J M⁻¹, 관절 부호가 최악으로 겹친 경우)
  ③ 방향별 허용 오차 재산정: 상한 K_max로 버틸 수 있는 오차 δ_a = F_fric,a / K_max (문서 1쪽 e_ss = K⁻¹F)
  ④ 팔로워(베이스 간 오차 Δ_a를 흡수): 손 내력 ≈ F_res,a + K_F,a Δ_a ≤ F_int,max (문서 3쪽 15 N)
       F_res,a = 팔로워가 움직이기 시작하려면 넘어야 하는 마찰력. 보상 없음 = F_fric,a (②), 보상 있음 = 실험값
       → F_fric,a > F_int,max 이면 K_F를 아무리 줄여도 내력 15 N을 못 지킴 = 마찰 보상이 필요한 이유
     리더 오차 (직렬 스프링, gain_design ⑥): e_L,a = Δ_a K_F/(K_L + K_F) + F_res,L,a / K_L
사용: python3 k_window.py --urdf ../../real_logs/robot_description.urdf [--delta 0.10 0.25]
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import gain_design as gd  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--urdf', required=True)
ap.add_argument('--delta', type=float, nargs=2, default=None, help='베이스 간 최대 오차 Δx Δy [m] (curve_base_error.py 결과)')
ap.add_argument('--F_res', type=float, nargs=2, default=None, help='보상 있을 때 팔로워 손 마찰력 x y [N] (실험값)')
a = ap.parse_args()

S = {k: v for k, (v, _) in gd.SPEC.items()}
G = yaml.safe_load(open(os.path.join(HERE, '../gains.yaml')))
FR = yaml.safe_load(open(os.path.join(HERE, '../../src/coop_carry_bringup/config/robots.yaml')))['joint_friction']
md = gd.model_at_grasp(a.urdf)
T = 1.0 / S['f_ctrl']
Kz, Krot, wn_null = G['K_pos'][2], G['K_rot'], G['null_omega']
wn_max = G['omega_max']


def rho_c(Kxy, c, all_eigs=False):
    """gain_design.rho와 같은 식, 플랜트 관절 점성 c를 관절별 벡터로"""
    from scipy.linalg import expm
    out = 0.0
    for s in 'rl':
        J, M = md[s]['J'], md[s]['M'] + np.diag(S['armature_real'])
        Mi = np.linalg.inv(M)
        L = np.linalg.inv(J @ Mi @ J.T + 1e-6 * np.eye(6))
        N = np.eye(7) - J.T @ L @ J @ Mi
        Ld = np.diag(L)
        K = np.minimum(np.array([Kxy, Kxy, Kz] + list(Krot), float), Ld * wn_max ** 2)
        D = np.minimum(2 * np.sqrt(Ld * K), 2 * Ld / T)
        Kq = J.T @ np.diag(K) @ J + N @ np.diag(np.diag(M) * wn_null ** 2)
        Dq = J.T @ np.diag(D) @ J + N @ np.diag(2 * np.diag(M) * wn_null)
        cs = np.diag(c[s] if isinstance(c, dict) else [c] * 7)
        A = np.block([[np.zeros((7, 7)), np.eye(7)], [np.zeros((7, 7)), -Mi @ cs]])
        B = np.vstack([np.zeros((7, 7)), Mi])
        E = expm(np.block([[A, B], [np.zeros((7, 21))]]) * T)
        Phi = E[:14, :14] - E[:14, 14:] @ np.hstack([Kq, Dq])
        z = np.linalg.eigvals(Phi)
        if all_eigs:
            out = z if isinstance(out, float) else np.concatenate([out, z])
        else:
            out = max(out, float(np.abs(z).max()))
    return out


def k_max(c):
    """ρ(Φ) < 1 (이산 안정)인 최대 K. 실측 점성은 느린 실수 극점(|z| ≈ 0.997, 안정)을 만들어 gain_design의 0.99 여유 기준이
    안정·불안정을 구분하지 못함 → 정의 그대로 ρ < 1 로 판정 (11/23 확인: 경계에서 음의 실수 극점이 −1을 넘음 = D T/Λ > 2)"""
    lo, hi = 50.0, 5000.0
    assert rho_c(lo, c) < 1.0 <= rho_c(hi, c)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if rho_c(mid, c) < 1.0 else (lo, mid)
    return lo


print('① 넣을 수 있는 수평 강성 상한 K_max (실물 100 Hz, 이산 안정 ρ(Φ) < 1, gain_design ⑦의 판정식)')
km_old = k_max(0.1)
b_real = {s: FR[s]['damping'] for s in 'rl'}
km_new = k_max(b_real)
print(f'   플랜트 점성 0.1 (URDF, 11/5 설계)        : K_max = {km_old:.0f} N/m')
print(f'   플랜트 점성 = 11/17 실측 b_viscous        : K_max = {km_new:.0f} N/m')
KM = min(km_old, km_new)
print(f'   → 사용 K_max = {KM:.0f} N/m (둘 중 작은 값). 실측 점성을 넣어도 경계는 거의 같음: 경계는 손목의 D T/Λ = 2 (감쇠 상한)에서 정해짐')
z = rho_c(500.0, b_real, all_eigs=True)
zr = z[np.abs(z.imag) < 1e-9]
zs = zr[np.argmax(np.abs(zr))]
print(f'   판정을 ρ ≤ 0.99 → ρ < 1 로 바꾼 이유: 실측 점성을 넣으면 가장 느린 극점 z = {zs.real:.4f} (양의 실수, 안정)')
print(f'     → 시정수 τ = −T/ln|z| = {-T / np.log(abs(zs)):.1f} s (점성이 큰 관절이 천천히 제자리로 가는 모드). 0.99 기준은 이 느린 안정 모드를 불안정으로 오판')
zn = zr[zr.real < 0]
if len(zn):
    print(f'     경계에서 커지는 것은 음의 실수 극점 (K 500: {zn.real.min():.3f}) → K ≳ {KM:.0f}에서 −1을 넘음 = 이산 감쇠 상한 D T/Λ = 2')

print('\n② 손끝 마찰력 F_fric = |J̄ᵀ| b_static (파지 자세, 관절 부호 최악)  [N]   (base_link: x = 상대 로봇 방향, y = 주행 방향)')
F = {}
for s in 'rl':                     # 11/17 k_lower_bound.py와 같은 식: J_v(3×7)만, Λ_v = (J_v M⁻¹ J_vᵀ)⁻¹, J̄ᵀ = Λ_v J_v M⁻¹
    Jv, Mi = md[s]['J'][:3], np.linalg.inv(md[s]['M'])
    F[s] = np.abs(np.linalg.inv(Jv @ Mi @ Jv.T) @ Jv @ Mi) @ np.array(FR[s]['friction'])
    print(f'   {s}팔: x {F[s][0]:.1f}, y {F[s][1]:.1f}, z {F[s][2]:.1f}')
Fw = np.maximum(F['r'], F['l'])

print(f'\n③ 방향별 허용 오차 재산정: δ_a = F_fric,a / K_max  (K_max = {KM:.0f} N/m)')
print(f'   11/17 (δ = 5 mm 고정)  : 필요한 K = x {Fw[0] / 0.005:.0f}, y {Fw[1] / 0.005:.0f} N/m  → 상한 초과')
print(f'   재산정 (K = K_max 고정): δx = {Fw[0] / KM * 1000:.0f} mm, δy = {Fw[1] / KM * 1000:.0f} mm  (보상 없을 때 정지 마찰이 남기는 손끝 오차)')
tb = yaml.safe_load(open(os.path.join(HERE, '../../src/coop_carry_bringup/config/robots.yaml')))['box']['table_size']
print(f'   비교: 받침대 윗면 반폭 x {tb[0] / 2 * 1000:.0f} mm, y {tb[1] / 2 * 1000:.0f} mm (박스 중심이 이 안이면 받침대 위에 섬, robots.yaml table_size)')

print(f'\n④ 팔로워 강성 범위 (손 내력 ≤ F_int,max = {S["F_int_max"]:.0f} N, 문서 3쪽)')
print(f'   보상 없음: 팔로워가 움직이려면 손 힘이 F_fric를 넘어야 함 → x {Fw[0]:.1f} N, y {Fw[1]:.1f} N', end='')
print('  > 15 N → K_F와 무관하게 내력 상한 초과 (마찰 보상 필요)' if min(Fw[:2]) > S['F_int_max'] else '')
if a.delta:
    dx, dy = a.delta
    print(f'   베이스 간 최대 오차 Δx = {dx * 1000:.0f} mm, Δy = {dy * 1000:.0f} mm (곡선 주행 실측)')
    Fr = a.F_res if a.F_res else [0.0, 0.0]
    rec = []
    for ax, D, f, dl in (('x', dx, Fr[0], Fw[0] / KM), ('y', dy, Fr[1], Fw[1] / KM)):
        hi = (S['F_int_max'] - f) / D if D > 0 else float('inf')
        k = math.floor(min(hi, KM) / 10) * 10                              # 10 단위 내림 (GAIN_RATIONALE ⑥과 같은 규칙)
        rec.append(k)
        why = '내력 조건' if hi < KM else f'Δ{ax}가 작아 내력 조건이 상한을 넘음 → 넣을 수 있는 상한 K_max'
        print(f'   {ax}: F_res = {f:.1f} N (보상 있음) → K_F,{ax} ≤ (15 − F_res)/Δ{ax} = {hi:.0f} N/m, K_max {KM:.0f} → K_F,{ax} = {k} N/m ({why})')
    KL = G['leader_K_pos'][0]
    eL = [D * K / (KL + K) * 1000 for D, K in zip((dx, dy), rec)]
    print(f'   확인 — 리더 오차 Δ K_F/(K_L + K_F) (GAIN_RATIONALE ⑥): x {eL[0]:.1f} mm, y {eL[1]:.1f} mm '
          f'≤ ③ 허용 오차 x {Fw[0] / KM * 1000:.0f}, y {Fw[1] / KM * 1000:.0f} mm')
    print(f'   → 재산정 K_F = ({rec[0]}, {rec[1]}) N/m')
    KL = G['leader_K_pos'][0]
    for KF in (10, 20, 40, 75, 150, 500):
        eL = [D * KF / (KL + KF) * 1000 for D in (dx, dy)]
        Fi = [D * KL * KF / (KL + KF) for D in (dx, dy)]
        print(f'     K_F = {KF:4d}: 내력(마찰 제외) x {Fi[0]:5.1f} y {Fi[1]:5.1f} N, 리더 오차 x {eL[0]:5.1f} y {eL[1]:5.1f} mm')
