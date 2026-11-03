#!/usr/bin/env python3
"""게인 설계 — 문서 #5~#9를 숫자로. 김민기.

  [문서 #5] ωn = √(K/Λ),  D = 2ζ√(ΛK),  ζ=1에서 T_s ≈ 5.8/ωn  (MR 11.2.2)
  [문서 #6] 예제: Λ=4 kg, F=20 N, e_ss=0.02 m → K=1000 N/m, ωn≈15.81, D≈126.5, T_s≈0.37 s  (T6)
  [문서 #7] K_z ≥ m g / Δz_max,  K_xy ≤ F_internal,max / Δx   (MR 11.7.1)
  [문서 #8] 이산화 상한 ωn ≤ 2πf/10 → K_max = Λ_min (2πf/10)²,  감쇠 상한 D < 2Λ/T
  [문서 #9] 무게 피드포워드가 있으면 K_z는 무게 추정 오차만 감당
  쥐기(S5): F_sq = K_y δ,  미끄러지지 않음 2 μ F_sq ≥ m g,  F_sq ≤ F_internal,max

[해석 — 문서 식을 손 하나 기준으로 적용] 박스 무게는 두 손이 나눠 지므로 손 하나의 K_z 하한은 (m g/2)/Δz_max.

사용: python3 gain_design.py [--urdf control.urdf]   (모델 없으면 T6만)
"""
import argparse
import math

import numpy as np

G = 9.81


def t6():
    """[문서 #6] 예제 재현"""
    Lam, F, e_ss, zeta = 4.0, 20.0, 0.02, 1.0
    K = F / e_ss
    wn = math.sqrt(K / Lam)
    D = 2 * zeta * math.sqrt(Lam * K)
    Ts = 5.8 / wn
    print('== T6 문서 예제 재현 (Λ=4 kg, F=20 N, e_ss=0.02 m, ζ=1)')
    print(f'K = F/e_ss = {K:.0f} N/m   ωn = √(K/Λ) = {wn:.2f} rad/s   D = 2ζ√(ΛK) = {D:.1f} N·s/m   T_s ≈ 5.8/ωn = {Ts:.3f} s')
    ok = abs(K - 1000) < 1e-9 and abs(wn - 15.81) < 0.01 and abs(D - 126.5) < 0.1 and abs(Ts - 0.37) < 0.005
    print('T6:', 'PASS' if ok else 'FAIL', '(문서값 K=1000, ωn≈15.81, D≈126.5, T_s≈0.37)')
    return ok


def lambda_at(urdf, side, p_target, lift=0.0):
    """박스 잡는 자세의 Λ = (J M⁻¹ Jᵀ)⁻¹ — [문서 #1]. 자세는 감쇠 최소제곱 IK(MR 6.2 수치 IK)."""
    import pinocchio as pin
    m = pin.buildModelFromXML(open(urdf).read())
    d = m.createData()
    q = pin.neutral(m)
    if m.existJointName('lift_joint'):
        q[m.joints[m.getJointId('lift_joint')].idx_q] = lift
    names = [f'arm_{side}_joint{i}' for i in range(1, 8)]
    qi = [m.joints[m.getJointId(n)].idx_q for n in names]
    vi = [m.joints[m.getJointId(n)].idx_v for n in names]
    fid = m.getFrameId(f'end_effector_{side}_link')
    target = pin.SE3(np.eye(3), np.array(p_target))
    sg = 1 if side == 'r' else -1
    for k, val in zip(qi, np.radians([-60, 0, 0, 60, 0, 30, 0]) * [1, sg, sg, 1, sg, 1, sg]):
        q[k] = val
    for _ in range(500):
        pin.framesForwardKinematics(m, d, q)
        err = pin.log6(d.oMf[fid].actInv(target)).vector
        if np.linalg.norm(err) < 1e-6:
            break
        J = pin.computeFrameJacobian(m, d, q, fid, pin.ReferenceFrame.LOCAL)[:, vi]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
        for j, k in enumerate(qi):
            q[k] = np.clip(q[k] + 0.5 * dq[j], m.lowerPositionLimit[k], m.upperPositionLimit[k])
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    J = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, vi]
    M = pin.crba(m, d, q)
    M = (np.triu(M) + np.triu(M, 1).T)[np.ix_(vi, vi)] + np.diag([0, 0, 0, 0, 0.05, 0.05, 0.05])
    Lam = np.linalg.inv(J @ np.linalg.inv(M) @ J.T + 1e-6 * np.eye(6))
    err = np.linalg.norm(pin.log6(d.oMf[fid].actInv(target)).vector[:3])
    return np.diag(Lam)[:3], np.degrees(q[qi]), err


def design(a):
    print('\n== 입력')
    for k, v in vars(a).items():
        print(f'  {k} = {v}')
    lam = {}
    if a.urdf:
        for side, y in (('r', -a.hand_y), ('l', a.hand_y)):
            L, qdeg, err = lambda_at(a.urdf, side, [a.hand_x, y, a.hand_z])
            lam[side] = L
            print(f'  [{side}] 잡는 자세 IK 오차 {err*1000:.2f} mm, q(deg) {np.round(qdeg, 1)}')
            print(f'  [{side}] diag(Λ)_xyz = {np.round(L, 3)} kg   [문서 #1]')
    Lmin = min(min(L) for L in lam.values()) if lam else 2.0
    Lmax = max(max(L) for L in lam.values()) if lam else 2.0
    mg2 = a.m_box * G / 2
    print('\n== 범위 (손 하나 기준)')
    Kz_min = mg2 / a.dz_max
    Kz_min_ff = a.weight_err * mg2 / a.dz_max
    Kxy_max = a.F_int_max / a.dx
    K_max = Lmin * (2 * math.pi * a.f_ctrl / 10) ** 2
    D_max = 2 * Lmin * a.f_ctrl
    print(f'  [문서 #7] K_z ≥ (m g/2)/Δz_max = {Kz_min:.0f} N/m  (피드포워드 없을 때)')
    print(f'  [문서 #9] K_z ≥ {a.weight_err:.0%}·(m g/2)/Δz_max = {Kz_min_ff:.0f} N/m  (피드포워드 있을 때)')
    print(f'  [문서 #7] K_xy ≤ F_int,max/Δx = {Kxy_max:.0f} N/m')
    print(f'  [문서 #8] K ≤ Λ_min(2πf/10)² = {K_max:.0f} N/m  (Λ_min={Lmin:.3f}, f={a.f_ctrl} Hz)')
    print(f'  [문서 #8] D < 2Λ_min/T = {D_max:.0f} N·s/m')
    print('\n== 추천값과 확인')
    Kz, Ky = a.Kz, a.Ky
    for name, K, lo, hi in (('K_z', Kz, Kz_min_ff, K_max), ('K_y', Ky, 0.0, min(Kxy_max, K_max))):
        flag = 'OK' if lo <= K <= hi else '위반'
        print(f'  {name} = {K:.0f} N/m  범위 [{lo:.0f}, {hi:.0f}]  {flag}')
    for L, axis in ((Lmin, 'Λ_min'), (Lmax, 'Λ_max')):
        for name, K in (('K_y', Ky), ('K_z', Kz)):
            wn = math.sqrt(K / L)
            D = 2 * math.sqrt(L * K)
            print(f'  [문서 #5] {name}, {axis}={L:.3f}: ωn={wn:.1f} rad/s, D={D:.1f} N·s/m (상한 {D_max:.0f}), T_s≈{5.8/wn:.3f} s')
    F_sq = Ky * a.delta
    print(f'\n== 쥐기: F_sq = K_y δ = {Ky:.0f}×{a.delta} = {F_sq:.1f} N')
    need = a.m_box * G / (2 * a.mu)
    print(f'  미끄럼 방지 2μF_sq ≥ m g → F_sq ≥ {need:.1f} N : {"OK" if F_sq >= need else "부족"}')
    print(f'  내력 상한 F_sq ≤ {a.F_int_max:.0f} N : {"OK" if F_sq <= a.F_int_max else "초과"}')
    print(f'  δ 범위 = [{need / Ky * 1000:.1f}, {a.F_int_max / Ky * 1000:.1f}] mm  (K_y={Ky:.0f})')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf')
    ap.add_argument('--hand_x', type=float, default=0.40)
    ap.add_argument('--hand_y', type=float, default=0.15)
    ap.add_argument('--hand_z', type=float, default=0.85)
    ap.add_argument('--m_box', type=float, default=1.0)
    ap.add_argument('--dz_max', type=float, default=0.01, help='허용 처짐 [m]')
    ap.add_argument('--weight_err', type=float, default=0.2, help='무게 추정 오차 비율')
    ap.add_argument('--F_int_max', type=float, default=30.0, help='내력(쥐는 힘) 상한 [N]')
    ap.add_argument('--dx', type=float, default=0.02, help='손 위치 오차(베이스·팔) [m]')
    ap.add_argument('--delta', type=float, default=0.02, help='쥐는 오프셋 [m]')
    ap.add_argument('--mu', type=float, default=1.0)
    ap.add_argument('--f_ctrl', type=float, default=500.0, help='토크 루프 주기 [Hz]')
    ap.add_argument('--Ky', type=float, default=500.0)
    ap.add_argument('--Kz', type=float, default=1000.0)
    a = ap.parse_args()
    t6()
    design(a)
