#!/usr/bin/env python3
"""게인 설계 — 사양(왜 이 숫자) → 문서 식 → 값. 결과를 gains.yaml로 저장하면 impedance_node·transport_scenario가 읽는다. 김민기.

근거: 「작업공간 임피던스 제어 PD gain 설정」(이하 '문서 n쪽'), MR 8.6·11.2.2·11.4.1·11.7.1, Khatib(1987).
순서 = 실물에서 켜는 순서
  ① 중력 보상 → ② 무게 피드포워드 → ③ 제어 주기 상한 → ④ 마찰 deadband 하한 → ⑤ K_z → ⑥ K_xy(리더·팔로워)
  → ⑦ D·정착시간·이산 감쇠 상한 → ⑧ 회전·영공간 → ⑨ 그리퍼 쥐는 힘(⑥의 내력 상한과 같은 숫자에서)
강성은 모두 문서 1쪽 정적 순응 e_ss = K⁻¹F 에서 K = F/e — 어떤 힘 F를 어떤 오차 e 안에 버티게 할지가 사양.
Λ, J̄ᵀ, 그리퍼 기구비는 실제 파지 자세(11/4 파지 직후 로그 관절각)에서 Pinocchio로 계산.

사용: python3 gain_design.py --urdf control.urdf
  control.urdf: docker compose -f docker/compose.yaml exec robot1 cat /run/coop/robot1/control.urdf > control.urdf
"""
import argparse
import math
import os

import numpy as np
import yaml

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))

# ---- 사양: (값, 이유) — 보고 때 그대로 읽는 표 ----
SPEC = {
    'f_ctrl': (100.0, '실물 ros2_control update_rate 100 Hz (ffw_sg2_follower_ai_hardware_controller.yaml). 게인 설계 기준 T'),
    'f_sim': (500.0, '시뮬 루프 주기 [Hz] (robots.yaml state_rate). 시뮬 플랜트는 armature가 없어 100 Hz에서 이산 불안정 → ⑦ 판정'),
    'armature_real': ([0, 0, 0, 0, 0.05, 0.05, 0.05], '실물 모터 회전자 반영 관성 [kg·m²] — impedance_node armature_real (손목 5·6·7)'),
    'm_box': (2.0, '시뮬 박스 질량 [kg] (robots.yaml box.mass)'),
    'n_hands': (4, '두 로봇 × 양팔 → 손 하나가 박스 무게의 1/4'),
    'weight_err': (0.20, '무게 추정 오차 비율 — 실물 박스는 내용물에 따라 질량이 달라 ±20% 가정 → 피드포워드 뒤 남는 힘'),
    'dz_max': (0.005, '허용 수직 처짐 [m] (문서 3쪽 5 mm). 내려놓기 마지막 구간 1 cm의 절반 → 받침대에 수평으로 닿음'),
    'base_err': (0.0575, '두 베이스 간 거리 오차 Δ [m] — 현대 스워브 2 m 주행 실측 최대 57.5 mm (base_error.py, 11/5, 정렬 단계 적용 후)'),
    'F_int_max': (15.0, '손당 허용 수평 내력 [N] (문서 3쪽 15 N). 같은 숫자로 그리퍼 쥐는 힘을 설계(⑨) → 이 이하면 손잡이가 미끄러지지 않음'),
    'dx_precision': (0.005, '목표 정밀도 δx [m] — 리더 경로 정밀도이자 마찰 deadband 허용치(문서 3·4쪽). z 허용 처짐과 같은 5 mm'),
    'tau_stiction_sim': (0.0, '시뮬 플랜트 관절 정지마찰 [N·m] = 0: 물리 모델 URDF <dynamics>에 damping 0.1만 있고 friction 없음(11/5 확인)'),
    'theta_max_deg': (2.0, '허용 손 자세 오차 [°] (박스 기울기 판정 기준과 같음)'),
    'mu': (1.0, '손끝 패드-손잡이 마찰계수 (robots.yaml box.mu, finger_mu)'),
    'grip_SF': (2.0, '쥐는 힘 안전율 (마찰계수·접촉 불확실성)'),
    'r_handle': (0.0205, '손끝 기준점(end_effector) ↔ 손잡이 중심 거리 [m] = grasp_sequence tool_offset_x (RH-P12 손가락 끝 오프셋)'),
    'zeta': (1.0, '임계감쇠 — 오버슈트 0: 손이 박스·상대 로봇을 밀고 들어가지 않음 (문서 2쪽 "ζ=1로 시작")'),
}
# 11/4 파지 직후(task 전환 순간) robot1 관절각 [rad] — 로그 1104_205635_task_robot1.csv 첫 행
Q_GRASP = {'r': [-0.3888, -0.7111, 1.9041, -1.37, -0.3074, 0.6082, -1.5823],
           'l': [0.5106, 0.1599, -0.0832, -1.266, 1.7479, -0.0301, -0.8224]}


def t6():
    """[문서 2쪽 예제] Λ=4 kg, F=20 N, e_ss=0.02 m, ζ=1 → K=1000, ωn≈15.81, D≈126.5, T_s≈0.37 s (check_theory.py가 사용)"""
    Lam, F, e_ss, zeta = 4.0, 20.0, 0.02, 1.0
    K = F / e_ss
    wn = math.sqrt(K / Lam)
    D = 2 * zeta * math.sqrt(Lam * K)
    Ts = 5.8 / wn
    print(f'== T6 문서 예제: K={K:.0f} N/m, ωn={wn:.2f} rad/s, D={D:.1f} N·s/m, T_s={Ts:.3f} s')
    ok = abs(K - 1000) < 1e-9 and abs(wn - 15.81) < 0.01 and abs(D - 126.5) < 0.1 and abs(Ts - 0.37) < 0.005
    print('T6:', 'PASS' if ok else 'FAIL')
    return ok


def model_at_grasp(urdf):
    """[문서 1·2쪽] Λ = (J M⁻¹ Jᵀ)⁻¹, J̄ᵀ = Λ J M⁻¹ (Khatib) — 팔 7축, base_link 축(LOCAL_WORLD_ALIGNED)"""
    import pinocchio as pin
    m = pin.buildModelFromXML(open(urdf).read())
    d = m.createData()
    q = pin.neutral(m)
    for s, qs in Q_GRASP.items():
        for i, v in enumerate(qs):
            q[m.joints[m.getJointId(f'arm_{s}_joint{i + 1}')].idx_q] = v
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    M = pin.crba(m, d, q)
    M = np.triu(M) + np.triu(M, 1).T
    out = {}
    for s in 'rl':
        vi = [m.joints[m.getJointId(f'arm_{s}_joint{i}')].idx_v for i in range(1, 8)]
        J = pin.getFrameJacobian(m, d, m.getFrameId(f'end_effector_{s}_link'), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, vi]
        Ma = M[np.ix_(vi, vi)]
        Mi = np.linalg.inv(Ma)
        L = np.linalg.inv(J @ Mi @ J.T + 1e-6 * np.eye(6))
        out[s] = dict(Lam=np.diag(L), JbT=L @ J @ Mi, J=J, M=Ma, N=np.eye(7) - J.T @ L @ J @ Mi)

    def gap(q1):
        """손끝 패드 안쪽 면 간격 (mimic 4관절 같은 각) — 쥐는 힘 가상일 계산용"""
        qq = pin.neutral(m)
        for j in range(1, 5):
            qq[m.joints[m.getJointId(f'gripper_r_joint{j}')].idx_q] = q1
        pin.framesForwardKinematics(m, d, qq)
        pr = d.oMf[m.getFrameId('gripper_r_rh_p12_rn_r2')].act(np.array([0, -0.0039, 0.0126]))
        pl = d.oMf[m.getFrameId('gripper_r_rh_p12_rn_l2')].act(np.array([0, 0.0039, 0.0126]))
        return float(np.linalg.norm(pr - pl))
    q_c = next(q1 for q1 in np.arange(0.5, 1.1, 0.001) if gap(q1) <= 0.030)    # 손잡이 두께 30 mm에 닿는 각
    out['grip'] = dict(q_contact=float(q_c), dgap_dq=(gap(q_c + 1e-4) - gap(q_c)) / 1e-4)
    return out


def modes(md, Kp, Kr, w_null, Dp=None, Dr=None):
    """관절 공간 폐루프 고유진동수: M q̈ = −(Jᵀ K J + N K_q) q (작은 변위 선형화, 중력은 g(q)로 상쇄).
    축별 Λ 대각 근사는 가벼운 손목 관절(M_ii ≈ 0.01)의 모드를 못 봄 → ③ 상한을 실제 모드에 적용 (11/5 시뮬 발산으로 확인)"""
    w, dT = 0.0, 0.0
    for s in 'rl':
        J, M, N = md[s]['J'], md[s]['M'], md[s]['N']
        Kq = N @ np.diag(np.diag(M) * w_null ** 2)
        A = np.linalg.solve(M, J.T @ np.diag(list(Kp) + list(Kr)) @ J + Kq)
        w = max(w, float(np.sqrt(np.abs(np.linalg.eigvals(A).real)).max()))
        if Dp is not None:
            B = np.linalg.solve(M, J.T @ np.diag(list(Dp) + list(Dr)) @ J + N @ np.diag(2 * np.diag(M) * w_null))
            dT = max(dT, float(np.abs(np.linalg.eigvals(B).real).max()))
    return w, dT


def rho(md, Kp, Kr, w_null, T, armature, c=0.1, wn_cap=None):
    """이산 시간 폐루프 스펙트럼 반경 ρ(Φ) — ρ < 1이면 안정. 문서 4쪽 두 조건(ωn ≤ ωs/10, D T/Λ < 2)을 한 번에, 관절 공간 전체로.
    플랜트: (M + armature) q̈ + c q̇ = τ (관절 점성 c = URDF damping 0.1), 제어: 매 T마다 계산한 τ를 T 동안 유지(영차 홀드)
    τ = −(Jᵀ K J + N K_q) q − (Jᵀ D J + N D_q) q̇,  D = min(2√(ΛK), 2Λ/T) — 노드와 같은 식"""
    from scipy.linalg import expm
    out = 0.0
    for s in 'rl':
        J, M0 = md[s]['J'], md[s]['M']
        M = M0 + np.diag(armature)
        Mi = np.linalg.inv(M)
        L = np.linalg.inv(J @ Mi @ J.T + 1e-6 * np.eye(6))
        N = np.eye(7) - J.T @ L @ J @ Mi
        Ld = np.diag(L)
        K = np.array(list(Kp) + list(Kr), float)
        if wn_cap:
            K = np.minimum(K, Ld * wn_cap ** 2)
        D = np.minimum(2 * np.sqrt(Ld * K), 2 * Ld / T)
        Kq = J.T @ np.diag(K) @ J + N @ np.diag(np.diag(M) * w_null ** 2)
        Dq = J.T @ np.diag(D) @ J + N @ np.diag(2 * np.diag(M) * w_null)
        n = 7
        A = np.block([[np.zeros((n, n)), np.eye(n)], [np.zeros((n, n)), -Mi * c]])
        B = np.vstack([np.zeros((n, n)), Mi])
        E = expm(np.block([[A, B], [np.zeros((n, 3 * n))]]) * T)
        Phi = E[:2 * n, :2 * n] - E[:2 * n, 2 * n:] @ np.hstack([Kq, Dq])
        out = max(out, float(np.abs(np.linalg.eigvals(Phi)).max()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--write', default=os.path.join(HERE, 'gains.yaml'))
    a = ap.parse_args()
    S = {k: v for k, (v, _) in SPEC.items()}
    print('== 사양 (값 — 이유)')
    for k, (v, why) in SPEC.items():
        print(f'  {k:17s} = {str(v):<7} {why}')
    md = model_at_grasp(a.urdf)
    L_min = np.minimum(md['r']['Lam'], md['l']['Lam'])          # 축별, 두 손 중 가벼운 쪽이 상한을 결정
    print('\n== 파지 자세 모델 (Pinocchio)')
    for s in 'rl':
        print(f'  [{s}] diag Λ  x y z [kg] = {np.round(md[s]["Lam"][:3], 3)},  rx ry rz [kg·m²] = {np.round(md[s]["Lam"][3:], 3)}')
    F_hand = S['m_box'] * G / S['n_hands']
    T = 1.0 / S['f_ctrl']

    print('\n① 중력 보상 [문서 2·5쪽]  τ += g(q). Pinocchio RNEA(q,0,0)은 중력을 지탱하는 토크라 +g. 게인 없음')
    print(f'② 무게 피드포워드 [문서 3쪽]  τ += J_vᵀ[0,0,F]ᵀ, 손당 F = m g/{S["n_hands"]} = {F_hand:.3f} N → K_z는 추정 오차만 버팀')

    wn_max = 2 * math.pi * S['f_ctrl'] / 10
    K_max = L_min * wn_max ** 2
    print(f'\n③ 제어 주기 상한 [문서 4쪽, MR 외 경험칙]  f = {S["f_ctrl"]:.0f} Hz, T = {T} s → ωn ≤ 2πf/10 = {wn_max:.1f} rad/s')
    print('   이유: 디지털 제어는 T마다 계산해 다음 T 동안 유지(영차 홀드) → 평균 T/2 지연 = 위상 지연 ωT/2.')
    print('   ω = ωs/10이면 지연이 π/10 = 18°로, ζ=1 연속시간 설계(오버슈트 0)가 이산 구현에서도 유지됨. 더 빠르면 위상 여유가 줄어 진동')
    print(f'   축별 K_max = Λ_min ωn,max²: x {K_max[0]:.0f}, y {K_max[1]:.0f}, z {K_max[2]:.0f} N/m, 회전 {np.round(K_max[3:], 1)} N·m/rad')

    Kxy_lo = 0.0
    JbT_abs = np.maximum(np.abs(md['r']['JbT']).sum(1), np.abs(md['l']['JbT']).sum(1))   # 관절 τ_s가 최악 부호로 겹칠 때
    F_fric = JbT_abs * S['tau_stiction_sim']
    K_fric = F_fric[:3] / S['dx_precision']
    Kxy_lo = max(K_fric[0], K_fric[1])
    print(f'\n④ 마찰 deadband 하한 [문서 3·4쪽]  F_fric,tip = J̄ᵀ τ_stiction,  K_min ≥ F_fric,tip / δx  (δx = {S["dx_precision"]*1000:.0f} mm)')
    print(f'   시뮬: τ_s = {S["tau_stiction_sim"]} (플랜트에 정지마찰 없음) → F_fric = 0 → K_min,fric = 0, 이 하한은 시뮬에서 비활성')
    print(f'   실물 감도(측정 전): 모든 관절 τ_s = 0.1 N·m이면 손끝 F_fric (x y z) = {np.round(JbT_abs[:3] * 0.1, 2)} N'
          f' → K_min = {np.round(JbT_abs[:3] * 0.1 / S["dx_precision"])} N/m')
    print('   실물: 관절마다 전류를 천천히 올려 움직이기 시작하는 토크 = τ_s 측정 → 이 식에 대입 (calib 절차)')

    F_res_z = S['weight_err'] * F_hand
    Kz_lo = max(F_res_z / S['dz_max'], K_fric[2])
    Kz = float(math.ceil(2 * Kz_lo / 50) * 50)
    print(f'\n⑤ K_z [문서 3쪽]  K_z ≥ F/Δz_max')
    print(f'   피드포워드 없으면 F = m g/4 = {F_hand:.2f} N → K_z ≥ {F_hand / S["dz_max"]:.0f} N/m')
    print(f'   피드포워드 있으면 F = {S["weight_err"]:.0%}·m g/4 = {F_res_z:.3f} N → K_z ≥ {F_res_z / S["dz_max"]:.0f} N/m (마찰 하한 {K_fric[2]:.0f})')
    print(f'   설정 K_z = {Kz:.0f} N/m (하한의 2배: 무게 오차가 가정의 2배여도 처짐 5 mm 이내). 상한 {K_max[2]:.0f} N/m → 충돌 없음')
    print('   리더·팔로워 같은 K_z: 같은 하중에 같은 처짐 → 박스 수평 유지 (문서 3쪽 "K_z는 둘 다")')

    # ⑧의 회전 강성을 먼저 정해야 ⑥ 상한(관절 공간 모드)이 정해짐
    th = math.radians(S['theta_max_deg'])
    M_res = F_hand * S['r_handle']
    Krot_lo = max(M_res / th, float((F_fric[3:] / th).max()))
    Krot = [float(math.ceil(1.5 * Krot_lo))] * 3
    wn_null = math.sqrt(Kz / max(md['r']['Lam'][2], md['l']['Lam'][2]))
    KF_hi = S['F_int_max'] / S['base_err']
    arm_real = S['armature_real']
    lo, hi = 50.0, float(K_max[0])
    for _ in range(30):                                    # 실물(100 Hz, armature 포함) ρ(Φ) ≤ 0.99 가 되는 최대 K_L,xy (이분법)
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if rho(md, [mid, mid, Kz], Krot, wn_null, T, arm_real, wn_cap=wn_max) <= 0.99 else (lo, mid)
    KL = float(math.floor(lo / 50) * 50)
    lo_modes = lo
    KF_prec = S['dx_precision'] * KL / (S['base_err'] - S['dx_precision'])
    KF = float(math.floor(min(KF_hi, KF_prec) / 10) * 10)
    F_int = S['base_err'] * KL * KF / (KL + KF)
    print(f'\n⑥ K_xy [문서 3쪽]  베이스 간 거리 오차 Δ = {S["base_err"]*1000:.1f} mm 먼저 산정(실측)')
    print(f'   리더 상한 — 축별 근사 K ≤ Λ_ii ωn,max² (x {K_max[0]:.0f} N/m)만으로는 부족: 가벼운 손목 관절(M_ii ≈ 0.01 kg·m²)로')
    print(f'     옮겨진 강성·감쇠가 관절 공간에서 더 빠른 모드를 만듦 (K_L 3000이면 최고 모드 {modes(md, [3000, 3000, Kz], Krot, wn_null)[0]:.0f} rad/s,'
          ' 11/5 시뮬 100 Hz에서 발산 확인)')
    print(f'   → 문서 4쪽 두 이산 조건을 관절 공간 전체에 적용한 엄밀판: 영차 홀드 이산 폐루프 스펙트럼 반경 ρ(Φ) < 1 (여유 0.99)')
    print(f'     실물 플랜트(T = {T} s, 손목 armature {arm_real[4]} kg·m²)에서 K_L,xy ≤ {lo_modes:.0f} N/m → 설정 K_L = {KL:.0f}')
    print(f'   팔로워 상한 ①: Δ를 혼자 다 흡수해도 내력 K_F·Δ ≤ F_int,max → K_F ≤ {S["F_int_max"]:.0f}/{S["base_err"]} = {KF_hi:.0f} N/m')
    print(f'   팔로워 상한 ②: 리더가 받는 내력으로 리더 오차 ≤ δx → Δ K_F/(K_L+K_F) ≤ δx → K_F ≤ δx K_L/(Δ−δx) = {KF_prec:.0f} N/m')
    print(f'   팔로워 하한: 마찰 {Kxy_lo:.0f} (시뮬 0) → 설정 K_F,xy = {KF:.0f} N/m')
    print(f'   확인(직렬 스프링): 실제 내력 Δ·K_L K_F/(K_L+K_F) = {F_int:.1f} N ≤ {S["F_int_max"]:.0f} N,'
          f' 리더 오차 {F_int / KL * 1000:.1f} mm ≤ {S["dx_precision"]*1000:.0f} mm, 박스 추종 비율 K_L/(K_L+K_F) = {KL / (KL + KF):.2f}')
    print('\n⑦ 감쇠·정착시간 [문서 2·4쪽]  D = 2ζ√(ΛK), T_s ≈ 5.8/ωn (ζ=1, 2%),  이산 감쇠 상한 D < 2Λ/T')
    print('   축       손  Λ[kg]     K[N/m]  ωn[rad/s]  D[N·s/m]  2Λ/T     T_s[s]  ωn≤%.1f  D<2Λ/T' % wn_max)
    for name, K, ax in (('리더 x', KL, 0), ('리더 y', KL, 1), ('팔로워 x', KF, 0), ('팔로워 y', KF, 1), ('z', Kz, 2)):
        for s in 'rl':
            L = md[s]['Lam'][ax]
            wn, D = math.sqrt(K / L), 2 * S['zeta'] * math.sqrt(L * K)
            print(f'   {name:8s} {s}  {L:6.3f}  {K:8.0f}  {wn:8.1f}  {D:8.1f}  {2 * L / T:7.0f}  {5.8 / wn:6.3f}   '
                  f'{"OK" if wn <= wn_max else "위반"}      {"OK" if D < 2 * L / T else "위반"}')
    print(f'   D/(2Λ/T) = ζ ωn T 이므로 ③ ωn ≤ {wn_max:.1f}이면 D는 상한의 {wn_max * T:.2f}배 이하로 자동 만족')

    Dp = [2 * math.sqrt(min(md['r']['Lam'][i], md['l']['Lam'][i]) * k) for i, k in enumerate([KL, KL, Kz])]
    Dr = [2 * math.sqrt(min(md['r']['Lam'][3 + i], md['l']['Lam'][3 + i]) * k) for i, k in enumerate(Krot)]
    r_real = rho(md, [KL, KL, Kz], Krot, wn_null, T, arm_real, wn_cap=wn_max)
    T_sim = 1.0 / S['f_sim']
    r_sim = rho(md, [KL, KL, Kz], Krot, wn_null, T_sim, [0.0] * 7, wn_cap=wn_max)
    r_sim100 = rho(md, [KL, KL, Kz], Krot, wn_null, T, [0.0] * 7, wn_cap=wn_max)
    print(f'   이산 안정 판정 ρ(Φ): 실물(100 Hz, armature) {r_real:.3f} < 1 OK,  시뮬 플랜트(armature 0) {S["f_sim"]:.0f} Hz {r_sim:.3f}'
          f' {"OK" if r_sim < 1 else "위반"} / 100 Hz {r_sim100:.2f} (불안정)')
    print('   → 시뮬 플랜트는 모터 회전자 관성(armature)이 없어 손목 관성이 실물보다 훨씬 작음 → 같은 게인을 100 Hz로 돌리면 발산.')
    print(f'     그래서 시뮬 검증은 {S["f_sim"]:.0f} Hz(robots.yaml state_rate), 게인은 실물 100 Hz 기준 그대로')
    print(f'\n⑧ 회전 K_rot  하한: 무게 피드포워드는 손끝(EE 원점) 힘이라, 손잡이 중심까지 r = {S["r_handle"]*1000:.1f} mm 떨어진 하중이 남기는')
    print(f'   모멘트 M = F r = {M_res:.3f} N·m 를 θ_max 안에 → K_rot ≥ M/θ_max = {M_res / th:.2f} N·m/rad (마찰 하한 시뮬 0)')
    print(f'   설정 K_rot = {Krot} (하한의 1.5배). 상한은 ⑥과 같은 관절 공간 모드 조건에 함께 포함')
    print(f'   영공간 τ0 [문서 2쪽 식을 관절 공간에]: K_i = M_ii ω², D_i = 2ζ M_ii ω, ω = 작업공간 가장 느린 축(z) ωn = {wn_null:.1f} rad/s'
          ' (자세 유지가 작업보다 빠르지 않게)')

    N_req = S['grip_SF'] * math.hypot(F_hand, S['F_int_max']) / (2 * S['mu'])
    tau_grip = N_req * abs(md['grip']['dgap_dq']) / 2
    print('\n⑨ 그리퍼 쥐는 힘 (⑥의 F_int,max와 같은 숫자)  손잡이 양면 마찰 2μN ≥ SF·|F_hand|, |F_hand| = √((m g/4)² + F_int,max²)')
    print(f'   N ≥ {S["grip_SF"]:.0f}·√({F_hand:.2f}² + {S["F_int_max"]:.0f}²)/(2·{S["mu"]}) = {N_req:.1f} N')
    print(f'   가상일: 손끝 간격 기울기 dgap/dq = {md["grip"]["dgap_dq"]*1000:.1f} mm/rad (손잡이 30 mm 접촉 q = {md["grip"]["q_contact"]:.3f}),'
          f' 구동 관절은 한쪽 손가락(간격의 절반)을 움직임 → τ_grip = N·|dgap/dq|/2 = {tau_grip:.2f} N·m')
    print('   시뮬: 그리퍼 관절 effort 한계 = τ_grip (위치 서보가 손잡이에 막히면 이 한계로 조임). 실물: RH-P12-RN 전류 한계로 같은 τ')

    ramp = round(2 * 5.8 / wn_null, 2)
    gains = dict(K_pos=[KL, KL, Kz], leader_K_pos=[KL, KL, Kz], follower_K_pos=[KF, KF, Kz], K_rot=Krot,
                 omega_max=round(wn_max, 2), D_safety=1.0, null_omega=round(wn_null, 2), zeta=S['zeta'],
                 weight_ramp=ramp, gripper_effort=round(tau_grip, 2))
    print(f'\n   피드포워드 램프 = 2 T_s,z = {ramp} s (무게를 z 정착시간보다 천천히 얹어 과도 처짐이 Δz_max를 넘지 않게)')
    with open(a.write, 'w') as f:
        f.write('# gain_design.py가 생성 — 값마다 근거는 gain_design.py 출력(GAIN_RATIONALE.md) 참고\n')
        yaml.safe_dump(gains, f, sort_keys=False)
    print(f'저장: {a.write}')


if __name__ == '__main__':
    main()
