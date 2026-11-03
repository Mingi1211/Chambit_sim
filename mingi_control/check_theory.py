#!/usr/bin/env python3
"""시뮬 없이 식 검증 — T3(영공간), T6(문서 예제). 김민기.

  T3 [문서 #3] J̄ = M⁻¹JᵀΛ,  N_T = I − Jᵀ J̄ᵀ  →  Λ J M⁻¹ N_T τ0 = 0
     뜻: 영공간 토크 τ0가 손(작업공간) 가속도를 만들지 않는다 (Khatib의 동역학적으로 일관된 영공간, MR 8.6)
  T6 [문서 #6] Λ=4, F=20 N, e_ss=0.02 m, ζ=1 → K=1000, ωn≈15.81, D≈126.5, T_s≈0.37 s

사용: python3 check_theory.py <control.urdf>
"""
import sys

import numpy as np
import pinocchio as pin

from gain_design import t6


def t3(urdf, trials=20, seed=0):
    m = pin.buildModelFromXML(open(urdf).read())
    d = m.createData()
    rng = np.random.default_rng(seed)
    names = [f'arm_r_joint{i}' for i in range(1, 8)]
    vi = [m.joints[m.getJointId(n)].idx_v for n in names]
    fid = m.getFrameId('end_effector_r_link')
    worst = 0.0
    for _ in range(trials):
        q = pin.randomConfiguration(m)
        pin.computeJointJacobians(m, d, q)
        pin.updateFramePlacements(m, d)
        J = pin.getFrameJacobian(m, d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, vi]
        M = pin.crba(m, d, q)
        M = (np.triu(M) + np.triu(M, 1).T)[np.ix_(vi, vi)] + np.diag([0, 0, 0, 0, 0.05, 0.05, 0.05])
        M_inv = np.linalg.inv(M)
        if np.linalg.cond(J @ M_inv @ J.T) > 1e8:       # 특이 자세 제외 (Λ가 무한대)
            continue
        Lambda = np.linalg.inv(J @ M_inv @ J.T)
        J_bar_T = Lambda @ J @ M_inv                     # 노드 코드와 같은 식
        N_T = np.eye(7) - J.T @ J_bar_T
        tau0 = rng.normal(size=7) * 10
        worst = max(worst, np.linalg.norm(Lambda @ J @ M_inv @ N_T @ tau0))
    print(f'== T3 영공간 간섭 ‖Λ J M⁻¹ N_T τ0‖ 최댓값 ({trials}개 임의 자세, |τ0|~10 N·m) = {worst:.2e}')
    print('T3:', 'PASS' if worst < 1e-9 else 'FAIL', '(기준 < 1e-9)')
    return worst < 1e-9


if __name__ == '__main__':
    t6()
    if len(sys.argv) > 1:
        t3(sys.argv[1])
