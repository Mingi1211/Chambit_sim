#!/usr/bin/env python3
"""곡선 주행 운반 분석 — 베이스 간 오차(Δ), 손 내력, 리더 오차, 박스. 김민기 (11/23).

입력 (run 하나 = <dir>/<tag>_*):
  _base.csv    두 베이스 월드 자세 (/sim/robotN/base_odometry = Gazebo 실제 자세)
  _status.jsonl 현대 베이스 노드 상태 변화 (MOVING ~ DONE = 곡선 주행 구간)
  _hand.csv    /coop/hand_state (손목 F/T → 손 → 박스 렌치, 내력 (I − G†G)h)
  _task_robot1.csv  리더 임피던스 로그 (e = p_d − p, base_link)
  _box.csv     박스 자세 (Gazebo)
손 내력 대표값 = 중앙값: F/T 원신호에 접촉 충격 스파이크(수천 N)가 섞여 평균·최댓값이 스파이크로 정해짐 (11/23 b1: 평균 60 N, 중앙값 27 N)
베이스 간 오차: 두 베이스가 한 강체처럼 움직이면 상대 위치 r = p2 − p1 이 일정 → Δ(t) = r(t) − r(0) (base_error.py와 같은 정의)
  월드 x = 두 로봇을 잇는 방향 (두 base_link의 x축), 월드 y = 주행 방향 (base_link y축)
사용: python3 curve_analysis.py --dir ../../real_logs/1123_curve_trials --tags b1_K40_off b2_K40_off ...
"""
import argparse
import csv
import json
import math
import os

import numpy as np

HANDS = ['robot1_r', 'robot1_l', 'robot2_r', 'robot2_l']


def window(prefix):
    t0 = t1 = None
    for line in open(prefix + '_status.jsonl'):
        d = json.loads(line)
        if d['state'] == 'MOVING' and t0 is None:
            t0 = d['t_wall']
        if d['state'] in ('DONE', 'FAULT') and t0 is not None and t1 is None:
            t1 = d['t_wall']; final = d
    return t0, t1, final


def base(prefix, t0, t1):
    rows = list(csv.DictReader(open(prefix + '_base.csv')))
    P = {r: [] for r in ('robot1', 'robot2')}
    for r in rows:
        P[r['robot']].append([float(r['t_wall']), float(r['t_sim']), float(r['x']), float(r['y']), float(r['yaw'])])
    P = {k: np.array(v) for k, v in P.items()}
    t = P['robot1'][:, 0]
    p2 = np.column_stack([np.interp(t, P['robot2'][:, 0], P['robot2'][:, i]) for i in (2, 3, 4)])
    p1 = P['robot1'][:, 2:5]
    rel = p2 - p1
    i0 = np.searchsorted(t, t0)
    d = rel - rel[i0]
    d[:, 2] = np.arctan2(np.sin(d[:, 2]), np.cos(d[:, 2]))
    m = (t >= t0) & (t <= t1 + 5.0)
    return dict(t=t, ts=P['robot1'][:, 1], d=d, m=m, p1=p1, p2=p2, i0=i0, rel0=rel[i0])


def hand(prefix, t0, t1):
    rows = [r for r in csv.reader(open(prefix + '_hand.csv'))]
    A = np.array([[float(r[0])] + [float(v) for v in r[3:]] for r in rows if len(r) == 43])
    m = (A[:, 0] >= t0) & (A[:, 0] <= t1)
    W = A[:, 1:25].reshape(-1, 4, 6)
    Fi = A[:, 25:37].reshape(-1, 4, 3)
    return dict(t=A[:, 0], m=m, F=W[:, :, :3], Fint=Fi, nint=np.linalg.norm(Fi, axis=2),
                fxy=np.linalg.norm(W[:, :, :2], axis=2))


def leader(prefix, t0, t1):
    rows = list(csv.DictReader(open(prefix + '_task_robot1.csv')))
    out = {}
    for s in ('r', 'l'):
        R = [r for r in rows if r['side'] == s and t0 <= float(r['t']) <= t1]
        e = np.array([[float(r['ex']), float(r['ey'])] for r in R])
        out[s] = np.abs(e).max(0) if len(e) else np.array([np.nan, np.nan])
    return out


def box(prefix, t0, t1):
    rows = list(csv.DictReader(open(prefix + '_box.csv')))
    B = np.array([[float(r[k]) for k in ('t', 'x', 'y', 'z', 'roll', 'pitch')] for r in rows])
    m = (B[:, 0] >= t0) & (B[:, 0] <= t1)
    tilt = np.degrees(np.hypot(B[:, 4], B[:, 5]))
    return dict(tilt_move=tilt[m].max() if m.any() else np.nan, final=B[-1, 1:4], tilt_final=tilt[-1])


def analyze(d, tag):
    pre = os.path.join(d, tag)
    t0, t1, final = window(pre)
    out = dict(tag=tag, t0=t0, t1=t1, base=base(pre, t0, t1), final=final)
    out['T_wall'] = t1 - t0
    b = out['base']
    out['T_sim'] = b['ts'][b['m']].max() - b['ts'][b['i0']]
    for k, f in (('hand', hand), ('leader', leader), ('box', box)):
        try:
            out[k] = f(pre, t0, t1)
        except (FileNotFoundError, ValueError, IndexError):
            out[k] = None
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--tags', nargs='+', required=True)
    a = ap.parse_args()
    print('tag            | Δx max  Δy max  |Δ|max  Δyaw max | Δx end  Δy end | R1 전진  R2 전진 | 손 내력 중앙값 [N] 출발 전 20 s → 주행 중 (손별 r1r r1l r2r r2l) | 주행 중 월드 성분 중앙값 | 리더 |e| x/y max [mm] | 박스 기울기 이동중/끝, 최종 xyz')
    for tag in a.tags:
        r = analyze(a.dir, tag)
        b = r['base']; d = b['d'][b['m']]
        i1 = np.where(b['m'])[0][-1]
        fw = {k: v['forward_progress'] for k, v in r['final']['robots'].items()}
        s = (f'{tag:14s} | {np.abs(d[:, 0]).max()*1e3:5.0f}  {np.abs(d[:, 1]).max()*1e3:5.0f}  {np.linalg.norm(d[:, :2], axis=1).max()*1e3:5.0f}'
             f'  {math.degrees(np.abs(d[:, 2]).max()):5.1f}° | {b["d"][i1, 0]*1e3:5.0f}  {b["d"][i1, 1]*1e3:5.0f} | {fw["robot1"]:.3f}  {fw["robot2"]:.3f} |')
        h = r['hand']
        if h is not None and h['m'].any():
            pre_m = (h['t'] > r['t0'] - 20) & (h['t'] < r['t0'])
            s += f' {np.median(h["nint"][pre_m]):5.1f} → {np.median(h["nint"][h["m"]]):5.1f}'
            s += ' (' + ' '.join(f'{np.median(h["nint"][h["m"], i]):4.1f}' for i in range(4)) + ')'
            fz = np.abs(h['Fint'][h['m']])
            s += f' | x {np.median(fz[:, :, 0]):4.1f} y {np.median(fz[:, :, 1]):4.1f} z {np.median(fz[:, :, 2]):4.1f}'
        L = r['leader']
        if L is not None:
            s += ' | ' + ' '.join(f'{L[k][0]*1e3:.0f}/{L[k][1]*1e3:.0f}' for k in 'rl')
        B = r['box']
        if B is not None:
            s += f' | {B["tilt_move"]:.1f}°/{B["tilt_final"]:.1f}° ({B["final"][0]:.2f}, {B["final"][1]:.2f}, {B["final"][2]:.2f})'
        print(s + f'  [주행 {r["T_wall"]:.0f} s 벽시계, {r["T_sim"]:.1f} s 시뮬]')
