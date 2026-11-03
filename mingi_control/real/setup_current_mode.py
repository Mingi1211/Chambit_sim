#!/usr/bin/env python3
"""실물 SG2 팔을 DYNAMIXEL 전류 제어로 — ai_worker 설정 패치 (원본 .orig 백업, --restore 복구).

1) ros2_control xacro: 팔 관절 14개에 command_interface effort (ROBOTIS 하드웨어 인터페이스가 effort → Goal Current로 보냄)
   팔 모터(gpio dxl1~7, dxl31~37): Operating Mode 3(위치) → 0(전류), command_interface "Goal Current" 추가
2) controller yaml: arm_r/l_effort_controller (ForwardCommandController, interface effort) 추가 — 노드 output:=dxl이 여기로 발행
그리퍼(dxl8, dxl38)·리프트·머리·베이스는 그대로(위치).

[미검증 — 실물에서 확인] 전류 모드에서는 위치 명령(QP·JTC)이 팔에 안 먹는다. 실물 순서는
  전원 켤 때부터 전류 모드 + impedance_node mode:=gravity(팔 떠 있게) → task로 유지·이동.
  동작 중 Operating Mode를 바꾸려면 토크를 꺼야 해서 팔이 떨어진다 → 런타임 전환은 하지 않는다.

사용: python3 setup_current_mode.py --ai_worker <ai_worker 경로> [--restore]
"""
import argparse
import os
import re
import shutil

ap = argparse.ArgumentParser()
ap.add_argument('--ai_worker', required=True)
ap.add_argument('--restore', action='store_true')
a = ap.parse_args()
XACRO = os.path.join(a.ai_worker, 'ffw_description/ros2_control/ffw_sg2_rev1_follower/ffw_sg2_follower.ros2_control.xacro')
YAML = os.path.join(a.ai_worker, 'ffw_bringup/config/ffw_sg2_rev1_follower/ffw_sg2_follower_ai_hardware_controller.yaml')
ARM_IDS = list(range(1, 8)) + list(range(31, 38))

for f in (XACRO, YAML):
    if a.restore:
        if os.path.exists(f + '.orig'):
            shutil.copy(f + '.orig', f)
            print('복구:', f)
        continue
    if not os.path.exists(f + '.orig'):
        shutil.copy(f, f + '.orig')
if a.restore:
    raise SystemExit

x = open(XACRO).read()
if 'MINGI_CURRENT' not in x:
    for side in 'rl':
        for i in range(1, 8):
            x = re.sub(rf'(<joint name="arm_{side}_joint{i}">\s*<command_interface name="position"/>)',
                       r'\1\n        <command_interface name="effort"/><!-- MINGI_CURRENT -->', x, count=1)
    for i in ARM_IDS:
        block = re.search(rf'<gpio name="dxl{i}">.*?</gpio>', x, re.S).group(0)
        new = block.replace('<param name="Operating Mode">3</param>', '<param name="Operating Mode">0</param><!-- MINGI_CURRENT -->')
        new = new.replace('<command_interface name="Goal Position"/>',
                          '<command_interface name="Goal Position"/>\n        <command_interface name="Goal Current"/>')
        x = x.replace(block, new)
    open(XACRO, 'w').write(x)
    print('xacro: 팔 14관절 effort, 모터 14개 Operating Mode 0 + Goal Current —', x.count('MINGI_CURRENT'), '곳')
y = open(YAML).read()
if 'arm_r_effort_controller' not in y:
    y += '\n# MINGI_CURRENT\n/**:\n  controller_manager:\n    ros__parameters:\n'
    for s in 'rl':
        y += f'      arm_{s}_effort_controller:\n        type: forward_command_controller/ForwardCommandController\n'
    for s in 'rl':
        joints = ''.join(f'        - arm_{s}_joint{i}\n' for i in range(1, 8))
        y += f'/**:\n  arm_{s}_effort_controller:\n    ros__parameters:\n      joints:\n{joints}      interface_name: effort\n'
    open(YAML, 'w').write(y)
    print('yaml: arm_r/l_effort_controller 추가')
print('다음: colcon build --packages-select ffw_description ffw_bringup → motor_params.yaml 채우기 → impedance_node output:=dxl')
