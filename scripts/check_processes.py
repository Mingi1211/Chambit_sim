#!/usr/bin/env python3
"""Read actual container processes/maps to verify all three actuation owners."""
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
compose=['docker','compose','-f',str(root/'docker/compose.yaml')]
probe=r'''
import json,os
from pathlib import Path
wanted={'ros2_control_node','ai_worker_bimanual_movel_controller_node','coop_gazebo_server'}
found=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit(): continue
 try:
  executable=Path(os.readlink(p/'exe')).name
  if executable not in wanted: continue
  maps=(p/'maps').read_text()
  found.append({'pid':int(p.name),'executable':executable,
   'remote_hardware':'libcoop_carry_remote_hardware.so' in maps,
   'gazebo_actuator':'libcoop_carry_gazebo_actuator.so' in maps,
   'swerve_hardware':'libcoop_carry_remote_swerve.so' in maps,
   'gazebo_swerve':'libcoop_carry_gazebo_swerve.so' in maps,
   'swerve_controller':'libffw_swerve_drive_controller.so' in maps,
   'gz_ros2_control':'libgz_ros2_control' in maps})
 except (FileNotFoundError,PermissionError,ProcessLookupError): pass
print(json.dumps({'pid_namespace':os.readlink('/proc/self/ns/pid'),'processes':found}))
'''
results={}
for role in ('gazebo','robot1','robot2'):
 cid=subprocess.check_output(compose+['ps','-q',role],text=True).strip()
 if not cid: raise SystemExit(f'{role}: container not running')
 result=json.loads(subprocess.check_output(['docker','exec',cid,'python3','-c',probe],text=True))
 info=json.loads(subprocess.check_output(['docker','inspect',cid],text=True))[0]
 result['container_init_host_pid']=info['State']['Pid']
 result['image_id']=info['Image']
 result['host_processes']=subprocess.check_output(['docker','top',cid,'-eo','pid,args'],text=True).splitlines()[1:]
 results[role]=result
 ps=result['processes']
 if role=='gazebo':
  servers=[p for p in ps if p['executable']=='coop_gazebo_server']
  assert len(servers)==1 and servers[0]['gazebo_actuator']
  assert not any(p['executable']=='ros2_control_node' or p['remote_hardware'] or p['swerve_hardware'] or p['swerve_controller'] or p['gz_ros2_control'] for p in ps)
 else:
  managers=[p for p in ps if p['executable']=='ros2_control_node']
  qps=[p for p in ps if p['executable']=='ai_worker_bimanual_movel_controller_node']
  assert len(managers)==len(qps)==1,(role,ps)
  assert managers[0]['remote_hardware'] and not managers[0]['gz_ros2_control'] and not managers[0]['gazebo_actuator']
  assert not managers[0]['gazebo_swerve']
  if any(p['gazebo_swerve'] for p in results['gazebo']['processes']):
   assert managers[0]['swerve_hardware'] and managers[0]['swerve_controller']
assert len({r['pid_namespace'] for r in results.values()})==3
assert len({r['image_id'] for r in results.values()})==1
print(json.dumps({'result':'PASS','containers':results},indent=2))
