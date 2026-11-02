"""Live host-side verification of four individual and simultaneous EE commands."""
import argparse
from copy import deepcopy
import json
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from .command_client import RobotClient, require_simulation


ARMS = [('robot1', 'r'), ('robot1', 'l'), ('robot2', 'r'), ('robot2', 'l')]


def joint_delta(before, after, side):
    return max(abs(after[j] - before[j]) for j in before if j.startswith(f'arm_{side}_joint'))


def position_error(a, b):
    return math.sqrt(sum((getattr(a, k)-getattr(b, k))**2 for k in ('x', 'y', 'z')))


def angle_error(a, b):
    dot = sum(getattr(a,k)*getattr(b,k) for k in ('x','y','z','w'))
    return 2*math.acos(min(1.0, abs(dot)))


def stationary(node):
    quiet = 0
    deadline = time.monotonic()+60
    while time.monotonic()<deadline:
        for r in node.robots: node.histories[r].clear()
        node.spin_for(2)
        drift = []
        for r, s in ARMS:
            samples = node.histories[r]
            if len(samples)<2: raise RuntimeError(f'{r}: measured joint stream stopped')
            drift.extend(max(x[j] for x in samples)-min(x[j] for x in samples)
                         for j in samples[0] if j.startswith(f'arm_{s}_joint'))
        quiet = quiet+1 if max(drift)<0.0001 else 0
        if quiet>=2: return max(drift)
    raise RuntimeError('Joints did not settle before next test')


def check_graph(node):
    for robot in node.robots:
        for topic, expected in [(f'/{robot}/joint_states', 'joint_state_broadcaster'),
                                 (f'/sim/{robot}/joint_commands', f'{robot}_remote_sim')]:
            pubs=node.get_publishers_info_by_topic(topic)
            if len(pubs)!=1 or pubs[0].node_name!=expected:
                raise RuntimeError(f'{topic}: wrong publisher ownership: {pubs}')
        for srv, suffix, field in [(ListControllers,'list_controllers','controller'),
                                   (ListHardwareComponents,'list_hardware_components','component')]:
            client=node.create_client(srv,f'/{robot}/controller_manager/{suffix}')
            future=client.call_async(srv.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            if not future.done() or future.result() is None: raise RuntimeError('Manager query failed')
            entries=getattr(future.result(),field)
            if field=='controller':
                controllers={x.name:x for x in entries}
                for name in ('arm_r_controller','arm_l_controller','joint_state_broadcaster'):
                    c=controllers[name]
                    if c.state!='active' or c.is_async: raise RuntimeError(f'{robot}/{name}: active synchronous controller required')
                if any('Serialized' in x.type for x in entries): raise RuntimeError('Unexpected legacy mutex controller')
            else:
                hardware={x.name:x for x in entries}
                h=hardware[f'{robot}_remote_sim']
                if h.state.label!='active' or h.plugin_name!='coop_carry_hardware/RemoteGazeboSystem':
                    raise RuntimeError('Remote hardware ownership mismatch')
            node.destroy_client(client)
    names={ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces()}
    if any('gz_ros_control' in x for x in names): raise RuntimeError('Unexpected Gazebo internal Controller Manager')
    print('PASS: independent ROS endpoints, external hardware, active synchronous stock JTCs',flush=True)


def trial(node, label, chosen, observe):
    baseline=stationary(node)
    before=deepcopy(node.states)
    before_commands=deepcopy(node.commands)
    stamps=dict(node.raw_stamp)
    goals={}
    offsets={('robot1','r'):(0.015,0,0),('robot1','l'):(0.012,0,0.003),
             ('robot2','r'):(0.014,0,-0.002),('robot2','l'):(0.011,0,0.004)}
    for key in chosen: goals[key]=node.make_goal(*key,offset=offsets[key])
    idle={key:0.0 for key in ARMS if key not in chosen}
    idle_commands={key:0.0 for key in idle}
    start=node.get_clock().now().nanoseconds
    send_times=[]
    for key,goal in goals.items():
        node.pubs[key].publish(goal)
        send_times.append(time.monotonic())
    deadline=time.monotonic()+60
    elapsed=0.0
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.01)
        for (r,s) in idle:
            idle[r,s]=max(idle[r,s],joint_delta(before[r],node.states[r],s))
            if (r,s) in before_commands and (r,s) in node.commands:
                idle_commands[r,s]=max(idle_commands[r,s],joint_delta(before_commands[r,s],node.commands[r,s],s))
        elapsed=(node.get_clock().now().nanoseconds-start)/1e9
        if elapsed>=observe: break
    results=[]
    for (r,s),goal in goals.items():
        measured=node.pose(r,s)
        error=position_error(measured.translation,goal.pose.pose.position)
        angular=angle_error(measured.rotation,goal.pose.pose.orientation)
        movement=joint_delta(before[r],node.states[r],s)
        if not (error<0.003 and angular<0.03 and movement>0.001):
            raise RuntimeError(f'{label} {r}/{s}: position={error}, orientation={angular}, movement={movement}')
        results.append({'robot':r,'arm':s,'position_error_mm':round(error*1000,4),'orientation_error_rad':angular,
                        'joint_motion_rad':movement,'goal':[goal.pose.pose.position.x,goal.pose.pose.position.y,goal.pose.pose.position.z]})
    if max(idle.values(),default=0)>0.001 or max(idle_commands.values(),default=0)>0.001:
        raise RuntimeError(f'{label}: interference: measured={idle}, commands={idle_commands}')
    if elapsed<observe or not all(node.raw_stamp.get(r,0)>stamps.get(r,0) for r in node.robots):
        raise RuntimeError('Simulation time or raw measured feedback stopped')
    # Compare raw measurement against the external Manager's broadcast after settling.
    node.spin_for(0.2)
    mismatch={r:max(abs(node.raw[r][j]-node.states[r][j]) for j in node.states[r]) for r in node.robots}
    if max(mismatch.values())>0.002: raise RuntimeError(f'Manager/raw feedback mismatch: {mismatch}')
    print(json.dumps({'test':label,'result':'PASS','active':results,'baseline_drift_rad':baseline,
        'inactive_arm_max_drift_rad':max(idle.values(),default=0),
        'inactive_arm_max_command_change_rad':max(idle_commands.values(),default=0),
        'raw_vs_manager_error_rad':mismatch,'send_window_ms':1000*(max(send_times)-min(send_times)),
        'elapsed_sim_sec':elapsed}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',choices=['individual','simultaneous','all'],default='all')
    parser.add_argument('--observation-time',type=float,default=6.0)
    args=parser.parse_args()
    if not math.isfinite(args.observation_time) or args.observation_time<=0: parser.error('Positive observation time required')
    require_simulation()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=RobotClient()
    try:
        node.wait_ready()
        check_graph(node)
        if args.suite in ('individual','all'):
            for key in ARMS: trial(node,'single_'+key[0]+'_'+key[1],[key],args.observation_time)
        if args.suite in ('simultaneous','all'):
            for label,chosen in [('robot1_both',ARMS[:2]),('robot2_both',ARMS[2:]),
                                  ('cross_robot',[ARMS[0],ARMS[3]]),('four_arms',ARMS)]:
                trial(node,label,chosen,args.observation_time)
        print('PASS: requested motion suite completed',flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
