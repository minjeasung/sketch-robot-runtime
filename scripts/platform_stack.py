#!/usr/bin/env python3
"""Own simulator/ROS process groups and publish fresh, measured readiness."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

CAMERA_TOPICS={
    'zed':('/zed/zed_node/rgb/color/rect/image','/zed/zed_node/depth/depth_registered','/zed/zed_node/rgb/color/rect/camera_info','/zed/zed_node/depth/camera_info'),
    'd405':('/d405/d405/color/image_raw','/d405/d405/depth/image_rect_raw','/d405/d405/color/camera_info','/d405/d405/depth/camera_info'),
}


def environment_ready(assets,bundle):
    try:
        marker=json.loads((Path(assets)/'environment-ready.json').read_text())
        os.kill(int(marker['pid']),0)
        return (marker.get('asset_revision')==bundle['revision']
                and marker.get('runtime_revision')==bundle.get('runtime_revision',0)
                and marker.get('applied') is True and marker.get('ground_applied') is True
                and marker.get('collision_object_count',0)>0)
    except (OSError,ValueError,KeyError,TypeError): return False


class StackReadiness:
    """Receipt leases renew only on complete sensor tuples and service replies."""
    def __init__(self,cameras,controllers):
        self.camera_names=tuple(cameras);self.required_controllers=set(controllers)
        self.parts={name:{} for name in cameras};self.complete={}
        self.controller_valid=False;self.controller_received=-float('inf')
        self.scene_status={};self.scene_received=-float('inf')

    def camera(self,name,kind,stamp,frame,width,height,now):
        if (kind not in ('rgb','depth','info_rgb','info_depth')
                or not frame or width<=0 or height<=0 or stamp<=0): return
        previous=self.complete.get(name)
        if previous is not None and stamp<=previous[0][0]: return
        # Independent topic delivery may interleave adjacent exposures. Retain
        # exact identities until all four parts arrive, rather than replacing
        # an older RGB with the next RGB before its depth/info callbacks run.
        parts=self.parts[name]
        for identity,(_,received) in list(parts.items()):
            if not 0<=now-received<=1.: del parts[identity]
        identity=(stamp,frame,width,height)
        kinds,_=parts.setdefault(identity,(set(),now));kinds.add(kind)
        if len(kinds)==4:
            self.complete[name]=(identity,now)
            for key in list(parts):
                if key[0]<=stamp: del parts[key]
        # At most sixteen pending exposures per camera, with no image payloads.
        # Prefer newer stamps; old/replayed frames cannot evict recent tuples.
        while len(parts)>16: del parts[min(parts)]

    def cameras_ready(self,now):
        return all(name in self.complete and 0<=now-self.complete[name][1]<=1. for name in self.camera_names)

    def controllers(self,controllers,now):
        active={c.name for c in controllers if c.state=='active'}
        self.controller_valid=self.required_controllers<=active;self.controller_received=now

    def controllers_ready(self,now):
        return self.controller_valid and 0<=now-self.controller_received<=1.

    def scene(self,payload,now):
        self.scene_status=payload;self.scene_received=now

    def scene_monitor_ready(self,now):
        return 0<=now-self.scene_received<=1.

    def scene_ready(self,now):
        p=self.scene_status
        return self.scene_monitor_ready(now) and p.get('ready') is True and bool(p.get('surface_id') and p.get('revision')) and all(p.get('acks',{}).get(engine) is True for engine in ('isaac','moveit'))


class PostInhibitObservation:
    """Keep an already-running simulator observable briefly after terminal stop.

    This never restores readiness or permits reset; the hardware inhibit is
    irreversible. Startup failures and non-simulation stacks still exit at once.
    """
    def __init__(self,*,simulation):
        self.simulation=simulation;self.was_ready=False;self.reason=None
        self.deadline=None;self.observation_only=False

    def update(self,marker,failure,now):
        if self.reason is None and failure:
            self.reason=failure
            self.observation_only=self.simulation and self.was_ready
            self.deadline=now+(5. if self.observation_only else 0.)
        if self.reason is None:
            self.was_ready=self.was_ready or marker.get('ready') is True
            return False
        marker.update(ready=False,work_ready=False,terminal_error=self.reason,
                      stop_observation_only=self.observation_only,
                      stop_observation_deadline_monotonic=self.deadline)
        return now>=self.deadline


class PaintStartupReadiness:
    """Hardware/safety prerequisites, independent of target, tare and plan readiness."""
    def __init__(self,session,scene):
        self.session=session;self.scene=scene;self.samples={};self.inhibited=False;self.guard_source_time=0.

    @staticmethod
    def guard_provenance_valid(payload,now):
        if not isinstance(payload,dict): return False
        source=payload.get('published_monotonic_s');sequence=payload.get('status_sequence')
        return (type(source) in (int,float) and math.isfinite(source) and 0<source<=now+.05
                and type(sequence) is int and sequence>0)

    def observe(self,kind,payload,now):
        if kind=='guard':
            if self.guard_provenance_valid(payload,now):
                source=payload['published_monotonic_s']
                if source<=self.guard_source_time: return
                self.guard_source_time=source
            else: payload=None  # A malformed sample cannot refresh controller health.
        self.samples[kind]=(payload,now)
        if (kind=='hardware' and isinstance(payload,dict)
                and payload.get('session_id')==self.session and payload.get('scene_revision')==self.scene
                and payload.get('inhibited') is True): self.inhibited=True

    def fatal_error(self):
        if self.inhibited:
            return 'Paint hardware motion inhibited; full stack restart required. Inspect the first Isaac hardware error in ros.log.'
        return None

    def gate_marker(self,marker,now):
        blockers=self.blockers(now);work_blockers=self.work_blockers(now)
        work_ready=marker.get('work_ready',marker.get('ready')) is True
        marker.update(ready=marker.get('ready') is True and not blockers,
                      work_ready=work_ready and not work_blockers,
                      paint_startup_ready=not blockers,paint_startup_blockers=blockers,
                      paint_work_ready=not work_blockers,paint_work_blockers=work_blockers)
        return blockers

    def work_blockers(self,now):
        problems=self.blockers(now)
        safety,_=self.samples.get('safety',(None,0.))
        if isinstance(safety,dict) and safety.get('abort_latched') is True:
            problems.append('Paint safety latch requires existing stationary/free-space reset: '+str(safety.get('reason','unknown')))
        guard,_=self.samples.get('guard',(None,0.))
        if isinstance(guard,dict) and guard.get('controller_fault') is True: problems.append('Paint controller fault requires existing reset interlocks')
        return problems

    def blockers(self,now):
        problems=[]
        hardware,received=self.samples.get('hardware',(None,-float('inf')))
        if not isinstance(hardware,dict) or not 0<=now-received<=.20:
            problems.append('Paint hardware status missing or stale (0.20 s lease)')
        else:
            if (hardware.get('simulation') is not True or hardware.get('session_id')!=self.session
                    or hardware.get('scene_revision')!=self.scene):
                problems.append('Paint hardware session/scene identity mismatch')
            if hardware.get('inhibited') is not False or self.inhibited:
                problems.append('Paint hardware motion inhibited; restart required')
            if hardware.get('state_fresh') is not True:
                problems.append('Paint hardware measured state invalid or stale')
        safety,received=self.samples.get('safety',(None,-float('inf')))
        if not isinstance(safety,dict) or not 0<=now-received<=.5:
            problems.append('Paint safety status missing or stale (0.5 s lease)')
        else:
            for field in ('ft_valid','tf_valid'):
                if safety.get(field) is not True: problems.append('Paint safety '+field+' is not valid')
            if type(safety.get('abort_latched')) is not bool:
                problems.append('Paint safety latch status missing or invalid')
            for field in ('nonfinite','saturated'):
                if safety.get(field) is not False:
                    problems.append('Paint safety '+field+': '+str(safety.get('reason','unknown')))
            for field in ('filtered_wrench','raw_wrench'):
                values=safety.get(field)
                if not isinstance(values,list) or len(values)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                    problems.append('Paint safety '+field+' must contain six finite components')
        guard,received=self.samples.get('guard',(None,-float('inf')))
        if (not 0<=now-received<=.5 or not self.guard_provenance_valid(guard,now)
                or not -.05<=now-guard['published_monotonic_s']<=.5):
            problems.append('Paint wrench guard controller status missing or stale (0.5 s lease)')
        elif (type(guard.get('controller_fault')) is not bool
                or not isinstance(guard.get('blockers'),list)
                or any(not isinstance(item,str) for item in guard['blockers'])):
            problems.append('Paint wrench guard controller status malformed')
        elif 'CONTROLLER_STATUS_STALE' in guard['blockers']:
            problems.append('Paint wrench guard reports controller status stale')
        return problems


def check_rosbridge_port(port):
    if not 1<=port<=65535: raise ValueError('rosbridge port must be in 1..65535')
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
            # Match rosbridge's reusable listener: TIME_WAIT is not a live owner.
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind(('0.0.0.0',port))
    except OSError as exc:
        raise RuntimeError(f'Rosbridge port {port} already in use; stop its owner or choose SKETCH_ROSBRIDGE_PORT') from exc


def bridge_listening(port):
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=.1): return True
    except OSError: return False


def shutdown_owned_groups(processes,interrupt_s=6.,terminate_s=2.):
    """Each child starts a new session; signal its whole owned descendant group."""
    def signal_all(sig):
        for process in reversed(processes):
            try: os.killpg(process.pid,sig)
            except ProcessLookupError: pass
    def wait_all(timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if all(p.poll() is not None for p in processes): return
            time.sleep(.05)
    signal_all(signal.SIGINT);wait_all(interrupt_s)
    # A ros2 launch parent can exit first. Kill its remaining owned group too.
    signal_all(signal.SIGTERM);wait_all(terminate_s)
    signal_all(signal.SIGKILL)
    for process in processes:
        try: process.wait(timeout=1.)
        except subprocess.TimeoutExpired: pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--rosbridge-port',type=int,default=int(os.environ.get('SKETCH_ROSBRIDGE_PORT','9090')))
    args=parser.parse_args();check_rosbridge_port(args.rosbridge_port)
    assets=args.assets.resolve();root=Path(__file__).resolve().parents[1]
    bundle=json.loads((assets/'bundle.json').read_text());selection=bundle['selection'];arm=bundle['arms'][selection['active_arm']]
    processes=[];logs=[];running=True;node=None
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def launch(name,command):
        log=(assets/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def alive():
        if any(p.poll() is not None for p in processes): raise RuntimeError('Runtime child exited; inspect bundle logs')
        return running
    def simulator_ready(sim_pid,expected_session=None):
        ready=json.loads((assets/'simulation-ready.json').read_text());now=time.monotonic()
        if (ready.get('pid')!=sim_pid or ready.get('asset_revision')!=bundle['revision'] or ready.get('runtime_revision')!=bundle.get('runtime_revision',0)
                or ready.get('scene_revision')!=bundle['revision'] or not ready.get('session_id') or not 0<=now-float(ready['updated_monotonic'])<2):
            raise ValueError('Simulator identity or freshness mismatch')
        if expected_session and ready['session_id']!=expected_session: raise RuntimeError('Simulator session changed; restart required')
        return ready
    try:
        for name in ('managed-ready.json','simulation-ready.json','environment-ready.json','startup-error.json'): (assets/name).unlink(missing_ok=True)
        sim=launch('simulation',['bash','scripts/run_platform_simulation.sh','--assets',str(assets),'--active-arm',selection['active_arm'],'--mode',selection['execution_mode'],'--process',selection['process']])
        deadline=time.monotonic()+240
        while alive():
            try:
                ready=simulator_ready(sim.pid)
                if ready.get('ready') is True: break
            except (OSError,ValueError,KeyError,TypeError): pass
            if time.monotonic()>deadline: raise RuntimeError('Simulator failed readiness')
            time.sleep(.1)
        if not running: return
        session=ready['session_id']
        check_rosbridge_port(args.rosbridge_port)
        launch('ros',['ros2','launch',str(root/'src/sketch_control/launch/platform_simulation.launch.py'),f'assets:={assets}',f'active_arm:={selection["active_arm"]}',f'execution_mode:={selection["execution_mode"]}',f'process_mode:={selection["process"]}'])
        launch('rosbridge',['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml',f'port:={args.rosbridge_port}'])
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import Image,CameraInfo
        from std_msgs.msg import String
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('platform_stack_monitor')
        namespace=f'/platform/{selection["active_arm"]}' if selection['process']=='paint' else ''
        controllers=node.create_client(ListControllers,namespace+'/controller_manager/list_controllers')
        names=['zed','d405'] if selection['process']=='paint' else ['zed']
        required=[arm['controller']]+(['joint_state_broadcaster','force_torque_sensor_broadcaster','force_torque_sensor_broadcaster_raw','admittance_controller'] if selection['process']=='paint' else [])
        state=StackReadiness(names,required);identities={};physical=None
        paint=PaintStartupReadiness(session,bundle['revision']) if selection['process']=='paint' else None
        if paint:
            def paint_json(msg,kind):
                try: payload=json.loads(msg.data)
                except (ValueError,TypeError): payload=None
                paint.observe(kind,payload,time.monotonic())
            node.create_subscription(String,namespace+'/isaac_painting/hardware_status',
                lambda msg:paint_json(msg,'hardware'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_subscription(String,namespace+'/painting_admittance/safety_status',lambda msg:paint_json(msg,'safety'),10)
            node.create_subscription(String,namespace+'/painting_admittance/wrench_guard_status',
                lambda msg:paint_json(msg,'guard'),10)
        if selection['execution_mode']=='mixed':
            from sketch_control.camera_registration import load_registration
            from sketch_control.mixed_frame_sync import PhysicalCameraReadiness
            physical=PhysicalCameraReadiness({name:load_registration(bundle['connections'][name+'_calibration_file'],name) for name in names})
        for name in names:
            for kind,topic,message_type in zip(('rgb','depth','info_rgb','info_depth'),CAMERA_TOPICS[name],(Image,Image,CameraInfo,CameraInfo)):
                def received(msg,n=name,k=kind):
                    state.camera(n,k,msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,msg.header.frame_id,msg.width,msg.height,time.monotonic())
                node.create_subscription(message_type,topic,received,qos_profile_sensor_data)
            def identity(msg,n=name):
                try:
                    payload=json.loads(msg.data)
                    if physical: physical.observe(n,payload,time.monotonic())
                    identities[n]=payload
                except (ValueError,KeyError,TypeError): identities.pop(n,None)
            node.create_subscription(String,f'/perception/{name}_frame_identity',identity,10)
        def scene(msg):
            try: state.scene(json.loads(msg.data),time.monotonic())
            except (ValueError,TypeError): state.scene({},time.monotonic())
        node.create_subscription(String,'/task_scene/status',scene,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        request=None;last_query=0.;deadline=time.monotonic()+180
        post_inhibit=PostInhibitObservation(simulation=bundle['selection']['execution_mode']=='simulation')
        bridge_ok=False;bridge_checked=-float('inf')
        while alive() and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
            ready=simulator_ready(sim.pid,session)
            if request is not None and request.done():
                try:
                    response=request.result();state.controllers(response.controller if response else [],now)
                except Exception: state.controllers([],now)
                request=None
            if request is not None and now-last_query>1.:
                request.cancel();request=None;state.controllers([],now)
            if controllers.service_is_ready() and request is None and now-last_query>.25:
                request=controllers.call_async(ListControllers.Request());last_query=now
            camera_ok=state.cameras_ready(now) and (physical is None or physical.ready(now))
            environment_ok=environment_ready(assets,bundle)
            if physical: environment_ok=environment_ok and state.scene_monitor_ready(now)
            services={name for name,_ in node.get_service_names_and_types()}
            nodes={name for name,_ in node.get_node_names_and_namespaces()}
            required_nodes={'move_group','moveit_executor','robot_state_publisher'}
            if physical: required_nodes.update(('task_scene','rgbd_weld_provider'))
            controller_ok=state.controllers_ready(now)
            if now-bridge_checked>=1.:
                bridge_ok=bridge_listening(args.rosbridge_port);bridge_checked=now
            ros_ready=controller_ok and camera_ok and environment_ok and '/get_planning_scene' in services and required_nodes<=nodes and bridge_ok
            marker=dict(ready,pid=os.getpid(),ready=ready.get('ready') is True and ros_ready,controllers_ready=controller_ok,cameras_ready=camera_ok,scene_monitor_ready=environment_ok,task_scene_ready=state.scene_ready(now) if physical else environment_ok,updated_monotonic=now,rosbridge_port=args.rosbridge_port)
            marker['work_ready']=marker['ready'] and marker['task_scene_ready']
            paint_blockers=paint.gate_marker(marker,now) if paint else []
            if physical:
                revisions={n:(identities.get(n,{}).get('frame_binding') or {}).get('registration_revision') for n in names}
                marker['calibration_revision']=json.dumps(revisions,sort_keys=True)
                marker['task_surface_revision']=state.scene_status.get('revision','')
            failure=paint.fatal_error() if paint else None
            teardown=post_inhibit.update(marker,failure,now)
            failure=post_inhibit.reason
            temp=assets/'managed-ready.tmp';temp.write_text(json.dumps(marker));temp.replace(assets/'managed-ready.json')
            if failure or now>deadline:
                reason=failure or '; '.join(paint_blockers) or 'Controllers, scene or cameras not ready; inspect ROS log'
                (assets/'startup-error.json').write_text(json.dumps(dict(error=reason,pid=os.getpid(),session_id=session,
                    asset_revision=bundle['revision'],runtime_revision=bundle.get('runtime_revision',0),updated_monotonic=now)))
                if teardown or not failure: raise RuntimeError(reason)
            if marker['ready']: deadline=float('inf')
    finally:
        (assets/'managed-ready.json').unlink(missing_ok=True);(assets/'managed-ready.tmp').unlink(missing_ok=True)
        shutdown_owned_groups(processes)
        for log in logs: log.close()
        if node:
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()
