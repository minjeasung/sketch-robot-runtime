#!/usr/bin/env python3
"""Selected robot/tool simulation; only active-arm commands reach the articulation."""
import argparse
import gc
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid
from collections import deque
from contextlib import ExitStack, contextmanager


@contextmanager
def simulation_gc_scope():
    """Exclude the initialized Kit heap from scans during measured feedback.

    A generation-2 scan of Kit's long-lived graph can exceed the 0.20 s
    state lease even without a slow physics/render step. Collect startup
    garbage before entering the loop, then freeze only the existing graph.
    Newly allocated cycles still receive normal automatic collection.
    This standalone process owns the GC scope; release it on every exit.
    """
    gc.collect()
    gc.freeze()
    try:
        yield
    finally:
        gc.unfreeze()


def spray_visual_active(payload,*,received,now,stamp,plan_hash):
    """A visual lease is independent of gun IO and expires on either clock."""
    if (not isinstance(payload,dict) or payload.get('enabled') is not True
            or payload.get('simulation_only') is not True or not plan_hash
            or payload.get('plan_hash')!=plan_hash or payload.get('lease_ms')!=250
            or type(payload.get('timestamp_ns')) is not int):
        return False
    return (0<=now-received<=.25 and 0<=stamp-payload['timestamp_ns']*1e-9<=.25)


def spray_nozzle_body(robot,link_indices,tcp_link):
    """Resolve empty fixed TCP links to an actual measured PhysX body."""
    import numpy as np
    from sketch_control.eoat_catalog import origin_tf
    parents={j.find('child').get('link'):j for j in robot.findall('joint')}
    transform=np.eye(4);link=tcp_link
    while link not in link_indices:
        joint=parents.get(link)
        if joint is None or joint.get('type')!='fixed':
            raise ValueError('No measured rigid ancestor for spray nozzle')
        transform=origin_tf(joint.find('origin'))@transform
        link=joint.find('parent').get('link')
    return link_indices[link],transform


def spray_plan_qos():
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    return QoSProfile(depth=10,durability=DurabilityPolicy.VOLATILE,
                      reliability=ReliabilityPolicy.RELIABLE)


class SprayVisualState:
    """Keep generated candidates distinct from the executor's accepted plan."""
    def __init__(self,profile_hash):
        self.profile_hash=profile_hash;self.plans={};self.active_plan=''
        self.payload={};self.received=-math.inf

    def plan(self,payload):
        if (isinstance(payload,dict) and payload.get('process_mode')=='spray'
                and payload.get('spray_envelope_profile_sha256')==self.profile_hash):
            key=payload.get('plan_hash')
            if isinstance(key,str) and len(key)==64:
                self.plans[key]=True
                if len(self.plans)>16:
                    old=next(k for k in self.plans if k!=self.active_plan)
                    del self.plans[old]

    def receive(self,payload,received):
        self.payload=payload;self.received=received

    def current(self,now,stamp):
        key=self.payload.get('plan_hash','') if isinstance(self.payload,dict) else ''
        active=key in self.plans and spray_visual_active(self.payload,received=self.received,
                       now=now,stamp=stamp,plan_hash=key)
        if active:self.active_plan=key
        return bool(active),self.active_plan


class SimulationPhaseTiming:
    """Bounded wall-clock diagnostics only; never a source of physics timestamps."""
    def __init__(self,*,session,revision,capacity=256):
        self.session=session;self.revision=revision
        self.recent=deque(maxlen=capacity);self.slow=deque(maxlen=min(capacity,16))
        self.previous_end=None;self.last_publication=None

    def begin(self,now,step):
        self.started=self.last_mark=now
        self.current=dict(step_index=step,started_monotonic=now,phases_s={},
            between_steps_wall_s=now-self.previous_end if self.previous_end is not None else 0.)

    def mark(self,phase,now):
        self.current['phases_s'][phase]=self.current['phases_s'].get(phase,0.)+now-self.last_mark
        self.last_mark=now

    def published(self,now):
        self.current['publication_gap_s']=now-self.last_publication if self.last_publication is not None else 0.
        self.last_publication=now

    def finish(self,now):
        self.mark('other',now);self.previous_end=now
        self.current.update(total_wall_s=now-self.started,session_id=self.session,scene_revision=self.revision)
        self.recent.append(self.current)
        if max(self.current['total_wall_s'],self.current['between_steps_wall_s'],
               self.current.get('publication_gap_s',0.))>.1:
            self.slow.append(self.current)
            return self.current
        return None

    def snapshot(self):
        return dict(session_id=self.session,scene_revision=self.revision,threshold_wall_s=.1,
                    recent_steps=list(self.recent),slow_steps=list(self.slow))


def command_positions(names,positions,active,limits):
    if len(names)!=len(positions) or len(set(names))!=len(names):raise ValueError('Invalid joint command dimensions')
    result={}
    for name,value in zip(names,positions):
        if name not in active:continue
        if not math.isfinite(value) or not limits[name][0]<=value<=limits[name][1]:raise ValueError('Joint command outside limits')
        result[name]=value
    return result


def simulator_timing_contract(process):
    """Resolve stiff drives at 500 Hz while retaining 100 Hz measured feedback."""
    if process not in ('paint','spray','weld'):raise ValueError('Unsupported simulation timing process')
    return dict(physics_hz=500,joint_state_hz=100,render_hz=15,state_stride=5)


def configure_simulator_solver(stage,scene_path):
    """Apply gravity per TGS substep before the physics scene is initialized."""
    from pxr import UsdPhysics,Sdf
    scene=UsdPhysics.Scene.Get(stage,scene_path)
    if not scene:raise ValueError('Expected an existing physics scene')
    # PhysX documents steady-position/nonzero-velocity articulation drives when
    # gravity is applied only once per frame. Fix the solver, not measured data:
    # https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/guides/limitations.html
    scene.GetPrim().CreateAttribute('physxScene:enableExternalForcesEveryIteration',
                                   Sdf.ValueTypeNames.Bool,custom=False).Set(True)
    return dict(scene_path=str(scene.GetPath()),
                solver_type=scene.GetPrim().GetAttribute('physxScene:solverType').Get(),
                enable_external_forces_every_iteration=scene.GetPrim().GetAttribute(
                    'physxScene:enableExternalForcesEveryIteration').Get())


def configure_simulator_articulation_solver(prim,selection):
    """Author the RB10 paint TGS experiment without changing other processes."""
    from pxr import UsdPhysics,Sdf
    if selection.get('execution_mode') not in ('simulation','mixed'):
        raise ValueError('Solver override is restricted to simulation robot modes')
    if not prim or not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise ValueError('Expected an articulation root')
    targeted=selection.get('platform')=='rb10' and selection.get('process')=='paint'
    # Installed PhysX 110.3.2 defaults to one velocity iteration. Current TGS
    # no longer converts iterations above four into position iterations:
    # https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/guides/limitations.html
    # Retain the position budget; compare actual measured velocities in live
    # loaded cancellation and normal painting before accepting this experiment.
    for name,value in (('solverPositionIterationCount',64),
                       ('solverVelocityIterationCount',1 if targeted else 8)):
        prim.CreateAttribute('physxArticulation:'+name,Sdf.ValueTypeNames.Int,custom=False).Set(value)
    return dict(articulation_path=str(prim.GetPath()),
        solver_position_iteration_count=prim.GetAttribute('physxArticulation:solverPositionIterationCount').Get(),
        solver_velocity_iteration_count=prim.GetAttribute('physxArticulation:solverVelocityIterationCount').Get(),
        selection={key:selection.get(key) for key in ('platform','process','execution_mode')},
        rb10_paint_tgs_iteration_override=targeted,
        provenance=('PhysX 110.3.2 schema default velocity count; documented TGS low-velocity-iteration workaround'
                    if targeted else 'Existing platform simulation 64/8 iteration configuration'))


def simulator_drive_contract(selection,limits):
    """Virtual actuator gains/limits; never physical actuator specifications."""
    if selection.get('execution_mode') not in ('simulation','mixed'):
        raise ValueError('Drive override is restricted to simulation robot modes')
    source={name:float(values['effort']) for name,values in limits.items()}
    if not source or any(not math.isfinite(value) or value<=0 for value in source.values()):
        raise ValueError('Invalid source joint effort limit')
    maximum=dict(source);provenance='Original URDF effort limits'
    if selection.get('platform')=='rb10':
        if set(source)!={'base','shoulder','elbow','wrist1','wrist2','wrist3'}:
            raise ValueError('RB10 simulation joint identity mismatch')
        # The source's 10 Nm is insufficient even for modeled gravity (elbow
        # needs ~57–59 Nm at ready). Reuse the established RB20 force-drive
        # simulator effort model. Contact gains remain the RB20 paint values.
        # Keep URDF limits and source ready pose unchanged; physical capability
        # is deliberately not inferred from this idealized simulation model.
        maximum={name:2000. for name in source}
        provenance='RB20 simulator force-drive contract: prepare_rb20_painting_assets.py scene.drive.max_torque_nm'
    elif selection.get('platform')!='commercial':
        raise ValueError('Unsupported simulation platform')
    process=selection.get('process')
    if process not in ('paint','spray','weld'):raise ValueError('Unsupported simulation drive process')
    # At 1 Hz, the previous D/K=.1 s attenuated a measured 4 mm weld weave
    # to 3.387 mm. Noncontact processes use D/K=.02 s; paint retains its
    # established force-contact dynamics. Effort and velocity limits stay fixed.
    damping=30000. if process=='paint' else 6000.
    gain_provenance=('RB20 contact simulation gains' if process=='paint' else
                     'Noncontact simulation tracking: 1 Hz weld trace D/K attenuation diagnosis')
    return dict(simulation_only=True,process=process,source_effort_limit_nm=source,max_torque_nm=maximum,
                provenance=provenance,stiffness_nm_per_rad=300000.,damping_nms_per_rad=damping,
                gain_provenance=gain_provenance+'; not physical actuator ratings',
                timing=simulator_timing_contract(process))


def apply_simulator_drive_efforts(stage,selection,limits):
    """Apply virtual effort and process gains to this robot's exact joint set."""
    from pxr import UsdPhysics
    from rb20_painting_scene import usd_angular_gain
    contract=simulator_drive_contract(selection,limits)
    joints=[p for p in stage.Traverse() if str(p.GetPath()).startswith('/World/Robot/')
            and p.IsA(UsdPhysics.RevoluteJoint)]
    if len(joints)!=len(limits) or {p.GetName() for p in joints}!=set(limits):
        raise ValueError('Simulation drive joint set differs from description')
    for prim in joints:
        drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
        drive.CreateTypeAttr('force')
        drive.CreateMaxForceAttr(contract['max_torque_nm'][prim.GetName()])
        drive.CreateStiffnessAttr(usd_angular_gain(contract['stiffness_nm_per_rad']))
        drive.CreateDampingAttr(usd_angular_gain(contract['damping_nms_per_rad']))
    return contract


def atomic_json(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data));tmp.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--active-arm',choices=['left','right','single'],required=True)
    parser.add_argument('--mode',choices=['simulation','mixed'],required=True)
    parser.add_argument('--process',choices=['spray','weld','paint'],required=True)
    parser.add_argument('--no-window',action='store_true',default=not bool(os.environ.get('DISPLAY')))
    parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--capture',type=Path)
    args=parser.parse_args();assets=args.assets.resolve();bundle=json.loads((assets/'bundle.json').read_text())
    selected=bundle['selection']
    if (args.active_arm,args.mode,args.process)!=(selected['active_arm'],selected['execution_mode'],selected['process']):
        raise ValueError('CLI differs from asset selection')
    import charset_normalizer
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.no_window,'renderer':'RaytracedLighting','width':1280,'height':900})
    node=None;running=True
    loop_resources=ExitStack()
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        import numpy as np
        import xml.etree.ElementTree as ET
        from scipy.spatial.transform import Rotation
        import omni.usd
        from pxr import Gf,UsdGeom,UsdLux,UsdPhysics,PhysxSchema
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge');enable_extension('isaacsim.sensors.camera')
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.core.api.objects import FixedCuboid
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from builtin_interfaces.msg import Time
        from rb20_painting_scene import ft_world_transform,configure_roller_contact,configure_paint_wall_material
        from rb20_painting_sensors import RenderedCameras,RollerSensorContract,WrenchWindowAverage,transport_qos_profiles
        scene=json.loads((assets/'scene.json').read_text());arm=bundle['arms'][args.active_arm]
        timing=simulator_timing_contract(args.process);physics_hz=timing['physics_hz']
        world=World(stage_units_in_meters=1.,physics_dt=1/physics_hz,rendering_dt=1/timing['render_hz'])
        stage=omni.usd.get_context().get_stage()
        solver_contract=configure_simulator_solver(stage,world.get_physics_context().prim_path)
        world.scene.add_default_ground_plane(z_position=-.025)
        add_reference_to_stage(bundle['usd'],'/World/Robot')
        light=UsdLux.DomeLight.Define(stage,'/World/Lighting');light.CreateIntensityAttr(1500.)
        if args.mode=='simulation':
            if args.process=='paint':
                wall=scene['wall'];world.scene.add(FixedCuboid('/World/TaskSurface',name='painting_wall',position=np.array(wall['center']),scale=np.array(wall['size']),size=1.,color=np.array([.7,.75,.8])))
            else:add_reference_to_stage(str(assets/'h_steel.usda'),'/World/TaskSurface')
        if args.process=='paint':
            friction=dict(static_friction=.4,dynamic_friction=.3,restitution=0.)
            contact=configure_roller_contact(stage,dict(acceleration_spring=False,stiffness_n_per_m=5000.,damping_ns_per_m=50.),
                friction,robot_root='/World/Robot',
                collision_name=arm['contact_link']+'_paint_eoat_no_camera_roller_contact_collision',
                body_name=arm['wrench_frame'],material_path='/World/RollerContactMaterial_'+args.active_arm)
            contact.update(friction=friction,contact_model='fixed_cylinder_sliding_approximation')
            if args.mode=='simulation':
                contact.update(configure_paint_wall_material(stage,'/World/TaskSurface',friction))
            atomic_json(assets/'simulation-contact-contract.json',contact)
            print('PLATFORM_ROLLER_CONTACT '+json.dumps(contact),flush=True)
        xml=ET.parse(bundle['physics_urdf']).getroot()
        limits={j.get('name'):{k:float(v) for k,v in j.find('limit').attrib.items()} for j in xml.findall("joint[@type='revolute']")}
        drive_contract=apply_simulator_drive_efforts(stage,selected,limits)
        atomic_json(assets/'simulation-drive-contract.json',drive_contract)
        print('PLATFORM_SIMULATED_DRIVES '+json.dumps(drive_contract),flush=True)
        roots=[];solver_contract['articulations']=[]
        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith('/World/Robot'):continue
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                roots.append(str(prim.GetPath()));PhysxSchema.PhysxArticulationAPI.Apply(prim)
                solver_contract['articulations'].append(configure_simulator_articulation_solver(prim,selected))
            if prim.IsA(UsdPhysics.RevoluteJoint):
                name=prim.GetName();drive=UsdPhysics.DriveAPI.Get(prim,'angular')
                drive.CreateTargetPositionAttr(float(np.degrees(bundle['initial_positions'][name])))
                PhysxSchema.JointStateAPI.Apply(prim,'angular').CreatePositionAttr(float(np.degrees(bundle['initial_positions'][name])))
                PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(float(np.degrees(limits[name]['velocity'])))
        if len(roots)!=1:raise RuntimeError('Expected single articulation')
        atomic_json(assets/'simulation-solver-contract.json',solver_contract)
        print('PLATFORM_SIMULATION_SOLVER '+json.dumps(solver_contract),flush=True)
        from platform_collision import apply_srdf_collision_filters
        filters=apply_srdf_collision_filters(stage,bundle)
        atomic_json(assets/'physics-collision-filters.json',filters)
        print('PLATFORM_COLLISION_FILTERS '+json.dumps({key:filters[key] for key in ('source_pair_count','applied_pair_count','srdf_sha256')}),flush=True)
        robot=world.scene.add(SingleArticulation(prim_path=roots[0],name='platform'));world.reset()
        if set(robot.dof_names)!=set(bundle['joint_names']):raise RuntimeError('Articulation joint contract mismatch')
        pose=np.array([bundle['initial_positions'][n] for n in robot.dof_names])
        robot.set_joints_default_state(positions=pose,velocities=np.zeros(len(pose)));robot.set_joint_positions(pose)
        robot.apply_action(ArticulationAction(joint_positions=pose))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('platform_simulation')
        state_pub=node.create_publisher(JointState,'/joint_states',10);clock_pub=node.create_publisher(Clock,'/clock',10)
        session=uuid.uuid4().hex;revision=bundle['revision'];hold=pose.copy();last_command=-math.inf
        index={name:i for i,name in enumerate(robot.dof_names)};active=arm['joints'];range_limits={n:(limits[n]['lower'],limits[n]['upper']) for n in active}
        def receive_joint(msg):
            nonlocal last_command
            if args.process=='paint':return
            try:values=command_positions(msg.name,msg.position,active,range_limits)
            except ValueError:return
            for name,value in values.items():hold[index[name]]=value
            if values:last_command=time.monotonic()
        node.create_subscription(JointState,'/isaac_joint_command',receive_joint,10)
        pending=[];ack_pub=node.create_publisher(String,'/task_scene/ack',10)
        surface_qos=rclpy.qos.QoSProfile(depth=1,reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,'/task_scene/apply',lambda msg:pending.append(msg.data),surface_qos)
        cameras=None;camera_manifest={'session_id':session};command_stats={'accepted':0,'rejected':0};ft_body=None;force_valid=args.process!='paint';lease=None
        if args.process=='paint':
            from isaac_painting_msgs.msg import State,Command
            sensor=RollerSensorContract.from_arm_contract(arm,session_id=session,scene_revision=revision,topic_namespace=f'/platform/{args.active_arm}/isaac_painting')
            ft_row,ft_body=sensor.physics_indices(robot._articulation_view._metadata)
            ft=next(UsdPhysics.FixedJoint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint) and p.GetName()==arm['sensor_joint'])
            joint_to_ft=np.eye(4);joint_to_ft[:3,3]=ft.GetLocalPos1Attr().Get();q=ft.GetLocalRot1Attr().Get()
            joint_to_ft[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
            stream_qos,command_qos=transport_qos_profiles(rclpy.qos)
            force_pub=node.create_publisher(State,sensor.topic('state'),stream_qos);lease=sensor.command_lease();average=WrenchWindowAverage(timing['state_stride'])
            def receive_force(msg):
                if not msg.enabled or (len(msg.positions)==6 and all(range_limits[n][0]<=v<=range_limits[n][1] for n,v in zip(active,msg.positions))):
                    accepted=lease.accept(msg,time.monotonic());command_stats['accepted' if accepted else 'rejected']+=1
                else:command_stats['rejected']+=1
            node.create_subscription(Command,sensor.topic('command'),receive_force,command_qos)
            camera_manifest['camera_ft_transforms']=bundle['camera_ft_transforms'][args.active_arm]
        zed_body=None;zed_fixed=None
        if args.mode=='simulation':
            cameras=RenderedCameras(node,camera_manifest,scene)
            if scene.get('zed_mount'):
                mount=scene['zed_mount']
                zed_body,zed_ancestor=spray_nozzle_body(xml,robot._articulation_view._metadata.link_indices,mount['parent_link'])
                zed_fixed=zed_ancestor@np.array(mount['parent_from_optical'])
        spray_visual=None;spray_report={}
        if args.mode=='simulation' and args.process=='spray' and bundle.get('spray_envelope_profile'):
            from platform_spray_visual import SprayVisual
            from rbpodo_painting_control.spray_envelope import load_spray_envelope
            import yaml
            envelope_path=Path(bundle['spray_envelope_profile']);envelope=load_spray_envelope(envelope_path)
            spray_state=SprayVisualState(envelope.sha256)
            spray_parameters=yaml.safe_load((assets/'painting.yaml').read_text())['sketch_to_waypoints']['ros__parameters']
            spray_visual=SprayVisual(stage,json.loads(envelope_path.read_text()),
                json.loads((assets/'h_steel_collision.json').read_text()),
                projection_end_m=spray_parameters['spray_standoff_m'])
            nozzle_body,nozzle_fixed=spray_nozzle_body(xml,robot._articulation_view._metadata.link_indices,arm['tcp_link'])
            def receive_spray_visual(msg):
                try:payload=json.loads(msg.data)
                except (ValueError,TypeError):payload={}
                spray_state.receive(payload,time.monotonic())
            def receive_spray_plan(msg):
                try:spray_state.plan(json.loads(msg.data))
                except (ValueError,TypeError):pass
            node.create_subscription(String,'/painting_system/spray_visualization',receive_spray_visual,1)
            node.create_subscription(String,'/sketch_eoat_segments',receive_spray_plan,spray_plan_qos())
        set_camera_view(eye=np.array([3.3,-4.5,3.]),target=np.array([0.,-.5,1.]))
        observation=None
        if os.environ.get('PAINTING_OBSERVATION_DIR'):
            if args.mode!='simulation' or args.process!='paint':
                raise ValueError('Painting observation requires simulation paint mode')
            from painting_observation import PaintingObservation
            observation=PaintingObservation(os.environ['PAINTING_OBSERVATION_DIR'],node,session,robot.dof_names)
            loop_resources.callback(observation.close)
            observation_body,observation_fixed=spray_nozzle_body(ET.parse(bundle['urdf']).getroot(),robot._articulation_view._metadata.link_indices,arm['tcp_link'])
        loop_resources.enter_context(simulation_gc_scope())
        world.play();step=0;begin=time.monotonic();last_ready=0;render_phase=0.;was_playing=True;was_active=False
        phase_timing=SimulationPhaseTiming(session=session,revision=revision)
        while running and app.is_running():
            now=time.monotonic()
            if args.duration and now-begin>args.duration:break
            phase_timing.begin(now,step+1)
            rclpy.spin_once(node,timeout_sec=0.)
            phase_timing.mark('ros_spin',time.monotonic())
            playing=world.is_playing()
            if was_playing and not playing:
                hold=np.array(robot.get_joint_positions());last_command=-math.inf
                if lease:lease.invalidate();average.flush()
                if cameras:cameras.invalidate()
            if not was_playing and playing:
                # A pause/reset creates a new session and needs an explicit stack restart.
                raise RuntimeError('Simulation clock resumed/reset; reapply runtime configuration')
            was_playing=playing
            if lease and playing:
                # Callback receipt happens after the loop's initial timestamp.
                command_active=lease.active_at_current_time()
                if command_active:
                    for name,value in zip(active,lease.target):hold[index[name]]=value
                elif was_active:
                    # Freeze once on lease loss, then retain gravity-supporting hold.
                    actual=np.array(robot.get_joint_positions())
                    for name in active:hold[index[name]]=actual[index[name]]
                was_active=command_active
            elif playing and now-last_command>.25 and math.isfinite(last_command):
                actual=np.array(robot.get_joint_positions())
                for name in active:hold[index[name]]=actual[index[name]]
                last_command=-math.inf
            if playing:robot.apply_action(ArticulationAction(joint_positions=hold))
            phase_timing.mark('command_apply',time.monotonic())
            world.step(render=False);sampled=time.monotonic()
            phase_timing.mark('physics_step',sampled)
            if playing:step+=1
            stamp=float(world.current_time);sec=int(stamp);ros_stamp=Time(sec=sec,nanosec=int((stamp-sec)*1e9))
            force_window=None
            if lease and playing:
                tf=ft_world_transform(robot,ft_body)
                incoming=np.asarray(robot.get_measured_joint_forces(joint_indices=np.array([ft_row])))[0]
                raw,gravity=sensor.measure_wrench(incoming,joint_to_ft,tf[:3,:3],bundle['ft_boundaries'][args.active_arm])
                force_window=average.add(raw,gravity,session_id=session,scene_revision=revision,step_index=step,stamp=stamp,playing=True)
                force_valid=force_window is not None and np.isfinite(np.r_[raw,gravity]).all()
            phase_timing.mark('force_readback',time.monotonic())
            render_phase+=timing['render_hz']/physics_hz
            if render_phase>=1 or not playing:
                render_phase=max(0.,render_phase-1)
                if spray_visual is not None:
                    spray_enabled,spray_plan=spray_state.current(time.monotonic(),stamp)
                    spray_report=spray_visual.update(ft_world_transform(robot,nozzle_body)@nozzle_fixed,
                                                     playing and spray_enabled,spray_plan)
                    phase_timing.mark('spray_visual',time.monotonic())
                if cameras:
                    world_from_zed=ft_world_transform(robot,zed_body)@zed_fixed if zed_body is not None else None
                    cameras.update_poses(ft_world_transform(robot,ft_body) if ft_body is not None else np.eye(4),
                                         world_from_zed=world_from_zed)
                phase_timing.mark('camera_pose',time.monotonic())
                from rb20_painting_sensors import render_sensor_and_viewport
                render_sensor_and_viewport(world,cameras,spray_visual,playing=playing,
                                           phase_timing=phase_timing)
                if observation:observation.capture()
            if step % timing['state_stride']==0:
                positions=np.asarray(robot.get_joint_positions());velocities=np.asarray(robot.get_joint_velocities())
                if observation:observation.sample(stamp,positions,velocities,ft_world_transform(robot,observation_body)@observation_fixed)
                joints_valid=playing and bool(np.isfinite(np.r_[positions,velocities]).all())
                phase_timing.mark('joint_readback',time.monotonic())
                state=JointState();state.header.stamp=ros_stamp;state.name=list(robot.dof_names);state.position=positions.tolist();state.velocity=velocities.tolist();state_pub.publish(state)
                clock=Clock();clock.clock=ros_stamp
                if playing:clock_pub.publish(clock)
                if lease:
                    measured=State();measured.header.stamp=ros_stamp;measured.header.frame_id=arm['wrench_frame']
                    measured.session_id=session;measured.scene_revision=revision;measured.step_index=step;measured.joint_names=active
                    order=[index[n] for n in active];measured.positions=positions[order].tolist();measured.velocities=velocities[order].tolist()
                    measured.efforts=np.asarray(robot.get_measured_joint_efforts())[order].tolist()
                    for message,data in [(measured.wrench,force_window.raw if force_window else np.full(6,np.nan)),(measured.gravity_wrench,force_window.gravity if force_window else np.full(6,np.nan))]:
                        message.force.x,message.force.y,message.force.z=map(float,data[:3]);message.torque.x,message.torque.y,message.torque.z=map(float,data[3:])
                    measured.joints_valid=joints_valid;measured.wrench_valid=bool(force_valid and playing);measured.playing=playing
                    if measured.joints_valid and measured.wrench_valid:lease.record_state(sec,ros_stamp.nanosec,sampled)
                    force_pub.publish(measured)
                phase_timing.mark('state_build_publish',time.monotonic())
                phase_timing.published(time.monotonic())
                if sampled-last_ready>.2:
                    ready=dict(pid=os.getpid(),asset_revision=revision,runtime_revision=bundle.get('runtime_revision',0),session_id=session,scene_revision=revision,
                               step_index=step,updated_monotonic=sampled,joints_valid=joints_valid,wrench_valid=bool(force_valid),
                               cameras_ready=bool(cameras.ready(time.monotonic())) if cameras else args.mode=='mixed')
                    ready['ready']=all(ready[k] for k in ('joints_valid','wrench_valid','cameras_ready'))
                    if lease:ready['command_diagnostics']={**command_stats,'lease_active':lease.active(time.monotonic()),'hold_positions':[float(hold[index[n]]) for n in active],'measured_positions':[float(positions[index[n]]) for n in active]}
                    atomic_json(assets/'simulation-ready.json',ready);last_ready=sampled
                    if spray_visual is not None:
                        atomic_json(assets/'spray-visualization.json',dict(spray_report,
                            session_id=session,scene_revision=revision,updated_monotonic=sampled,
                            envelope_metadata=envelope.metadata(),
                            sensor_overlays='visible in viewport and simulation RGB/depth; shared render'))
            phase_timing.mark('readiness',time.monotonic())
            while pending:
                from platform_task_scene import apply_task_surface
                try:ack=apply_task_surface(stage,json.loads(pending.pop(0)),expected_frame=arm['base_frame'],
                                           friction=friction if args.process=='paint' else None)
                except Exception as exc:ack=dict(engine='isaac',success=False,error=str(exc))
                if ack.get('success') and ack.get('contact_material'):
                    contact.update(ack['contact_material'])
                    contact.update(surface_id=ack['surface_id'],surface_revision=ack['revision'],frame_id=arm['base_frame'])
                    atomic_json(assets/'simulation-contact-contract.json',contact)
                ack_pub.publish(String(data=json.dumps(ack)))
            phase_timing.mark('task_scene',time.monotonic())
            if args.capture and cameras and cameras.ready(time.monotonic()):
                from PIL import Image
                args.capture.mkdir(parents=True,exist_ok=True)
                for name,(camera,_,_) in cameras.cameras.items():Image.fromarray(np.asarray(camera.get_rgba())[:,:,:3].astype(np.uint8)).save(args.capture/(name+'.png'))
                args.capture=None
            phase_timing.mark('capture',time.monotonic())
            slow=phase_timing.finish(time.monotonic())
            if slow:
                print('PLATFORM_SLOW_STEP '+json.dumps(slow),flush=True)
                atomic_json(assets/'simulation-phase-timing.json',phase_timing.snapshot())
        atomic_json(assets/'simulation-phase-timing.json',phase_timing.snapshot())
        stage.GetRootLayer().Export(str(assets/'scene.usda'))
    except Exception:
        # Kit shutdown may terminate Python before an outer traceback is printed.
        import traceback
        traceback.print_exc()
        raise
    finally:
        loop_resources.close()
        (assets/'simulation-ready.json').unlink(missing_ok=True)
        if node is not None:
            node.destroy_node()
            import rclpy
            rclpy.try_shutdown()
        app.close()


if __name__=='__main__':main()
