#!/usr/bin/env python3
"""Standalone measured-state application with steady command lease and real rendering."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import uuid


def atomic_json(path, content):
    tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(content)+'\n'); tmp.replace(path)


def close_runtime(node, shutdown, app_close, exit_code):
    """Kit cleanup must run even if ROS was already shut down by a signal."""
    try:
        if node is not None:
            node.destroy_node()
    finally:
        try:
            if shutdown is not None:
                shutdown()
        finally:
            app_close(exit_code=exit_code)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--no-window',action='store_true',
        default=not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')),
        help='Render without a desktop window (automatic when no display is available)')
    parser.add_argument('--duration',type=float,default=0.,help='Exit after this many steady seconds (verification)')
    parser.add_argument('--trace-timing',action='store_true',default=os.environ.get('RB20_PAINTING_TRACE_TIMING')=='1',help='Attribute source state gaps to physics, render, DDS and callback timings')
    parser.add_argument('--verify-reset',action='store_true',help='Exercise owned timeline Pause/Play and Stop/Play during verification')
    parser.add_argument('--capture',type=Path,help='Save actual camera RGB PNG and metric depth NPY once ready')
    args=parser.parse_args()
    import charset_normalizer  # Preload coherent pip modules before Kit.
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.no_window,'width':1440,'height':1000,'renderer':'RaytracedLighting','disable_viewport_updates':args.no_window})
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    root=Path(os.environ.get('SKETCH_WORKSPACE',Path(__file__).resolve().parents[1]))
    ready_path=root/'.runtime/rb20_painting/managed-ready.json'
    logs=root/'logs/rb20-painting'; logs.mkdir(parents=True,exist_ok=True)
    node=None; camera_node=None; camera_context=None; exit_code=0
    try:
        import numpy as np
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        enable_extension('isaacsim.sensors.camera')
        import rclpy
        import rclpy.qos
        from rclpy.signals import SignalHandlerOptions
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import PoseStamped
        from scipy.spatial.transform import Rotation
        from isaac_painting_msgs.msg import State, Command
        from isaacsim.core.utils.types import ArticulationAction
        from rb20_painting_scene import build_scene,ft_world_transform
        from rb20_painting_sensors import CommandLease, RenderedCameras, transform_wrench,gravity_wrench,invalid_measurements,LoopTrace,transport_qos_profiles,publication_mode,WrenchWindowAverage
        world,robot,manifest,scene,ft_row,ft_body,joint_to_ft=build_scene(root)
        revision=manifest['scene_revision']
        if os.environ.get('RB20_PAINTING_SCENE_REVISION',revision)!=revision:
            raise RuntimeError('Supervisor scene revision differs from generated assets')
        session=os.environ.get('RB20_PAINTING_SESSION_ID') or str(uuid.uuid4())
        lease=CommandLease(session,revision)
        with publication_mode('ASYNCHRONOUS'):
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node=rclpy.create_node('rb20_painting_isaac')
        stream_qos,command_qos=transport_qos_profiles(rclpy.qos)
        state_pub=node.create_publisher(State,'/isaac/rb20_painting/state',stream_qos)
        clock_pub=node.create_publisher(Clock,'/clock',stream_qos)
        pose_pub=node.create_publisher(PoseStamped,'/isaac/rb20_painting/measured_ft_pose',stream_qos)
        def on_command(msg):
            if not world.is_playing():
                return
            if msg.enabled and any(not manifest['limits'][name]['lower']<=v<=manifest['limits'][name]['upper'] for name,v in zip(manifest['joint_names'],msg.positions)):
                return
            lease.accept(msg,time.monotonic())
        node.create_subscription(Command,'/isaac/rb20_painting/command',on_command,command_qos)
        # Large reliable image writes have their own DDS participant/socket.
        # Control state, clock and command discovery never share that writer context.
        camera_context=rclpy.context.Context()
        with publication_mode('ASYNCHRONOUS'):
            rclpy.init(context=camera_context,signal_handler_options=SignalHandlerOptions.NO)
            camera_node=rclpy.create_node('rb20_painting_cameras',context=camera_context)
        print('RB20_ROS_TRANSPORT '+json.dumps({'control':'ASYNCHRONOUS','cameras':'ASYNCHRONOUS','transport':'UDPv4','state_reliability':'BEST_EFFORT','separate_camera_context':True}),flush=True)
        cameras=RenderedCameras(camera_node,manifest,scene)
        hold=np.array(robot.get_joint_positions(),copy=True)
        world.play()
        order=[robot.dof_names.index(n) for n in manifest['joint_names']]
        robot_order=[manifest['joint_names'].index(n) for n in robot.dof_names]
        trace=LoopTrace(args.trace_timing)
        wrench_average=WrenchWindowAverage(5); wrench_window=None
        step=0; render_phase=0.; previous_playing=True; stopped=False
        begin=time.monotonic(); last_ready=0.; last_perf=begin; sample_sim=world.current_time
        reset_phase=0; reset_deadline=begin+5.
        physics_count=render_count=0; pace_wall=begin; pace_sim=world.current_time
        evidence=[]; valid_joints=False; valid_wrench=False
        while running and app.is_running():
            now=time.monotonic()
            if args.duration and now-begin>=args.duration:
                break
            if args.verify_reset and now>=reset_deadline:
                if reset_phase==0:
                    world.pause(); reset_deadline=now+1.
                elif reset_phase==1:
                    world.play(); reset_deadline=now+5.
                elif reset_phase==2:
                    world.stop(); reset_deadline=now+1.
                elif reset_phase==3:
                    world.play(); reset_deadline=float('inf')
                reset_phase+=1
            if step%5==0 or not world.is_playing():
                trace.call('rclpy_spin',rclpy.spin_once,node,timeout_sec=0.,step=step)
            playing=world.is_playing()
            if world.is_stopped():
                stopped=True
            if not playing and previous_playing:
                lease.invalidate(); cameras.invalidate(); wrench_average.flush()
            if playing and not previous_playing:
                lease.invalidate()
                if stopped:
                    session=str(uuid.uuid4()); lease=CommandLease(session,revision)
                    wrench_average.flush()
                    world.reset(); step=0; stopped=False
                    cameras.invalidate()
                    for camera,_,_ in cameras.cameras.values():
                        camera.initialize()
                hold=np.array(robot.get_joint_positions(),copy=True)
                pace_wall=now; pace_sim=world.current_time
            # The command callback can accept after the loop-start timestamp.
            now=time.monotonic()
            active=playing and lease.active_at_current_time()
            if active:
                hold=lease.target[robot_order].copy()
            else:
                # Expired command cannot leave its old target moving the articulation.
                # Hold measured position once on the transition, then retain a gravity-supporting servo.
                if playing and getattr(lease,'_was_active',False):
                    hold=np.array(robot.get_joint_positions(),copy=True); lease.invalidate()
            lease._was_active=active
            if playing and step%5==0:
                robot.apply_action(ArticulationAction(joint_positions=hold))
            trace.call('world_step',world.step,render=False,step=step)
            physics_sample_monotonic=time.monotonic()
            if playing:
                step+=1; physics_count+=1
            # Anti-alias actual reaction before decimation. Every sample retains its
            # own physics step and simulation stamp; invalid windows never become zeros.
            sample_stamp=float(world.current_time)
            if playing:
                tf=ft_world_transform(robot,ft_body)
                incoming=np.asarray(robot.get_measured_joint_forces(joint_indices=np.array([ft_row])))[0]
                raw_sample=-transform_wrench(incoming,joint_to_ft)
                gravity_sample=gravity_wrench(manifest['ft_boundary']['mass_kg'],manifest['ft_boundary']['cog_ft_m'],tf[:3,:3])
                wrench_window=wrench_average.add(raw_sample,gravity_sample,session_id=session,scene_revision=revision,
                                                 step_index=step,stamp=sample_stamp,playing=True)
            else:
                wrench_average.flush(); wrench_window=None
            render_phase+=scene['render_hz']/scene['physics_hz'] if playing else 0
            if render_phase>=1:
                render_phase=max(0.,render_phase-1)
                tf=ft_world_transform(robot,ft_body)
                trace.call('camera_pose',cameras.update_poses,tf,step=step)
                trace.call('world_render',world.render,step=step); render_count+=int(playing)
                trace.call('camera_publish',cameras.publish,time.monotonic(),step=step)
            stamp=float(world.current_time); sec=int(stamp)
            if not playing:
                trace.call('paused_render',world.render,step=step)
            if step%5==0 or not playing:
                if playing:
                    tf=ft_world_transform(robot,ft_body)
                    positions=np.asarray(robot.get_joint_positions())[order]
                    velocities=np.asarray(robot.get_joint_velocities())[order]
                    efforts=np.asarray(robot.get_measured_joint_efforts())[order]
                    if wrench_window is not None:
                        wrench=wrench_window.raw; gravity=wrench_window.gravity
                        stamp=wrench_window.stamp; sec=int(stamp)
                    else:
                        wrench=np.full(6,np.nan); gravity=np.full(6,np.nan)
                    valid_joints=bool(playing and np.all(np.isfinite(np.r_[positions,velocities,efforts])))
                    valid_wrench=bool(playing and wrench_window is not None and np.all(np.isfinite(np.r_[wrench,gravity])))
                else:
                    positions,velocities,efforts,wrench,gravity=invalid_measurements()
                    valid_joints=valid_wrench=False
                if step%5==0 or not playing:
                    state=State(); state.header.stamp.sec=sec; state.header.stamp.nanosec=int((stamp-sec)*1e9)
                    state.header.frame_id='ft_link'; state.session_id=session; state.scene_revision=revision; state.step_index=step
                    state.joint_names=manifest['joint_names']; state.positions=positions.tolist(); state.velocities=velocities.tolist(); state.efforts=efforts.tolist()
                    for msg,data in [(state.wrench,wrench),(state.gravity_wrench,gravity)]:
                        msg.force.x,msg.force.y,msg.force.z=map(float,data[:3]); msg.torque.x,msg.torque.y,msg.torque.z=map(float,data[3:])
                    state.joints_valid=valid_joints; state.wrench_valid=valid_wrench; state.playing=playing
                    if playing and valid_joints and valid_wrench:
                        lease.record_state(state.header.stamp.sec,state.header.stamp.nanosec,physics_sample_monotonic)
                    else:
                        lease.invalidate()
                    trace.call('state_publish',state_pub.publish,state,step=step)
                    trace.state_published(time.monotonic(),step)
                    if playing:
                        clock=Clock(); clock.clock=state.header.stamp; trace.call('clock_publish',clock_pub.publish,clock,step=step)
                        # Independent PhysX body transform, never reconstructed from FK.
                        if step%25==0 and valid_joints:
                            pose_msg=PoseStamped(); pose_msg.header.stamp=state.header.stamp; pose_msg.header.frame_id='link0'
                            pose_msg.pose.position.x,pose_msg.pose.position.y,pose_msg.pose.position.z=map(float,tf[:3,3])
                            q=Rotation.from_matrix(tf[:3,:3]).as_quat()
                            pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w=map(float,q)
                            trace.call('pose_publish',pose_pub.publish,pose_msg,step=step)
            now=time.monotonic()
            if args.capture and cameras.ready(now):
                from PIL import Image
                args.capture.mkdir(parents=True,exist_ok=True)
                for name,(camera,_,_) in cameras.cameras.items():
                    Image.fromarray(np.asarray(camera.get_rgba())[:,:,:3].astype(np.uint8)).save(args.capture/(name+'.png'))
                    np.save(args.capture/(name+'-depth.npy'),camera.get_current_frame()['distance_to_image_plane'])
                args.capture=None
            if (step%5==0 or not playing) and now-last_ready>=.25:
                ready={'pid':os.getpid(),'session_id':session,'scene_revision':revision,'model_id':manifest['model_id'],
                       'step_index':step,'updated_monotonic':now,'joints_valid':valid_joints,'wrench_valid':valid_wrench,
                       'cameras_ready':playing and cameras.ready(now)}
                atomic_json(ready_path,ready); last_ready=now
                evidence.append({'step_index':step,'stamp':stamp,'wrench':wrench.tolist(),'gravity_wrench':gravity.tolist(),
                                 'playing':playing,'session_id':session,'joint_positions':positions.tolist(),'camera_frames':cameras.frames.copy(),'command_active':active})
                if len(evidence)>100:
                    evidence.pop(0)
            if now-last_perf>=5.:
                elapsed=now-last_perf
                atomic_json(logs/'sim-performance.json',{'configured_physics_hz':scene['physics_hz'],'configured_render_hz':scene['render_hz'],
                            'physics_hz':physics_count/elapsed,'render_hz':render_count/elapsed,'realtime_factor':(world.current_time-sample_sim)/elapsed,'state_max_gap_s':trace.max_gap})
                atomic_json(logs/'sim-measurements.json',{'session_id':session,'scene_revision':revision,'samples':evidence})
                if args.trace_timing:
                    atomic_json(logs/'sim-timing-trace.json',{'pid':os.getpid(),'session_id':session,'updated_monotonic':now,**trace.report()})
                last_perf=now; sample_sim=world.current_time; physics_count=render_count=0
            if playing:
                ahead=(world.current_time-pace_sim)-(now-pace_wall)
                if 0<ahead<=.1:
                    time.sleep(ahead)
                elif ahead<-.1:
                    pace_wall=now; pace_sim=world.current_time
            previous_playing=playing
        atomic_json(logs/'sim-measurements.json',{'session_id':session,'scene_revision':revision,'samples':evidence})
        if args.trace_timing:
            atomic_json(logs/'sim-timing-trace.json',{'pid':os.getpid(),'session_id':session,'updated_monotonic':time.monotonic(),**trace.report()})
        print('RB20_SIM_EXIT '+json.dumps({'step_index':step,'cameras':cameras.frames,'joints_valid':valid_joints,'wrench_valid':valid_wrench}),flush=True)
    except Exception:
        exit_code=1
        import traceback
        traceback.print_exc()
        raise
    finally:
        if ready_path.exists():
            try:
                if json.loads(ready_path.read_text()).get('pid')==os.getpid():
                    ready_path.unlink()
            except (OSError,ValueError):
                pass
        try:
            if camera_node is not None:
                camera_node.destroy_node()
        finally:
            try:
                if camera_context is not None:
                    rclpy.try_shutdown(context=camera_context)
            finally:
                close_runtime(node,rclpy.try_shutdown if node else None,app.close,exit_code)

if __name__=='__main__':
    main()
