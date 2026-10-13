"""Standalone Isaac application with an explicit physics/render loop."""
import argparse
import runpy
from pathlib import Path
import signal
import json
import os
import time
from construct_render_schedule import RenderSchedule
import charset_normalizer  # Keep Kit and pip bundles from mixing Cython modules.
from isaacsim import SimulationApp

parser=argparse.ArgumentParser()
parser.add_argument('--no-window', action='store_true')
parser.add_argument('--verify-reset', action='store_true', help='Exercise Stop/Play and check the restored initial pose')
parser.add_argument('--capture', help='Save a viewport PNG after scene initialization')
args=parser.parse_args()
render_hz=float(os.environ.get('CONSTRUCT_RENDER_HZ', '20'))
schedule=RenderSchedule(physics_hz=100, render_hz=render_hz)
app=SimulationApp({'headless':args.no_window,'width':1440,'height':1000, 'create_new_stage':False})
running=True

def stop(signum,frame):
    global running
    running=False

signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
try:
    import omni.usd
    omni.usd.get_context().new_stage()
    scene=runpy.run_path(str(Path(__file__).with_name('isaac_sim_construct.py')))
    world=scene['world']
    frames = 0
    capture = None
    needs_reset = False
    perf_path=Path(__file__).resolve().parents[1]/'logs/construct-performance.json'
    sample_wall=time.monotonic(); sample_sim=world.current_time
    physics_cost=render_cost=0.; physics_count=render_count=0
    pace_wall=sample_wall; pace_sim=sample_sim
    while running and app.is_running():
        if world.is_stopped():
            needs_reset = True
        if world.is_playing() and needs_reset:
            world.reset()
            needs_reset = False
            pace_wall=time.monotonic(); pace_sim=world.current_time
        tick_start=time.monotonic()
        playing=world.is_playing()
        world.step(render=False)
        physics_cost+=time.monotonic()-tick_start; physics_count+=int(playing)
        if schedule.advance(playing):
            render_start=time.monotonic()
            if 'update_zed_pose' in scene:
                scene['update_zed_pose']()
            world.render()
            render_cost+=time.monotonic()-render_start; render_count+=1
            # Never run faster than real time against wall-clock ROS clients.
            # Reset the pacing origin when slow, paused, or rewound: no catch-up burst.
            now=time.monotonic()
            ahead=(world.current_time-pace_sim)-(now-pace_wall)
            if playing and 0 < ahead <= .1:
                time.sleep(ahead)
            elif not playing or ahead < -.1:
                pace_wall=now; pace_sim=world.current_time
        now=time.monotonic()
        if now-sample_wall >= 5:
            elapsed=now-sample_wall
            perf_path.write_text(json.dumps({'wall_time':time.time(),
                'realtime_factor':(world.current_time-sample_sim)/elapsed,
                'physics_hz':physics_count/elapsed,'render_hz':render_count/elapsed,
                'physics_step_ms':1000*physics_cost/max(physics_count,1),
                'render_ms':1000*render_cost/max(render_count,1),
                'configured_physics_hz':100,'configured_render_hz':render_hz}))
            sample_wall=now; sample_sim=world.current_time
            physics_cost=render_cost=0.; physics_count=render_count=0
        frames += 1
        if frames % 300 == 0 and 'zed_path' in scene:
            import json
            from pxr import UsdGeom
            cache=UsdGeom.XformCache()
            cameras={str(p.GetPath()):[list(row) for row in cache.GetLocalToWorldTransform(p)] for p in omni.usd.get_context().get_stage().Traverse() if p.IsA(UsdGeom.Camera)}
            (Path(__file__).resolve().parents[1]/'logs/construct-camera-poses.json').write_text(json.dumps(cameras))
        if args.capture and frames in (180,1600):
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture = capture_viewport_to_file(get_active_viewport(), args.capture)
        if args.verify_reset:
            if frames == 80:
                world.stop()
            elif frames == 90:
                world.play()
            elif frames == 170:
                import numpy as np
                error = float(np.max(np.abs(scene['robot'].get_joint_positions() - scene['pose'])))
                assert error < 0.03, f'Reset pose error: {error}'
                print(f'CONSTRUCT_RESET_OK max_error_rad={error}', flush=True)
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
