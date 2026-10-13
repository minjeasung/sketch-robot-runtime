#!/usr/bin/env python3
"""Run Construct and official camera services with local ZED stream input."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]

def managed_inputs_ready(root):
    for name, marker in [('camera-keepalive', 'CONSTRUCT_CAMERA_STREAMING'),
                         ('environment', 'CONSTRUCT_CAMERA_VIEW_POSE_READY')]:
        path = Path(root) / 'logs' / ('construct-' + name + '.log')
        if not path.exists() or marker not in path.read_text(errors='replace'):
            return False
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-window',action='store_true')
    parser.add_argument('--managed-runtime',action='store_true',help='Provide devices for the original Sketch runtime')
    parser.add_argument('--process-mode', choices=['spray','weld'], default='spray')
    parser.add_argument('--model-id', choices=['construct_left','construct_right'], default='construct_left')
    args=parser.parse_args()
    ready_path = ROOT / '.runtime/construct/managed-ready.json'
    if args.managed_runtime:
        ready_path.unlink(missing_ok=True)
    (ROOT/'logs').mkdir(exist_ok=True)
    for port in (8100,8101):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',port))==0:
                raise SystemExit(f'Port {port} already in use. Stop the previous Construct camera demo first.')
    # Existing robot-only launchers must be closed before starting a new world.
    processes=[];logs=[]
    def launch(name,command,extra_env=None):
        log=(ROOT/'logs'/f'construct-{name}.log').open('w');logs.append(log)
        env=dict(os.environ,**(extra_env or {}))
        process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,process))
        return process
    def check(ignore=None):
        for name,p in processes:
            if p is ignore:continue
            if p.poll() is not None:raise RuntimeError(f'{name} exited ({p.returncode}); see logs/construct-{name}.log')
    def interrupt(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    try:
        command=['bash','scripts/run_construct_isaac.sh','--capture',str(ROOT/'logs/construct-zed-scene.png')]
        if args.no_window or os.environ.get('CONSTRUCT_HEADLESS') == '1':command.append('--no-window')
        launch('zed-scene',command,{'CONSTRUCT_ZED':'1','OMNI_KIT_ACCEPT_EULA':'YES', 'CONSTRUCT_PROCESS_MODE':args.process_mode, 'CONSTRUCT_WELD_MODEL':args.model_id})
        launch('moveit',['bash','scripts/run_construct_moveit.sh'],{'CONSTRUCT_PROCESS_MODE':args.process_mode})
        launch('outpost',['bash','scripts/run_construct_camera.sh'])
        launch('michelo',['bash','scripts/run_construct_michelo.sh'])
        launch('environment',['bash','scripts/run_construct_environment.sh'])
        print('Official Outpost: http://127.0.0.1:8100/console/\n'
              'Official Michelo: http://127.0.0.1:8101/test/manual/camera/\n'
              'Isaac stereo input: stream://127.0.0.1:30000',flush=True)
        deadline=time.monotonic()+300
        while time.monotonic()<deadline:
            check()
            try:
                for port in (8100,8101):
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}/health',timeout=2) as response:
                        if response.status!=200:raise ValueError('Service is not healthy')
                break
            except (OSError,ValueError):pass
            time.sleep(.5)
        else:raise TimeoutError('Official services did not become healthy; inspect logs')
        connector = launch('camera-connect', ['python3', 'scripts/connect_construct_stream.py'])
        while connector.poll() is None:
            check(ignore=connector); time.sleep(.2)
        if connector.returncode != 0:
            raise RuntimeError('Camera connect failed; see logs/construct-camera-connect.log')
        processes.remove(('camera-connect', connector))
        if args.managed_runtime:
            launch('camera-keepalive', [str(ROOT / '.venv-api/bin/python'), '-u',
                                      'scripts/construct_camera_keepalive.py'])
            deadline = time.monotonic() + 180
            while not managed_inputs_ready(ROOT):
                check()
                if time.monotonic() >= deadline:
                    raise TimeoutError('Camera stream or initial view is not ready')
                time.sleep(.2)
            temporary = ready_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'pid': os.getpid()}))
            temporary.replace(ready_path)
        print('CONSTRUCT_ZED_READY — Open the official Michelo camera page to start preview. '
              'Ctrl+C stops this launcher\'s processes.',flush=True)
        while True:
            check();time.sleep(.5)
    except KeyboardInterrupt:pass
    finally:
        # A second Ctrl+C must not interrupt cleanup and leave SDK/Isaac children.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if args.managed_runtime:
            ready_path.unlink(missing_ok=True)
        for name,p in reversed(processes):
            if p.poll() is None:
                # Allow the official Outpost lifespan to close camera adapters.
                if name=='outpost':p.terminate()
                else:os.killpg(p.pid,signal.SIGTERM)
        deadline=time.monotonic()+25
        while any(p.poll() is None for _,p in processes) and time.monotonic()<deadline:time.sleep(.2)
        for _,p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
            p.wait()
        for log in logs:log.close()

if __name__=='__main__':main()
