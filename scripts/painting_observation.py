"""Optional observational camera. Never feeds control or perception topics."""
import json
from pathlib import Path
import queue
import threading
import numpy as np


class PaintingObservation:
    def __init__(self, directory, node, session, joint_names):
        from isaacsim.sensors.camera import Camera
        from scipy.spatial.transform import Rotation
        from std_msgs.msg import String
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        if (self.directory/'measured.jsonl').exists():
            raise ValueError('Use a fresh observation directory for each simulation session')
        self.samples = (self.directory / 'measured.jsonl').open('w', buffering=1024*1024)
        self.events = (self.directory / 'events.jsonl').open('w', buffering=1)
        self.queue = queue.Queue(maxsize=120)
        self.error = None
        self.last_frame = -1
        eye = np.array([4.9, .5, 3.0])
        target = np.array([0., -.65, 1.25])
        forward = target-eye; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0,0,1]); right /= np.linalg.norm(right)
        rotation = np.column_stack((right, np.cross(forward,right), forward))
        self.world_camera = np.eye(4)
        self.world_camera[:3,:3] = rotation; self.world_camera[:3,3] = eye
        quat = Rotation.from_matrix(rotation).as_quat()
        self.camera = Camera('/World/ObservationCamera', name='painting_observation', frequency=-1, resolution=(1280,720))
        self.camera.set_focal_length(2.8)
        self.camera.set_horizontal_aperture(3.6)
        self.camera.set_clipping_range(.05,20.)
        self.camera.initialize()
        self.camera.set_world_pose(position=eye,orientation=np.r_[quat[3],quat[:3]],camera_axes='ros')
        self.metadata = dict(session=session, joint_names=list(joint_names), fps=15,
            world_from_camera=self.world_camera.tolist(), K=np.asarray(self.camera.get_intrinsics_matrix()).tolist(),
            timestamp_basis='camera rendering_time and physics simulation time', recorded_frames=0, dropped_frames=0)
        (self.directory/'camera.json').write_text(json.dumps(self.metadata,indent=2))
        node.create_subscription(String, '/painting_system/execution_events', self.event, 100)
        self.worker = threading.Thread(target=self.write_frames, daemon=True)
        self.worker.start()

    def event(self, msg):
        self.events.write(msg.data+'\n')

    def sample(self, stamp, positions, velocities, tcp):
        self.samples.write(json.dumps(dict(sim_time=stamp, positions=list(map(float,positions)),
            velocities=list(map(float,velocities)), tcp_world=np.asarray(tcp).tolist()))+'\n')

    def capture(self):
        from rb20_painting_sensors import camera_frame_index
        if self.error:
            raise RuntimeError('Observation recording failed: '+self.error)
        data = self.camera.get_current_frame()
        index = camera_frame_index(data.get('rendering_frame'))
        stamp = data.get('rendering_time', 0.)
        rgb = data.get('rgb')
        if rgb is None:
            rgb = self.camera.get_rgba()
        if index is None or index <= self.last_frame or stamp <= 0 or rgb is None or np.size(rgb)==0:
            return
        try:
            self.queue.put_nowait((float(stamp), np.ascontiguousarray(rgb[:,:,:3]).copy()))
        except queue.Full:
            self.metadata['dropped_frames'] += 1
        self.last_frame = index

    def write_frames(self):
        from PIL import Image
        frames=self.directory/'frames'
        frames.mkdir(exist_ok=True)
        try:
            with (self.directory/'frames.jsonl').open('w') as stream:
                while True:
                    item = self.queue.get()
                    if item is None: break
                    stamp, rgb = item
                    Image.fromarray(rgb).save(frames/f"{self.metadata['recorded_frames']:07d}.jpg",quality=90)
                    stream.write(json.dumps(dict(frame=self.metadata['recorded_frames'],sim_time=stamp))+'\n')
                    self.metadata['recorded_frames'] += 1
        except Exception as exc:
            self.error = repr(exc)

    def close(self):
        if self.worker.is_alive():
            self.queue.put(None)
            self.worker.join(timeout=30)
        self.samples.close(); self.events.close()
        self.metadata['error'] = self.error
        (self.directory/'camera.json').write_text(json.dumps(self.metadata,indent=2))
