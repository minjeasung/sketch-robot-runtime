"""Read the running MoveIt model and persist snapshots only in owned state."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from .model import JOINTS, parse_stack_model, save_descriptor
from .config import owned_path

LIMIT_FIELDS = ('has_position_limits', 'min_position', 'max_position',
                'has_velocity_limits', 'max_velocity', 'has_acceleration_limits', 'max_acceleration')


def run(config):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_share_directory

    class ModelNode(Node):
        def __init__(self):
            super().__init__('snucem_sketch_model')
            self.urdf = ''
            self.model = None
            self.model_at = float('-inf')
            self.pending = None
            self.pending_at = 0
            self.error = 'waiting for live model'
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/snucem_sketch/model', latched)
            self.create_subscription(String, '/robot_description', self.on_description, latched)
            self.client = self.create_client(GetParameters, '/move_group/get_parameters')
            self.names = ['robot_description', 'robot_description_semantic'] + [
                'robot_description_planning.joint_limits.'+n+'.'+f for n in JOINTS for f in LIMIT_FIELDS]
            self.create_timer(.5, self.query)
            self.create_timer(.2, self.publish)

        def on_description(self, msg):
            if msg.data != self.urdf:
                self.model = None
            self.urdf = msg.data

        def query(self):
            if self.pending is not None:
                if time.monotonic()-self.pending_at > 2:
                    self.pending.cancel()
                    self.pending = None
                    self.model = None
                    self.error = 'MoveIt parameter query timed out'
                return
            if not self.urdf or not self.client.service_is_ready():
                return
            req = GetParameters.Request(names=self.names)
            self.pending = self.client.call_async(req)
            self.pending_at = time.monotonic()
            self.pending.add_done_callback(self.received)

        def received(self, future):
            if future is not self.pending:
                return
            self.pending = None
            try:
                response = future.result()
                values = [None if v.type == 0 else {1:v.bool_value, 2:v.integer_value,
                          3:v.double_value, 4:v.string_value}.get(v.type) for v in response.values]
                if len(values) != len(self.names) or values[0] != self.urdf:
                    raise ValueError('MoveIt and robot_state_publisher models differ')
                limits = {n:dict(zip(LIMIT_FIELDS, values[2+i*len(LIMIT_FIELDS):2+(i+1)*len(LIMIT_FIELDS)]))
                          for i, n in enumerate(JOINTS)}
                roots = {}
                tree = ET.fromstring(self.urdf)
                for mesh in tree.findall('.//mesh'):
                    uri = mesh.get('filename', '')
                    if not uri.startswith('package://'):
                        raise ValueError('expected installed mesh resource')
                    package = uri[10:].split('/', 1)[0]
                    roots[package] = get_package_share_directory(package)
                if tree.find("link[@name='spray_eoat']/collision/geometry/mesh") is None:
                    raise ValueError('live spray tool collision mesh missing')
                model = parse_stack_model(self.urdf, values[1], limits, roots)
                if model['model_id'] != config.model_id:
                    raise ValueError('configured robot differs from live stack')
                self.path = save_descriptor(model, config.state_root)
                self.model, self.model_at, self.error = model, time.monotonic(), ''
            except Exception as exc:
                self.model, self.error = None, str(exc)

        def publish(self):
            fresh = self.model is not None and time.monotonic()-self.model_at <= 2
            payload = dict(fingerprint=self.model['fingerprint'] if fresh else '',
                           profile=str(self.path) if fresh else '', error='' if fresh else self.error)
            # This file is also the management process's readiness handshake.
            state = Path(config.state_root)
            state.mkdir(parents=True, exist_ok=True)
            temp = owned_path(state, 'model-status.tmp')
            temp.write_text(json.dumps(dict(payload, written_at=time.time())), encoding='utf-8')
            temp.replace(owned_path(state, 'model-status.json'))
            self.pub.publish(String(data=json.dumps(payload)))

    rclpy.init()
    node = ModelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
