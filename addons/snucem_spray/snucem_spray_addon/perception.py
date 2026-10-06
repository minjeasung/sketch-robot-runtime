"""Run the installed perception class with an add-on-only export timer."""
import json
from pathlib import Path
import time
import uuid

from .compatibility import require_upstream
from .model import file_hash
from .planes import measured_cells, snapshot, StableSupport


def run(config):
    require_upstream(config.upstream_root)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from rb20_spray.node import RB20SprayNode, make_parser
    import rb20_spray.node as upstream

    if not Path(upstream.__file__).resolve().is_relative_to(Path(config.upstream_root)):
        raise ValueError('rb20_spray import resolved outside the configured upstream')
    calibration = Path(config.calibration_file)
    if not calibration.is_file():
        raise ValueError('calibration_file must identify the active measured extrinsic')
    # Only read-only perception settings are accepted. Never pass arbitrary output/cache args.
    permitted = {'--spray-config', '--spray-perception-hz', '--hbeam-lock-samples',
                 '--max-extracted-planes', '--max-cached-planes', '--plane-threshold'}
    args = list(config.perception_args)
    if len(args) % 2 or any(args[i] not in permitted for i in range(0, len(args), 2)):
        raise ValueError('unsupported perception argument')
    parsed = make_parser().parse_args(args)
    parsed.base_frame, parsed.tool_frame = 'link0', 'tcp'
    parsed.zed_points_topic = config.points_topic

    class Exporter(RB20SprayNode):
        def __init__(self):
            self.source_session = uuid.uuid4().hex
            self.export_at = float('-inf')
            self.stable_support = StableSupport()
            self.export_pub = None
            super().__init__(parsed)
            self.export_pub = self.create_publisher(String, '/snucem_sketch/plane_catalog',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def _on_timer(self):
            # Deliberately replace the upstream CONTROL timer. The inherited
            # point-cloud callbacks still own extraction and persistent planes.
            now = time.monotonic()
            if self.export_pub is None or now-self.export_at < .2:
                return
            self.export_at = now
            try:
                if any(name in ('rb20_spray_shared_autonomy', 'rb20_spray_shared_autonomy_isaac')
                       for name, _ in self.get_node_names_and_namespaces()):
                    raise ValueError('duplicate upstream perception node')
                entries = []
                with self._observation_lock:
                    # This upstream timestamp advances only AFTER stamped TF,
                    # cloud filtering and measured-patch observation succeed.
                    # Receiving an invalid/replayed cloud cannot renew a lock.
                    observation = self._patch_observations.get('zed')
                    if observation is None or not 0 <= now-observation[1] <= 1.0:
                        raise ValueError('processed ZED observation stale')
                    revision = self._workcell_revision
                    measured = [(ident, patch.point.copy(), patch.normal.copy(), patch.support.copy(), float(patch.rms))
                                for ident, patch, _ in self._patch_memories['zed'].locked_entries()]
                # Triangulation is outside the upstream observation lock.
                for ident, point, normal, support, rms in measured:
                    cells = measured_cells(support, normal)
                    if cells and len(support) >= 80 and rms <= .015:
                        entries.append(dict(plane_id='zed:'+str(ident), center=point.tolist(),
                            normal=normal.tolist(), cells=cells,
                            inlier_count=len(support), rms_m=rms))
                session = self.source_session+':'+str(revision)
                entries = self.stable_support.update(entries, session)
                data = snapshot(entries, session,
                                self.get_clock().now().nanoseconds, file_hash(calibration))
            except (ValueError, OSError, KeyError) as exc:
                data = dict(schema_version=1, planes=[], revision='', error=str(exc),
                            source_session=self.source_session)
            self.export_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    rclpy.init(args=['--ros-args', '-r', '__node:=snucem_sketch_perception'])
    executor = MultiThreadedExecutor(num_threads=3)
    node = Exporter()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
