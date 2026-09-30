"""ZED selection and geometric plan status, with no robot command interfaces."""
import json

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .multi_surface_execution import MultiSurfaceMixin
from .zed_spray_execution import ZedSprayExecutionMixin


class ZedPreviewNode(ZedSprayExecutionMixin, Node):
    # Reuse only the existing non-contact plane-selection callbacks. Do not
    # inherit the execution/D405 scan methods or construct any action client.
    process_mode = 'spray'
    _multi_on_catalog = MultiSurfaceMixin._multi_on_catalog
    _multi_on_select = MultiSurfaceMixin._multi_on_select
    _multi_on_activate = MultiSurfaceMixin._multi_on_activate
    _multi_activate = MultiSurfaceMixin._multi_activate
    _multi_lock_zed_plane = MultiSurfaceMixin._multi_lock_zed_plane
    _multi_pose = MultiSurfaceMixin._multi_pose
    _multi_status = MultiSurfaceMixin._multi_status

    def __init__(self, **kwargs):
        super().__init__('zed_preview', **kwargs)
        self.executing = False
        self._motion_abort_requested = False
        self._multi_catalog = dict(generation='', planes=[])
        self._multi_selected = []
        self._multi_refined = {}
        self._multi_active_id = ''
        self._multi_current = None
        self._multi_queue = []
        self._multi_state = 'empty'
        self._preview_plan = {}
        self._current_work_area_id = ''
        self._current_plane_generation_id = ''
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_pub = self.create_publisher(String, '/painting_system/process_mode', latched)
        self._readiness_pub = self.create_publisher(String, '/painting_system/readiness', latched)
        self._execution_pub = self.create_publisher(String, '/painting_system/execution_status', latched)
        self._multi_status_pub = self.create_publisher(String, '/painting_system/planes', latched)
        self._init_zed_spray()
        self.create_subscription(String, '/perception/target_planes', self._multi_on_catalog, latched)
        self.create_subscription(String, '/painting_system/select_planes', self._multi_on_select, 10)
        self.create_subscription(String, '/painting_system/activate_plane', self._multi_on_activate, 10)
        self.create_subscription(String, '/painting_system/set_process_mode', self._on_set_mode, 10)
        self.create_subscription(String, '/painting_system/plan_status', self._on_plan_status, latched)
        self.create_timer(.25, self._publish_state)
        self._publish_state()

    def _multi_busy(self):
        return False

    @staticmethod
    def _execution_snapshot_updates_locked(*_args):
        return False

    @staticmethod
    def _mark_scene_dirty(*_args):
        # Preview has no MoveIt planning scene or robot collision model.
        pass

    def _reset_d405_refined_lock(self, _reason, clear_surface=False):
        # Shared selection callback resets the contact-workflow lock. No D405
        # interfaces exist in this node; only the preview candidate is cleared.
        self._preview_plan = {}

    @staticmethod
    def _canonical_world_frame(frame):
        return 'link0' if frame in ('World', 'world', 'link0') else frame

    def _lookup_transform_to_base(self, frame, timeout_s=.2):
        try:
            return self.tf_buffer.lookup_transform('link0', frame, Time(),
                                                  timeout=Duration(seconds=timeout_s))
        except TransformException:
            return None

    def _set_dynamic_work_area_corners(self, corners):
        self.dynamic_work_area_corners = corners

    def _publish_execution_status(self, state, reason):
        self._preview_plan = {}
        self._execution_pub.publish(String(data=json.dumps(dict(
            state=state, reason=reason, running=False, planning_only=True))))

    def _on_set_mode(self, msg):
        self._publish_mode('PREVIEW_REQUIRES_SPRAY_MODE' if msg.data != 'spray' else '')

    def _publish_mode(self, error=''):
        self._mode_pub.publish(String(data=json.dumps(dict(
            mode='spray', planning_only=True, spray_motion_test=False, error=error))))

    def _on_plan_status(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('plan status must be an object')
        except (ValueError, TypeError):
            payload = {}
        self._preview_plan = payload

    def readiness_payload(self):
        plan = self._preview_plan
        generated = bool(
            self._zed_plane_accepted and plan.get('process_mode') == 'spray'
            and plan.get('state') == 'generated'
            and plan.get('work_area_id') == self._current_work_area_id
            and plan.get('plane_generation_id') == self._current_plane_generation_id
            and plan.get('path_id') and plan.get('plan_hash'))
        return dict(
            process_mode='spray', planning_only=True, ready=False, running=False,
            state='PREVIEW', dry_run=True, real_painting_enabled=False, abort_reason='NONE',
            work_area_id=self._current_work_area_id,
            plane_generation_id=self._current_plane_generation_id,
            path_id=plan['path_id'] if generated else '',
            plan_hash=plan['plan_hash'] if generated else '',
            plan_blockers=['ROBOT_VALIDATION_NOT_PERFORMED'],
            checks=dict(zed_surface_valid=bool(self._zed_target_lock),
                        zed_plane_accepted=self._zed_plane_accepted,
                        zed_work_area_locked=self._zed_plane_accepted,
                        current_plan_generated=generated, current_plan_validated=False))

    def _publish_state(self):
        self._accept_pending_zed_area()
        self._publish_mode()
        self._readiness_pub.publish(String(data=json.dumps(self.readiness_payload())))


def main(args=None):
    rclpy.init(args=args)
    node = ZedPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
