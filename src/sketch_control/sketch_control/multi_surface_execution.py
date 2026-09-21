"""Sequential D405 measurements of explicitly selected ZED planes."""
import copy
import json
import time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, String
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from rclpy.qos import QoSProfile, DurabilityPolicy
from sketch_control.rotation_utils import quat_apply, quat_from_matrix
from sketch_control.target_selector_node import _normal_to_quaternion


class MultiSurfaceMixin:
    def _init_multi_surface(self):
        self._multi_catalog = dict(generation='', planes=[])
        self._multi_queue = []
        self._multi_selected = []
        self._multi_refined = {}
        self._multi_active_id = ''
        self._multi_current = None
        self._multi_refined_result = None
        self._multi_scene_ids = set()
        self._multi_state = 'empty'
        self._multi_activation_only = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._multi_status_pub = self.create_publisher(String, '/painting_system/planes', latched)
        self._multi_target_pub = self.create_publisher(PoseStamped, '/perception/target_surface', latched)
        self._multi_refined_pub = self.create_publisher(PoseStamped, '/perception/target_surface_refined', latched)
        self._multi_target_capture_pub = self.create_publisher(Bool, '/d405/refine_target_capture', 10)
        self.create_subscription(String, '/perception/target_planes', self._multi_on_catalog, latched)
        self.create_subscription(String, '/painting_system/select_planes', self._multi_on_select, 10)
        self.create_subscription(String, '/painting_system/activate_plane', self._multi_on_activate, 10)
        self.create_subscription(String, '/perception/refined_target_record', self._multi_on_refined, 10)

    def _multi_busy(self):
        return self.executing or self._active_trajectory_goal_token is not None or self._d405_prescan_active or bool(self._multi_queue)

    def _multi_status(self, state=None, error=''):
        if state:
            self._multi_state = state
        self._multi_status_pub.publish(String(data=json.dumps(dict(
            generation=self._multi_catalog.get('generation',''), state=self._multi_state,
            selected=self._multi_selected, measured=list(self._multi_refined),
            active_id=self._multi_active_id, error=error,
            current_id=self._multi_current['id'] if self._multi_current else '',
            running=bool(self._multi_queue or self._multi_current)))))

    def _multi_on_catalog(self, msg):
        if self._multi_busy():
            self._multi_status(error='BUSY')
            return
        try:
            data = json.loads(msg.data)
            if not isinstance(data.get('generation'), str) or not isinstance(data.get('planes'), list):
                return
            ids = set()
            for plane in data['planes']:
                if plane['id'] in ids:
                    raise ValueError('duplicate plane ID')
                ids.add(plane['id'])
                for key, shape in [('center',(3,)), ('normal',(3,)), ('corners',(4,3))]:
                    arr = np.asarray(plane[key], float)
                    if arr.shape != shape or not np.isfinite(arr).all():
                        raise ValueError('invalid plane geometry')
                if abs(np.linalg.norm(plane['normal'])-1) > .001:
                    raise ValueError('invalid plane normal')
        except (ValueError, TypeError, KeyError):
            self._multi_status(error='INVALID_CATALOG')
            return
        self._multi_catalog = data
        self._multi_selected = []
        self._multi_refined = {}
        self._multi_active_id = ''
        self._segment_path = None
        self._accepted_plan_hash = ''
        self._d405_plane_accepted = False
        self.current_waypoints = []
        self._reset_d405_refined_lock('new_multi_plane_catalog', clear_surface=True)
        self._mark_scene_dirty(self)
        self._multi_status('candidates', data.get('error',''))

    def _multi_on_select(self, msg):
        if self._multi_busy() or self._motion_abort_requested:
            self._multi_status(error='BUSY_OR_ABORTED')
            return
        try:
            data = json.loads(msg.data)
            selected = data['ids']
            catalog = {p['id']:p for p in self._multi_catalog['planes']}
            if data['generation'] != self._multi_catalog['generation'] or not selected or len(selected) != len(set(selected)):
                raise ValueError('stale/empty selection')
            planes = [copy.deepcopy(catalog[p]) for p in selected]
        except (ValueError, TypeError, KeyError):
            self._multi_status(error='INVALID_SELECTION')
            return
        if self.current_joint_state is None or self._current_tcp_pose_np() is None:
            self._multi_status(error='ROBOT_STATE_UNAVAILABLE')
            return
        self._multi_activation_only = False
        self._multi_selected = list(selected)
        self._multi_refined = {}
        self._multi_queue = planes
        self._multi_start_next()

    def _multi_pose(self, plane, refined=False):
        if refined:
            return copy.deepcopy(self._multi_refined[plane['id']])
        msg = PoseStamped()
        msg.header.frame_id = self._multi_catalog['frame_id']
        # Distinct identity for every measurement, including retries.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, plane['center'])
        q = _normal_to_quaternion(plane['normal'])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def _multi_start_next(self):
        if self._motion_abort_requested:
            self._multi_cancel('ABORT')
            return
        if not self._multi_queue:
            self._multi_current = None
            self._multi_status('measured')
            self._multi_apply_active(self._multi_selected[-1])
            return
        plane = self._multi_queue.pop(0)
        self._multi_current = plane
        self._multi_refined_result = None
        self._multi_capture_started = 0.
        self._multi_target_pose = self._multi_pose(plane)
        self._multi_active_id = plane['id']
        self._current_work_area_id = ''
        self._current_plane_generation_id = ''
        self._d405_plane_accepted = False
        self._reset_d405_refined_lock('multi_target_scan', clear_surface=True)
        self._segment_path = None
        self._accepted_plan_hash = ''
        self.current_waypoints = []
        self._multi_target_pub.publish(self._multi_target_pose)
        self.dynamic_work_area_corners = None
        self._work_area_corners_signature = None
        self.on_active_surface(self._multi_target_pose)
        corners = PoseArray()
        corners.header = copy.deepcopy(self._multi_target_pose.header)
        for xyz in plane['corners']:
            p = Pose()
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.w = 1.
            corners.poses.append(p)
        self.on_work_area_corners(corners)
        if self.dynamic_surface_point is None or self.dynamic_work_area_corners is None:
            self._multi_cancel('TARGET_TF_UNAVAILABLE')
            return
        self._mark_scene_dirty(self)
        self._multi_status('measuring')
        if not self._begin_d405_prescan(mode='multi_target'):
            self._multi_cancel('D405_APPROACH_UNAVAILABLE')

    def _multi_on_refined(self, msg):
        if self._multi_current is None or self._multi_capture_started <= 0:
            return
        try:
            data = json.loads(msg.data)
            expected = self._multi_target_pose.header.stamp
            if data['target_stamp'] != dict(sec=expected.sec, nanosec=expected.nanosec):
                return
            if data['frame_id'] != self._multi_target_pose.header.frame_id:
                return
            out = copy.deepcopy(self._multi_target_pose)
            xyz, q = data['position'], data['orientation']
            if not np.isfinite(xyz+q).all() or len(xyz) != 3 or len(q) != 4:
                return
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, xyz)
            out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float,q)
            self._multi_refined_result = out
        except (ValueError, TypeError, KeyError):
            return

    def _multi_scan_done(self, success):
        if not success or self._multi_refined_result is None:
            self._multi_cancel('D405_MEASUREMENT_FAILED')
            return
        plane_id = self._multi_current['id']
        self._multi_refined[plane_id] = copy.deepcopy(self._multi_refined_result)
        self._multi_current = None
        if self._multi_activation_only:
            self._multi_activation_only = False
            self._multi_apply_active(plane_id)
            return
        self._multi_status('measuring')
        self._schedule_process_once(.1, self._multi_start_next)

    def _multi_cancel(self, reason):
        self._multi_queue = []
        self._multi_current = None
        self._multi_status('failed', reason)

    def _multi_on_activate(self, msg):
        if self._multi_busy():
            self._multi_status(error='BUSY')
            return
        try:
            data = json.loads(msg.data)
            if data['generation'] != self._multi_catalog['generation']:
                return
            self._multi_activate(data['id'])
        except (ValueError, TypeError, KeyError):
            self._multi_status(error='INVALID_ACTIVE_PLANE')

    def _multi_activate(self, plane_id):
        if plane_id not in self._multi_refined or plane_id not in self._multi_selected:
            self._multi_status(error='PLANE_NOT_MEASURED')
            return
        # A D405 front view is camera-pose dependent. Bring the wrist back to
        # the selected face before allowing a new work-area sketch.
        self._multi_activation_only = True
        self._multi_queue = [copy.deepcopy(next(p for p in self._multi_catalog['planes'] if p['id'] == plane_id))]
        self._multi_start_next()

    def _multi_apply_active(self, plane_id):
        if plane_id not in self._multi_refined or plane_id not in self._multi_selected:
            self._multi_status(error='PLANE_NOT_MEASURED')
            return
        plane = next(p for p in self._multi_catalog['planes'] if p['id'] == plane_id)
        self._reset_painting_process()
        self._segment_path = None
        self._accepted_plan_hash = ''
        self._accepted_plan_path_id = ''
        self._d405_plane_accepted = False
        self.current_waypoints = []
        self._multi_active_id = plane_id
        self.dynamic_work_area_corners = None
        self._work_area_corners_signature = None
        self._reset_d405_refined_lock('active_plane_changed', clear_surface=True)
        refined = self._multi_pose(plane, refined=True)
        # Target publication invalidates old work-area identities. Refiner is
        # configured to defer captures until arrival, so activation is read-only.
        self._multi_target_pub.publish(refined)
        self._multi_refined_pub.publish(refined)
        self.on_active_surface(refined)
        self._mark_scene_dirty(self)
        self._multi_status('ready')

    def _multi_collision_objects(self):
        """Keep every other detected wall in MoveIt's collision world."""
        planes = self._multi_catalog.get('planes', [])
        out, ids = [], set()
        if planes:
            tf = self._lookup_transform_to_base(self._multi_catalog['frame_id'], timeout_s=.1)
            if tf is None:
                return None
            q, t = tf.transform.rotation, tf.transform.translation
            quat, trans = [q.x,q.y,q.z,q.w], np.array([t.x,t.y,t.z])
        for plane in planes:
            if plane['id'] == self._multi_active_id:
                continue
            corners = quat_apply(quat, np.asarray(plane['corners'])) + trans
            n = quat_apply(quat, np.asarray(plane['normal']))
            u = corners[1]-corners[0]; width = np.linalg.norm(u); u /= max(width, 1e-9)
            v = np.cross(n,u); height = np.ptp(corners @ v)
            if min(width,height) < .005:
                continue
            center = corners.mean(axis=0) - .01*n
            co = CollisionObject(); co.id = 'sketch_plane_'+plane['id']; ids.add(co.id)
            co.header.frame_id = 'link0'; co.operation = CollisionObject.ADD
            prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(width+.02),float(height+.02),.02]
            pose=Pose(); pose.position.x,pose.position.y,pose.position.z=map(float,center)
            rot=quat_from_matrix(np.column_stack((u,v,n)))
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,rot)
            co.primitives=[prim]; co.primitive_poses=[pose]; out.append(co)
        for stale in self._multi_scene_ids-ids:
            co=CollisionObject(); co.id=stale; co.header.frame_id='link0'; co.operation=CollisionObject.REMOVE; out.append(co)
        self._multi_scene_pending_ids = (int(self._scene_revision), ids)
        return out
