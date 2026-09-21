"""D405 camera-aware view candidates and sequential face ordering.

This mixin does not dispatch robot motion. Face ordering uses collision-aware
IK from the same fresh measured joint state. The executor subsequently checks
complete paths against the confirmed scene before moving to the chosen face.
"""
import copy
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from sketch_control.rotation_utils import quat_apply
from sketch_control.d405_view_geometry import measurement_samples, camera_view, unit, support_mask


class D405ScanSelectionMixin:
    def _d405_mount_transform(self):
        transform = self.tf_buffer.lookup_transform(
            'tcp', 'd405_color_optical_frame', rclpy.time.Time(), timeout=Duration(seconds=.2))
        t,q = transform.transform.translation, transform.transform.rotation
        translation = np.array([t.x,t.y,t.z],float)
        rotation = np.array([q.x,q.y,q.z,q.w],float)
        if not np.isfinite(translation).all() or not np.isfinite(rotation).all() or np.linalg.norm(rotation) < 1e-6:
            raise ValueError('invalid D405 optical mount TF')
        return translation, rotation/np.linalg.norm(rotation)

    def _d405_support_in_base(self, plane):
        frame = self._multi_catalog['frame_id']
        transform = self._lookup_transform_to_base(frame, timeout_s=.2)
        if transform is None:
            raise ValueError('plane TF unavailable')
        t,q = transform.transform.translation, transform.transform.rotation
        rotation = [q.x,q.y,q.z,q.w]
        translation = np.array([t.x,t.y,t.z],float)
        polygon = quat_apply(rotation,np.asarray(plane.get('support_polygon',plane['corners']),float))+translation
        normal = unit(quat_apply(rotation,np.asarray(plane['normal'],float)))
        return polygon, normal

    def _d405_face_candidates(self, plane, tcp_pose, mount, *, samples=3):
        polygon,normal = self._d405_support_in_base(plane)
        camera = tcp_pose[0]+quat_apply(tcp_pose[1],mount[0])
        points = measurement_samples(camera,polygon,normal,max_samples=samples)
        candidates = []
        for sample_index,point in enumerate(points):
            for flipped in (False,True):
                xyz,q = camera_view(point,normal,tcp_pose,mount,flipped=flipped)
                pose = Pose()
                pose.position.x,pose.position.y,pose.position.z = map(float,xyz)
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = map(float,q)
                candidates.append(dict(plane_id=plane['id'],sample=point,pose=pose,
                                       sample_index=sample_index,flipped=flipped))
        return candidates

    def _d405_capture_sample_in_base(self):
        """Intersect the measured calibrated optical ray with the selected plane."""
        try:
            current = self._current_tcp_pose_np()
            mount = self._d405_mount_transform()
            frozen = self._d405_scan_mount
            if current is None or np.linalg.norm(mount[0]-frozen[0]) > .001 or abs(np.dot(mount[1],frozen[1])) < np.cos(np.deg2rad(.5)/2):
                raise ValueError('camera mount changed or TCP unavailable')
            polygon,normal = self._d405_support_in_base(self._multi_current)
            camera = current[0]+quat_apply(current[1],mount[0])
            axis = unit(quat_apply(current[1],quat_apply(mount[1],[0.,0.,1.])))
            distance = (camera-polygon[0])@normal
            if abs(distance-.32) > .015 or axis@(-normal) < np.cos(np.deg2rad(3.)):
                raise ValueError('measured camera standoff/aim outside tolerance')
            sample = camera-axis*(distance/(axis@normal))
            if not support_mask([sample],polygon,normal,margin=.075)[0]:
                raise ValueError('measured view outside safe plane support')
            return sample
        except Exception as exc:
            self.get_logger().error(f'[D405 VIEW] capture rejected: {exc}')
            return None

    def _cancel_multi_order(self):
        timer = getattr(self,'_multi_order_timer',None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self._multi_order_timer = None
        self._multi_order_context = None

    def _begin_multi_order(self):
        self._cancel_multi_order()
        context = dict(revision=int(self._scene_revision), started=time.monotonic(),
                       phase='scene', results={}, candidates=[], seed=None)
        self._multi_order_context = context
        self._multi_status('ordering')
        self.publish_scene_periodic()
        self._multi_order_timer = self.create_timer(.05,lambda:self._multi_order_tick(context))

    def _multi_order_tick(self, context):
        if context is not getattr(self,'_multi_order_context',None):
            return
        if self._motion_abort_requested or context['revision'] != int(self._scene_revision):
            self._multi_cancel('ORDER_ABORT_OR_SCENE_CHANGED')
            return
        if time.monotonic()-context['started'] > max(15.,len(context['candidates'])*.5+5.):
            self._multi_cancel('ORDER_TIMEOUT')
            return
        if context['phase'] != 'scene':
            return
        if not self._d405_scene_revision_confirmed(context['revision']):
            return
        try:
            age = time.monotonic()-self.current_joint_state_time
            if not 0 <= age <= .2 or not self._joint_state_within_limits(self.current_joint_state,'D405 face order'):
                raise ValueError('stale joint state')
            tcp = self._current_tcp_pose_np()
            if tcp is None:
                raise ValueError('TCP TF unavailable')
            mount = self._d405_mount_transform()
            context['seed'] = copy.deepcopy(self.current_joint_state)
            for plane in self._multi_queue:
                # Compare interior alternatives too: the closest point may be unreachable.
                context['candidates'].extend(self._d405_face_candidates(plane,tcp,mount,samples=3))
            if not self.ik_client.wait_for_service(timeout_sec=.2):
                raise ValueError('IK unavailable')
        except Exception as exc:
            self.get_logger().error(f'[D405 ORDER] {exc}')
            self._multi_cancel('ORDER_GEOMETRY_OR_STATE_UNAVAILABLE')
            return
        context['phase'] = 'ik'
        for index,candidate in enumerate(context['candidates']):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'manipulator'
            request.ik_request.ik_link_name = 'tcp'
            request.ik_request.robot_state.joint_state = copy.deepcopy(context['seed'])
            request.ik_request.avoid_collisions = True
            request.ik_request.pose_stamped.header.frame_id = 'link0'
            request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            request.ik_request.pose_stamped.pose = copy.deepcopy(candidate['pose'])
            request.ik_request.timeout = Duration(seconds=.5).to_msg()
            try:
                future = self.ik_client.call_async(request)
                future.add_done_callback(lambda done,c=context,i=index:self._multi_order_ik_done(done,c,i))
            except Exception:
                self._multi_cancel('ORDER_IK_REQUEST_FAILED')
                return

    def _multi_order_ik_done(self, future, context, index):
        if context is not getattr(self,'_multi_order_context',None):
            return
        if self._motion_abort_requested or not self._d405_scene_revision_confirmed(context['revision']):
            self._multi_cancel('ORDER_ABORT_OR_SCENE_CHANGED')
            return
        try:
            response = future.result()
            code = int(response.error_code.val)
            if code not in (1,-31):
                raise ValueError(f'IK error {code}')
            metrics = None
            if code == 1:
                metrics,reason = self._stage1_candidate_joint_metrics(response.solution.joint_state,seed_state=context['seed'])
                if metrics is None:
                    raise ValueError(reason)
            context['results'][index] = metrics
        except Exception as exc:
            self.get_logger().error(f'[D405 ORDER] {exc}')
            self._multi_cancel('ORDER_IK_FAILED')
            return
        if len(context['results']) != len(context['candidates']):
            return
        candidates = [(metrics['max_delta'],metrics['l2_delta'],metrics['sum_delta'],index)
                      for index,metrics in context['results'].items() if metrics is not None]
        if not candidates:
            self._multi_cancel('ORDER_NO_REACHABLE_FACE')
            return
        # Never use an order computed from a robot state that has since moved.
        seed = dict(zip(context['seed'].name,context['seed'].position))
        state = self.current_joint_state
        if state is None:
            self._multi_cancel('ORDER_MEASURED_STATE_LOST')
            return
        actual = dict(zip(state.name,state.position))
        if not 0 <= time.monotonic()-self.current_joint_state_time <= .2 or any(
                name not in actual or not np.isfinite(actual[name]) or abs(actual[name]-value) > .01
                for name,value in seed.items()):
            self._multi_cancel('ORDER_MEASURED_SEED_CHANGED')
            return
        selected = context['candidates'][min(candidates)[-1]]['plane_id']
        self._cancel_multi_order()
        self.get_logger().info(f'[D405 ORDER] next face={selected}; smallest collision-free IK joint change')
        self._multi_start_next(ordered_id=selected)
