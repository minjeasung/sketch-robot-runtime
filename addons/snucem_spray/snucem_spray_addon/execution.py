"""Opt-in guarded adapter around the existing Sketch automatic executor."""
import json
import math
import signal
import time

from .execution_guard import ExecutionGuard


def executor_class():
    from rclpy.action.graph import get_action_client_names_and_types_by_node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from sketch_control.moveit_executor import MoveItExecutor

    class ExternalSprayExecutor(MoveItExecutor):
        def __init__(self):
            self.external_guard = ExecutionGuard()
            self.external_model = self.external_source = ''
            self.external_model_at = self.external_source_at = float('-inf')
            super().__init__()
            if (not self._external_stack_model or self.process_mode != 'spray'
                    or self.painting_force_enabled or self.execution_backend != 'follow_joint_trajectory'):
                raise ValueError('external executor requires Spray, an external model and position trajectory control')
            if self.spray_eoat_profile != str(self.get_parameter('external_stack_model').value):
                raise ValueError('generation/execution must use the same external model profile')
            target = dict(next(obj for obj in self.cfg['objects'] if obj['name'] == self.active_target_name))
            target['name'] = 'snucem_sketch_target'
            self.cfg = dict(active_target=target['name'], objects=[target])
            self.active_target_name = target['name']
            self._enabled_ids = {target['name']}
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/snucem_sketch/model', self.on_external_model, latched)
            self.create_subscription(String, '/snucem_sketch/source', self.on_external_source, latched)

        def create_subscription(self, msg_type, topic, *args, **kwargs):
            if (topic.startswith('/debug_') or topic in {
                    '/robot_pose_preset', '/go_ready_pose', '/go_calibration_pose',
                    '/perception/obstacles', '/perception/planes', '/perception/plane_labels'}):
                return None
            return super().create_subscription(msg_type, topic, *args, **kwargs)

        def on_external_model(self, msg):
            try:
                self.external_model = str(json.loads(msg.data).get('fingerprint', ''))
            except (ValueError, TypeError, AttributeError):
                self.external_model = ''
            self.external_model_at = time.monotonic()

        def on_external_source(self, msg):
            try:
                self.external_source = str(json.loads(msg.data).get('revision', ''))
            except (ValueError, TypeError, AttributeError):
                self.external_source = ''
            self.external_source_at = time.monotonic()

        def _external_motion_blockers(self):
            now = time.monotonic()
            model = self.external_model if now-self.external_model_at <= .75 else ''
            source = self.external_source if now-self.external_source_at <= .75 else ''
            controllers = self._controller_states if now-getattr(self, '_controller_states_time', 0) <= 1.0 else {}
            competitors = []
            try:
                nodes = self.get_node_names_and_namespaces()
                own = (self.get_name(), self.get_namespace())
                if nodes.count(own) != 1:
                    competitors.append('duplicate add-on executor')
                for name, namespace in nodes:
                    if (name, namespace) == own or name == 'move_group':
                        continue
                    if name in ('rb20_spray_executor', 'rb10_shared_autonomy_pose_executor'):
                        competitors.append(name)
                        continue
                    clients = get_action_client_names_and_types_by_node(self, name, namespace)
                    if any(action == self.follow_joint_trajectory_action for action, _ in clients):
                        competitors.append(namespace+'/'+name)
            except Exception:
                competitors.append('ROS ownership graph unavailable')
            self.external_guard.update(model=model, source=source, controllers=controllers,
                                       competitors=competitors, now=now)
            expected = getattr(self, '_external_stack_model', None)
            selected = getattr(self, '_multi_catalog', {}).get('source_revision')
            return self.external_guard.blockers(now,
                model=expected['fingerprint'] if expected else None,
                source=selected or None)

        def _spray_motion_blockers(self):
            # Use real controller-manager state instead of manufacturing a
            # compliance_active message that the external stack does not publish.
            return tuple(self._external_motion_blockers()) + tuple(self._spray_io_blockers())

        def _spray_tick(self):
            reasons = self._external_motion_blockers()
            if reasons and (self.executing or self._active_trajectory_goal_token is not None):
                self._spray_off()
                self._request_motion_abort(','.join(reasons))
                return
            super()._spray_tick()

        def on_execute(self, msg):
            if not msg.data:
                return super().on_execute(msg)
            reasons = self._external_motion_blockers()
            if reasons:
                self._publish_execution_status('BLOCKED', ','.join(reasons))
                return
            selected = self._multi_catalog.get('source_revision', '')
            if not selected:
                self._publish_execution_status('BLOCKED', 'EXTERNAL_SELECTION_REQUIRED')
                return
            self.external_guard.arm(self._external_stack_model['fingerprint'], selected, now=time.monotonic())
            super().on_execute(msg)

        def on_reset_execution_abort(self, request, response):
            # This is an explicit user reset. Keep the normal stationary,
            # pending-action and collision checks. It never resumes an old path.
            was_latched = self.external_guard.latched
            self.external_guard.reset()
            result = super().on_reset_execution_abort(request, response)
            if result.success:
                self._invalidate_zed_target('EXTERNAL_MANUAL_RESET_RESELECT_REQUIRED')
            else:
                self.external_guard.latched = was_latched
            return result

        def publish_scene_periodic(self):
            if getattr(self, 'dynamic_surface_point', None) is None:
                return
            super().publish_scene_periodic()

        def _execute_trajectory_follow_joint(self, *args, **kwargs):
            reasons = self._external_motion_blockers()
            if reasons:
                self._last_dispatch_inhibit_reason = ','.join(reasons)
                self._spray_off()
                return False
            return super()._execute_trajectory_follow_joint(*args, **kwargs)

        def remove_owned_scene(self):
            from moveit_msgs.msg import PlanningScene, CollisionObject
            from moveit_msgs.srv import ApplyPlanningScene
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            ids = set(self._multi_scene_ids) | {self.active_target_name}
            pending = getattr(self, '_multi_scene_pending_ids', None)
            if pending is not None:
                ids.update(pending[1])
            for ident in sorted(ids):
                if ident != 'snucem_sketch_target' and not ident.startswith('sketch_plane_'):
                    continue
                obj = CollisionObject(id=ident, operation=CollisionObject.REMOVE)
                obj.header.frame_id = 'link0'
                scene.world.collision_objects.append(obj)
            self.scene_pub.publish(scene)
            if self.apply_scene_client.service_is_ready():
                return self.apply_scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
            return None

        def _trajectory_within_joint_limits(self, jt, label):
            if not super()._trajectory_within_joint_limits(jt, label):
                return False
            limits = self._external_stack_model['joint_limits']
            for point in jt.points:
                for values, field in ((point.velocities, 'velocity'), (point.accelerations, 'acceleration')):
                    if len(values) != len(jt.joint_names):
                        return False
                    if any(not math.isfinite(value) or abs(value) > limits[name][field]+1e-6
                           for name, value in zip(jt.joint_names, values)):
                        return False
            return True

    return ExternalSprayExecutor


def run(config, profile_path):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    args = ['--ros-args', '-r', '__node:=snucem_sketch_executor']
    params = dict(external_stack_model=profile_path, spray_eoat_profile=profile_path,
                  process_mode='spray', model_id=config.model_id, spray_tool_axis='+z',
                  execution_backend='follow_joint_trajectory',
                  follow_joint_trajectory_action='/joint_trajectory_controller/follow_joint_trajectory',
                  dry_run=config.profile == 'dry_run', real_painting_enabled=True,
                  painting_force_enabled=False, spray_motion_test=config.profile == 'motion_test')
    for key, value in params.items():
        args += ['-p', key+':='+str(value).lower() if isinstance(value, bool) else key+':='+str(value)]
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def stop_requested(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)
    node = executor_class()()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Preserve the context long enough to receive cancellation results.
        # Timers may only finish the aborted run while the latch is set.
        node._spray_off()
        node._request_motion_abort('ADDON_SHUTDOWN')
        deadline = time.monotonic()+3
        while rclpy.ok() and node._active_trajectory_goal_token is not None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        for timer in list(node.timers):
            node.destroy_timer(timer)
        future = node.remove_owned_scene()
        deadline = time.monotonic()+1
        while rclpy.ok() and future is not None and not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
