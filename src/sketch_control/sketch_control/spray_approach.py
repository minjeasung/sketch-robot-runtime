"""Check complete circular-cone roll alternatives before dispatching approach."""
import copy
import math
from rclpy.clock import Clock, ClockType
from moveit_msgs.srv import GetCartesianPath


class SprayApproachMixin:
    def _spray_ik_seed_candidates(self, candidates, measured):
        # Search a second shoulder/elbow basin; these are IK seeds only, never
        # robot commands. Inactive arm/head positions always stay measured.
        names = list(self.arm_joint_names)
        if len(names) < 2:
            return list(candidates)
        joint = names[1]
        positions = dict(zip(measured.name, measured.position))
        limits = self._robot_joint_limits.get(joint)
        alternate = -float(positions[joint])
        if (limits is None or not limits[0] <= alternate <= limits[1]
                or abs(alternate - positions[joint]) < 1e-6):
            return list(candidates)
        expanded = []
        for candidate in candidates:
            expanded.append(candidate)
            other = copy.deepcopy(candidate)
            other['name'] += '_alternate_ik'
            other['ik_seed_positions'] = {joint: alternate}
            expanded.append(other)
        return expanded

    def _spray_preflight_valid(self, context):
        return (not context['done']
                and context['token'] is self._stage1_attempt_token
                and context['generation'] == self._stage1_ik_candidate_generation
                and not self._motion_abort_requested)

    def _preflight_spray_orientation_paths(self, records, generation, token):
        context = dict(records=records, index=0, accepted=[], done=False,
                       token=token, generation=generation, scene=self._scene_revision)
        self._cancel_stage1_ik_candidate_timer()
        def timeout():
            if self._spray_preflight_valid(context):
                context['done'] = True
                self._cancel_stage1_ik_candidate_timer()
                self._fail_stage1_before_motion('SPRAY_FULL_PATH_PREFLIGHT_TIMEOUT')
        self._stage1_ik_candidate_timer = self.create_timer(
            30., timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._spray_preflight_next(context)

    def _spray_preflight_next(self, context):
        if not self._spray_preflight_valid(context):
            return
        if (not self.scene_confirmed or self._scene_revision != context['scene']
                or self._scene_confirmed_revision != context['scene']):
            context['done'] = True
            self._cancel_stage1_ik_candidate_timer()
            self._fail_stage1_before_motion('SPRAY_PREFLIGHT_SCENE_CHANGED')
            return
        if context['index'] == len(context['records']):
            context['done'] = True
            self._cancel_stage1_ik_candidate_timer()
            self._stage1_orientation_ranked = context['accepted']
            if not context['accepted']:
                self._fail_stage1_before_motion('NO_COLLISION_FREE_COMPLETE_SPRAY_PATH')
            else:
                self._activate_stage1_orientation_rank(0)
            return
        if not self.cartesian_client.service_is_ready():
            context['done'] = True
            self._cancel_stage1_ik_candidate_timer()
            self._fail_stage1_before_motion('SPRAY_PREFLIGHT_SERVICE_UNAVAILABLE')
            return
        record = context['records'][context['index']]
        candidate = record['candidate']
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = self.planning_group
        request.link_name = self.ee_link
        request.start_state.joint_state = copy.deepcopy(self._stage1_ik_seed_state)
        target = dict(zip(record['joint_state'].name, record['joint_state'].position))
        measured = request.start_state.joint_state
        measured.position = [target.get(name, position)
                             for name, position in zip(measured.name, measured.position)]
        request.start_state.is_diff = False
        request.waypoints = [copy.deepcopy(pose) for _, pose in sorted(candidate['row_tcp_poses'].items())]
        request.waypoints.append(copy.deepcopy(candidate['retreat_tcp_pose']))
        request.max_step = .0025
        request.jump_threshold = 2.
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = .1
        request.max_acceleration_scaling_factor = .1
        try:
            future = self.cartesian_client.call_async(request)
        except Exception as exc:
            context['done'] = True
            self._cancel_stage1_ik_candidate_timer()
            self._fail_stage1_before_motion('SPRAY_PREFLIGHT_REQUEST_FAILED:' + str(exc))
            return
        future.add_done_callback(lambda done: self._spray_preflight_done(done, context, record))

    def _spray_preflight_done(self, future, context, record):
        if not self._spray_preflight_valid(context):
            return
        try:
            response = future.result()
            fraction = float(response.fraction)
            complete = (response.error_code.val == 1 and math.isfinite(fraction)
                        and .999999 <= fraction <= 1.
                        and bool(response.solution.joint_trajectory.points))
        except Exception as exc:
            context['done'] = True
            self._cancel_stage1_ik_candidate_timer()
            self._fail_stage1_before_motion('SPRAY_PREFLIGHT_RESPONSE_FAILED:' + str(exc))
            return
        self.get_logger().info('[SPRAY FULL PATH] %s: fraction=%.6f accepted=%s' %
                               (record['candidate']['name'], fraction, complete))
        if complete:
            context['accepted'].append(record)
        context['index'] += 1
        self._spray_preflight_next(context)
