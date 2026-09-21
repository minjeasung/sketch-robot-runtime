#!/usr/bin/env python3
# Copyright (c) 2024 Rainbow Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Auto-reset helper for the admittance controller.

Why this exists
---------------
With stiffness K = 0 the admittance controller has no spring restoration,
so once an F/T input has been integrated into its internal joint_pos
(e.g. during a manual human push), that offset never decays. When MoveIt
later runs plan&execute, the controller chain
    MoveIt -> JTC -> admittance -> hardware
adds this stale accumulated offset on top of the trajectory, and the
cobot lurches at execute start to "reflect" the previous push.

What this node does
-------------------
Watches MoveIt's /move_action and, on the rising edge of an EXECUTING
goal (planning has just started, JTC is not yet running a trajectory),
calls /controller_manager/switch_controller to cycle the pair
    [joint_trajectory_controller, admittance_controller]
through deactivate -> activate. The deactivate forces both controllers'
on_activate to run again the next instant, which:
  - resets admittance_controller's internal joint_pos / joint_vel to 0,
  - refreshes JTC's "current" output from joint_states.
By the time MoveIt finishes planning and dispatches the trajectory to
JTC, the chain is fresh, so no accumulated offset is left to add on top.

Safety
------
1. Rising-edge only: the reset fires when ANY move_action goal first
   transitions to EXECUTING. Goals that stay EXECUTING for the whole
   plan+execute window do not re-trigger.
2. JTC state guard: if JTC already reports its own action as EXECUTING
   (a trajectory is currently being played back), the reset is SKIPPED
   so an in-progress trajectory cannot be cancelled mid-flight.
3. Throttled: at most one reset per MIN_RESET_INTERVAL_S, even across
   back-to-back plan&execute commands.

Manual usage
------------
Same cycle can be triggered on demand:
    ros2 service call /rbpodo_admittance_helper/reset_admittance \
        std_srvs/srv/Trigger
"""

import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_srvs.srv import Trigger


# Action status topics are published with this QoS by rclcpp_action.
ACTION_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class AdmittanceResetNode(Node):

    # Jazzy's controller_manager registers this as ~/switch_controller
    # (singular). The plural form occasionally appears in discovery dumps
    # from a stale ros2 daemon cache, but no service with that name is
    # actually advertised, so creating a client against the plural path
    # makes service_is_ready() return False forever and call_async time out.
    SWITCH_SERVICE = "/controller_manager/switch_controller"
    MOVE_ACTION_STATUS = "/move_action/_action/status"
    JTC_ACTION_STATUS = (
        "/joint_trajectory_controller/follow_joint_trajectory/_action/status"
    )
    ADMITTANCE_CONTROLLER = "admittance_controller"
    JTC_CONTROLLER = "joint_trajectory_controller"
    MIN_RESET_INTERVAL_S = 1.0

    def __init__(self):
        super().__init__("rbpodo_admittance_helper")
        # Put the switch_controllers client (and the service callbacks that
        # may invoke it) in their own callback group, isolated from the
        # subscription callbacks. With everything in a single ReentrantCallback
        # Group on Jazzy's MultiThreadedExecutor we hit a discovery starvation
        # where the client's graph view never sees the existing service, so
        # service_is_ready() and call_async() both think it doesn't exist.
        self._client_cb = ReentrantCallbackGroup()
        self._sub_cb = ReentrantCallbackGroup()
        self._reset_lock = threading.Lock()
        self._move_was_active = False
        self._jtc_active = False
        self._last_reset_time = None

        self._switch_client = self.create_client(
            SwitchController,
            self.SWITCH_SERVICE,
            callback_group=self._client_cb,
        )

        self._move_status_sub = self.create_subscription(
            GoalStatusArray,
            self.MOVE_ACTION_STATUS,
            self._on_move_action_status,
            ACTION_STATUS_QOS,
            callback_group=self._sub_cb,
        )

        self._jtc_status_sub = self.create_subscription(
            GoalStatusArray,
            self.JTC_ACTION_STATUS,
            self._on_jtc_action_status,
            ACTION_STATUS_QOS,
            callback_group=self._sub_cb,
        )

        self._reset_srv = self.create_service(
            Trigger,
            "~/reset_admittance",
            self._on_reset_request,
            callback_group=self._client_cb,
        )

        self.get_logger().info(
            "Admittance reset helper ready. Watching %s; manual trigger "
            "via /%s/reset_admittance."
            % (self.MOVE_ACTION_STATUS, self.get_name())
        )

    # ---- status subscribers ------------------------------------------------

    def _on_move_action_status(self, msg: GoalStatusArray) -> None:
        has_active = any(
            s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list
        )
        rising_edge = has_active and not self._move_was_active
        self._move_was_active = has_active

        if not rising_edge:
            return

        # Safety guard: JTC must NOT already be executing a trajectory.
        # If it is, our deactivate would cancel it mid-flight.
        if self._jtc_active:
            self.get_logger().warn(
                "/move_action active but JTC is already executing - "
                "skipping reset to avoid cancelling the trajectory"
            )
            return

        # Throttle to one reset per MIN_RESET_INTERVAL_S.
        if not self._enough_time_passed():
            self.get_logger().info(
                "skipping reset (throttled, last reset too recent)"
            )
            return

        self.get_logger().info(
            "/move_action rising edge - resetting admittance during the "
            "planning window"
        )
        self._do_reset(reason="move_action rising edge")

    def _on_jtc_action_status(self, msg: GoalStatusArray) -> None:
        self._jtc_active = any(
            s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list
        )

    def _on_reset_request(self, request, response):
        # Manual trigger - still respects the JTC guard, but bypasses
        # the throttle so the user can force a reset on demand.
        if self._jtc_active:
            response.success = False
            response.message = (
                "JTC is currently executing a trajectory - refusing to "
                "reset (would cancel it). Wait for the trajectory to "
                "complete and try again."
            )
            self.get_logger().warn(response.message)
            return response

        ok, msg = self._do_reset(reason="manual service call")
        response.success = ok
        response.message = msg
        return response

    # ---- core --------------------------------------------------------------

    def _enough_time_passed(self) -> bool:
        if self._last_reset_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self._last_reset_time
        ).nanoseconds / 1e9
        return elapsed >= self.MIN_RESET_INTERVAL_S

    def _do_reset(self, reason: str):
        with self._reset_lock:
            # Diagnostic: dump what services and node graph state our process
            # actually sees right now. This is the smoking gun when
            # service_is_ready() lies about availability - we can see whether
            # the service is even in our node's graph view.
            try:
                all_services = self.get_service_names_and_types()
                cm_services = sorted(
                    name for (name, _types) in all_services
                    if "controller_manager" in name
                )
                node_names = sorted(self.get_node_names())
                self.get_logger().info(
                    "Discovery diagnostic before reset:\n"
                    "  node sees %d controller_manager services: %s\n"
                    "  node sees %d total nodes: first 10 = %s\n"
                    "  switch_client.service_is_ready() = %s"
                    % (
                        len(cm_services),
                        cm_services,
                        len(node_names),
                        node_names[:10],
                        self._switch_client.service_is_ready(),
                    )
                )
            except Exception as e:  # pragma: no cover - diagnostic only
                self.get_logger().warn("diagnostic dump failed: %s" % e)

            # Skip the service_is_ready() pre-check entirely. On Jazzy we have
            # seen the Python client's graph view stay stale even when the
            # service is reachable in the underlying DDS layer; in that case
            # service_is_ready() returns False forever even though call_async
            # would succeed. So we just attempt the call directly and let the
            # call_deadline below catch genuine unreachability.
            req = SwitchController.Request()
            # Order matters: deactivate_controllers runs first, then
            # activate_controllers. Listing the chained pair in both
            # forces both controllers' on_activate to run, which is
            # what re-initialises admittance's internal state to zero
            # and refreshes JTC's "current" output from joint_states.
            req.deactivate_controllers = [
                self.JTC_CONTROLLER,
                self.ADMITTANCE_CONTROLLER,
            ]
            req.activate_controllers = [
                self.ADMITTANCE_CONTROLLER,
                self.JTC_CONTROLLER,
            ]
            req.strictness = SwitchController.Request.STRICT

            # Async call + future polling. This is callback-safe under
            # MultiThreadedExecutor: while we sleep in the poll loop,
            # another executor thread receives the service response and
            # marks the future done.
            future = self._switch_client.call_async(req)
            call_deadline = time.monotonic() + 5.0
            while not future.done():
                if time.monotonic() >= call_deadline:
                    msg = "switch_controllers call timed out"
                    self.get_logger().error(msg)
                    return False, msg
                time.sleep(0.05)

            try:
                result = future.result()
            except Exception as e:
                msg = "switch_controllers call raised: %s" % e
                self.get_logger().error(msg)
                return False, msg

            if result is None:
                msg = "switch_controllers returned no result"
                self.get_logger().error(msg)
                return False, msg

            if not result.ok:
                msg = "switch_controllers returned ok=False"
                self.get_logger().error(msg)
                return False, msg

            self._last_reset_time = self.get_clock().now()
            self.get_logger().info(
                "admittance reset OK (trigger: %s)" % reason
            )
            return True, "admittance reset (%s)" % reason


def main():
    rclpy.init()
    node = AdmittanceResetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
