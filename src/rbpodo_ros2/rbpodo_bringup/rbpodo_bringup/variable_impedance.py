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
Variable impedance helper for the admittance controller.

Why
---
With K = 0 admittance accumulates joint_pos from any residual F/T noise
that survives the deadband, and on release the velocity decays through
joint_damping but with the original (low) damping value the residual
motion is still visible. We want:

  * High mass + high joint_damping when the user is NOT pushing -> the
    arm aggressively resists drift, effectively "holds in place".
  * Low (= yaml-loaded) mass + joint_damping when the user IS pushing
    -> the arm feels free for the human to move.

This node:
  1. At startup, queries admittance_controller for its current mass and
     joint_damping. Those are cached as the LOW values (the user's yaml).
  2. Computes HIGH = LOW * multiplier (defaults to 10x, configurable).
  3. Subscribes to the broadcaster's wrench topic (already deadband-
     filtered by RBPodoHardwareInterface) and computes |F|.
  4. With hysteresis on |F|, decides target state LOW or HIGH.
  5. On state changes ONLY, calls /admittance_controller/set_parameters
     to swap mass and joint_damping.

The admittance controller's integration of joint_vel / joint_pos is
continuous across parameter changes (the change only affects the next
cycle's joint_acc = F/M - joint_damping*joint_vel), so there is no jolt.

Tunables (ROS parameters, set via --ros-args)
---------------------------------------------
  enter_low_force  : |F| above this -> switch to LOW impedance (default 1.5 N,
                     aligned with the hardware-side kFtCollabDeadband so the
                     switch fires exactly when the wrench crosses the deadband)
  exit_low_force   : |F| below this -> switch to HIGH impedance (default 0.5 N)
  high_multiplier  : HIGH = LOW * this (default 10.0)
  start_state      : initial state before any wrench seen: 'HIGH' or 'LOW' (default 'HIGH')
"""

import math
import threading
import time

from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class VariableImpedanceNode(Node):

    WRENCH_TOPIC = "/force_torque_sensor_broadcaster/wrench"
    SET_PARAM_SERVICE = "/admittance_controller/set_parameters"
    GET_PARAM_SERVICE = "/admittance_controller/get_parameters"

    MASS_PARAM = "admittance.mass"
    JOINT_DAMPING_PARAM = "admittance.joint_damping"

    def __init__(self):
        super().__init__("rbpodo_variable_impedance")
        self._cb = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        self.declare_parameter("enter_low_force", 1.5)
        self.declare_parameter("exit_low_force", 0.5)
        self.declare_parameter("high_multiplier", 10.0)
        self.declare_parameter("start_state", "HIGH")
        # LOW -> HIGH is ramped linearly over this many seconds so the user
        # does not feel a sudden mass/damping bump on release.
        self.declare_parameter("ramp_to_high_duration_s", 2.0)
        self.declare_parameter("ramp_update_period_s", 0.1)
        # HIGH -> LOW is stepped down in discrete jumps. With the defaults
        # below the user feels 5 evenly-spaced drops every 0.1 s over a
        # total of 0.5 s instead of an abrupt cliff, but still gets a
        # free-push feel quickly. Step count is derived as
        # round(duration / period).
        self.declare_parameter("step_to_low_duration_s", 0.5)
        self.declare_parameter("step_to_low_period_s", 0.1)

        self._enter_low = float(self.get_parameter("enter_low_force").value)
        self._exit_low = float(self.get_parameter("exit_low_force").value)
        self._high_multiplier = float(self.get_parameter("high_multiplier").value)
        start_state = str(self.get_parameter("start_state").value).upper()
        if start_state not in ("HIGH", "LOW"):
            start_state = "HIGH"
        self._ramp_duration = max(
            0.0, float(self.get_parameter("ramp_to_high_duration_s").value)
        )
        self._ramp_period = max(
            0.01, float(self.get_parameter("ramp_update_period_s").value)
        )
        self._step_duration = max(
            0.0, float(self.get_parameter("step_to_low_duration_s").value)
        )
        self._step_period = max(
            0.01, float(self.get_parameter("step_to_low_period_s").value)
        )
        # Step count derived from duration / period (rounded). With T = 0.5 s
        # and dt = 0.1 s -> 5 evenly-spaced applies, the last one landing on
        # LOW. A degenerate (count <= 1 or duration <= 0) config snaps
        # straight to LOW in _start_stepdown_unlocked.
        self._step_count = max(
            1, int(round(self._step_duration / self._step_period))
            if self._step_duration > 0 else 1
        )

        if self._exit_low >= self._enter_low:
            self.get_logger().warn(
                "exit_low_force (%.2f) should be < enter_low_force (%.2f); "
                "hysteresis is degenerate." % (self._exit_low, self._enter_low)
            )

        # State machine. States:
        #   "LOW"      - at yaml values (free push)
        #   "RAMPING"  - linearly interpolating LOW -> HIGH (on release)
        #   "HIGH"     - at multiplied values (idle hold)
        # Note: a "STEPPING" state for a discrete HIGH -> LOW step-down
        # exists in the helper section below but is currently disabled
        # because it hurt UX (felt sluggish on push). HIGH -> LOW now
        # snaps directly to LOW.
        self._state = None  # set after first successful param read + first wrench
        self._pending_state = start_state
        self._mass_low = None
        self._joint_damping_low = None
        self._mass_high = None
        self._joint_damping_high = None
        self._initialised = False
        # Ramp bookkeeping (only used while _state == "RAMPING")
        self._ramp_start_time = None
        self._ramp_timer = None
        # Step bookkeeping (only used while _state == "STEPPING")
        self._step_index = 0
        self._step_timer = None

        self._set_param_client = self.create_client(
            SetParameters, self.SET_PARAM_SERVICE, callback_group=self._cb
        )
        self._get_param_client = self.create_client(
            GetParameters, self.GET_PARAM_SERVICE, callback_group=self._cb
        )

        self.create_subscription(
            WrenchStamped,
            self.WRENCH_TOPIC,
            self._on_wrench,
            10,
            callback_group=self._cb,
        )

        # One-shot timer to do the LOW-values discovery once the param
        # service is up.
        self._init_timer = self.create_timer(
            0.5, self._init_callback, callback_group=self._cb
        )

        self.get_logger().info(
            "Variable impedance helper starting. "
            "enter_low=%.2f, exit_low=%.2f, high_multiplier=%.1f, start_state=%s"
            % (self._enter_low, self._exit_low, self._high_multiplier, start_state)
        )

    # ---- initialisation ----------------------------------------------------

    def _init_callback(self):
        if self._initialised:
            return
        if not self._get_param_client.service_is_ready():
            return
        # Read current mass + joint_damping; these become the LOW (yaml) values.
        req = GetParameters.Request()
        req.names = [self.MASS_PARAM, self.JOINT_DAMPING_PARAM]
        future = self._get_param_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "Timed out reading initial mass/joint_damping from "
                    "admittance_controller; will retry on next tick."
                )
                return
            time.sleep(0.05)

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error("get_parameters raised: %s" % e)
            return
        if result is None or len(result.values) != 2:
            self.get_logger().error(
                "get_parameters returned unexpected result: %s" % result
            )
            return

        mass_val = result.values[0]
        jd_val = result.values[1]
        if mass_val.type != ParameterType.PARAMETER_DOUBLE_ARRAY:
            self.get_logger().error("admittance.mass has unexpected type")
            return
        if jd_val.type != ParameterType.PARAMETER_DOUBLE:
            self.get_logger().error("admittance.joint_damping has unexpected type")
            return

        self._mass_low = list(mass_val.double_array_value)
        self._joint_damping_low = float(jd_val.double_value)
        self._mass_high = [m * self._high_multiplier for m in self._mass_low]
        self._joint_damping_high = self._joint_damping_low * self._high_multiplier

        self.get_logger().info(
            "Cached LOW (yaml) values: mass=%s, joint_damping=%.2f. "
            "HIGH (multiplied) values: mass=%s, joint_damping=%.2f."
            % (
                self._mass_low,
                self._joint_damping_low,
                self._mass_high,
                self._joint_damping_high,
            )
        )

        self._initialised = True
        # Apply the initial state immediately (so we start in a known mode).
        with self._state_lock:
            self._state = self._pending_state
            if self._pending_state == "HIGH":
                mass, jd = self._mass_high, self._joint_damping_high
            else:
                mass, jd = self._mass_low, self._joint_damping_low
        self._apply_values(mass, jd, self._pending_state)

        # Initialisation done -> cancel the timer.
        self._init_timer.cancel()

    # ---- wrench callback ---------------------------------------------------

    def _on_wrench(self, msg: WrenchStamped):
        if not self._initialised:
            return  # still waiting for parameter read

        f = msg.wrench.force
        mag = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

        # Decide and act under the lock so concurrent wrench callbacks and
        # ramp ticks don't race on _state / timers. (Step-down path is
        # currently disabled - see the HIGH branch below.)
        action = None  # one of: None, "ramp", "low", "high_instant"
        #                       ("stepdown" reserved for the disabled path)
        previous = None
        with self._state_lock:
            if self._state == "LOW":
                if mag <= self._exit_low:
                    # Release -> start linear ramp toward HIGH.
                    previous = self._state
                    self._state = "RAMPING"
                    action = "ramp"
            elif self._state == "RAMPING":
                if mag >= self._enter_low:
                    # User pushed mid-ramp -> cancel and snap to LOW. The
                    # current value is already partially LOW so a single
                    # jump is small; no need to step further.
                    previous = self._state
                    self._cancel_ramp_unlocked()
                    self._state = "LOW"
                    action = "low"
            elif self._state == "HIGH":
                if mag >= self._enter_low:
                    # Push -> snap directly to LOW for an immediate
                    # free-push feel. The discrete step-down path
                    # (state "STEPPING") felt sluggish in practice and
                    # is disabled below; the helpers remain in place so
                    # it can be re-enabled by restoring this branch.
                    previous = self._state
                    self._state = "LOW"
                    action = "low"
                    # previous = self._state
                    # self._state = "STEPPING"
                    # action = "stepdown"
            # elif self._state == "STEPPING":
            #     # Already stepping down; ignore both push (we're going to
            #     # LOW anyway) and release (let the step-down complete; the
            #     # subsequent LOW state will trigger a fresh ramp on the
            #     # next wrench).
            #     pass
            else:  # first wrench after init (defensive; _init_callback sets _state)
                if mag >= self._enter_low:
                    previous = self._state
                    self._state = "LOW"
                    action = "low"
                else:
                    previous = self._state
                    self._state = "HIGH"
                    action = "high_instant"

            if action == "ramp":
                self._start_ramp_unlocked()
            # elif action == "stepdown":
            #     self._start_stepdown_unlocked()

        if action is None:
            return

        self.get_logger().info(
            "|F|=%.2f N -> state %s -> %s" % (mag, previous, self._state)
        )

        if action == "low":
            self._apply_values(self._mass_low, self._joint_damping_low, "LOW")
        elif action == "high_instant":
            self._apply_values(self._mass_high, self._joint_damping_high, "HIGH")
        # action == "ramp": first tick scheduled via timer; nothing to
        # apply now. (action == "stepdown" is disabled; see HIGH branch.)

    # ---- ramp helpers (LOW -> HIGH linear interpolation) ------------------

    def _start_ramp_unlocked(self):
        """Begin LOW -> HIGH ramp. Caller must hold _state_lock."""
        # Cancel any stale timer (defensive; normally none when entering RAMPING).
        if self._ramp_timer is not None:
            self._ramp_timer.cancel()
            self._ramp_timer = None
        # If ramp duration is zero, jump straight to HIGH.
        if self._ramp_duration <= 0.0:
            self._state = "HIGH"
            # Apply outside the lock would be cleaner, but _apply_values does no
            # locking itself; keeping it inline is safe enough.
            self._ramp_start_time = None
            self.get_logger().info(
                "Ramp duration is 0; snapping LOW -> HIGH immediately."
            )
            self._apply_values(self._mass_high, self._joint_damping_high, "HIGH")
            return
        self._ramp_start_time = time.monotonic()
        self._ramp_timer = self.create_timer(
            self._ramp_period, self._ramp_tick, callback_group=self._cb
        )
        self.get_logger().info(
            "Starting LOW -> HIGH ramp over %.2fs (period %.2fs)"
            % (self._ramp_duration, self._ramp_period)
        )

    def _cancel_ramp_unlocked(self):
        """Cancel an in-flight LOW -> HIGH ramp. Caller must hold _state_lock."""
        if self._ramp_timer is not None:
            self._ramp_timer.cancel()
            self._ramp_timer = None
        self._ramp_start_time = None

    def _ramp_tick(self):
        # Snapshot ramp progress under the lock; only push params outside it
        # so set_parameters service calls don't block the lock.
        with self._state_lock:
            if self._state != "RAMPING" or self._ramp_start_time is None:
                # Cancelled by a concurrent push -> nothing to do.
                return
            elapsed = time.monotonic() - self._ramp_start_time
            alpha = min(1.0, elapsed / self._ramp_duration)
            if alpha >= 1.0:
                # Ramp complete -> transition to HIGH and tear down the timer.
                self._state = "HIGH"
                if self._ramp_timer is not None:
                    self._ramp_timer.cancel()
                    self._ramp_timer = None
                self._ramp_start_time = None
                mass = list(self._mass_high)
                jd = self._joint_damping_high
                label = "HIGH"
                done = True
            else:
                mass = [
                    lo + (hi - lo) * alpha
                    for lo, hi in zip(self._mass_low, self._mass_high)
                ]
                jd = (
                    self._joint_damping_low
                    + (self._joint_damping_high - self._joint_damping_low) * alpha
                )
                label = "RAMPING %.0f%%" % (alpha * 100.0)
                done = False

        self._apply_values(mass, jd, label)
        if done:
            self.get_logger().info("Ramp complete -> HIGH")

    # ---- step-down helpers (HIGH -> LOW discrete steps) -------------------
    # NOTE: Disabled. The step-down path is no longer triggered from
    # _wrench_cb (HIGH -> LOW snaps directly), but the helpers below are
    # kept so it can be re-enabled by restoring the STEPPING branch in
    # _wrench_cb. Related ROS parameters step_to_low_duration_s and
    # step_to_low_period_s are still declared so existing launch configs
    # don't break, but they have no effect while step-down is disabled.

    def _start_stepdown_unlocked(self):
        """Begin HIGH -> LOW discrete step-down. Caller must hold _state_lock."""
        if self._step_timer is not None:
            self._step_timer.cancel()
            self._step_timer = None
        # Degenerate config (zero duration or single step) -> snap straight to LOW.
        if self._step_duration <= 0.0 or self._step_count <= 1:
            self._state = "LOW"
            self._step_index = 0
            self.get_logger().info(
                "Step-down duration/count is trivial; snapping HIGH -> LOW immediately."
            )
            self._apply_values(self._mass_low, self._joint_damping_low, "LOW")
            return
        self._step_index = 0
        self._step_timer = self.create_timer(
            self._step_period, self._stepdown_tick, callback_group=self._cb
        )
        self.get_logger().info(
            "Starting HIGH -> LOW step-down: %d steps over %.2fs (period %.3fs)"
            % (self._step_count, self._step_duration, self._step_period)
        )

    def _cancel_stepdown_unlocked(self):
        """Cancel an in-flight HIGH -> LOW step-down. Caller must hold _state_lock."""
        if self._step_timer is not None:
            self._step_timer.cancel()
            self._step_timer = None
        self._step_index = 0

    def _stepdown_tick(self):
        # Snapshot progress under the lock; push params outside it.
        with self._state_lock:
            if self._state != "STEPPING":
                # Cancelled / concurrent transition -> nothing to do.
                return
            self._step_index += 1
            # alpha is fraction toward LOW (0.0 = HIGH, 1.0 = LOW).
            alpha = self._step_index / self._step_count
            if self._step_index >= self._step_count:
                # Final step -> LOW.
                self._state = "LOW"
                if self._step_timer is not None:
                    self._step_timer.cancel()
                    self._step_timer = None
                self._step_index = 0
                mass = list(self._mass_low)
                jd = self._joint_damping_low
                label = "LOW"
                done = True
            else:
                mass = [
                    hi + (lo - hi) * alpha
                    for lo, hi in zip(self._mass_low, self._mass_high)
                ]
                jd = (
                    self._joint_damping_high
                    + (self._joint_damping_low - self._joint_damping_high) * alpha
                )
                label = "STEPPING %d/%d" % (self._step_index, self._step_count)
                done = False

        self._apply_values(mass, jd, label)
        if done:
            self.get_logger().info("Step-down complete -> LOW")

    # ---- parameter writer --------------------------------------------------

    def _apply_values(self, mass, jd, state_label: str):
        if not self._set_param_client.service_is_ready():
            # Don't block here; just log. Wrench keeps firing so the next
            # transition will retry. Initialisation already gave us a brief
            # service availability check.
            self.get_logger().warn(
                "%s not ready - skipping apply" % self.SET_PARAM_SERVICE
            )
            return

        params = []

        p_mass = Parameter()
        p_mass.name = self.MASS_PARAM
        p_mass.value = ParameterValue()
        p_mass.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        p_mass.value.double_array_value = list(mass)
        params.append(p_mass)

        p_jd = Parameter()
        p_jd.name = self.JOINT_DAMPING_PARAM
        p_jd.value = ParameterValue()
        p_jd.value.type = ParameterType.PARAMETER_DOUBLE
        p_jd.value.double_value = float(jd)
        params.append(p_jd)

        req = SetParameters.Request()
        req.parameters = params
        future = self._set_param_client.call_async(req)
        future.add_done_callback(
            lambda fut, s=state_label: self._on_set_done(fut, s)
        )

    def _on_set_done(self, future, state: str):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error("set_parameters raised: %s" % e)
            return
        if result is None:
            self.get_logger().error("set_parameters returned no result")
            return
        failures = [
            (i, r.reason)
            for i, r in enumerate(result.results)
            if not r.successful
        ]
        if failures:
            self.get_logger().error(
                "set_parameters failed for state %s: %s" % (state, failures)
            )
        else:
            self.get_logger().info("set_parameters OK for state %s" % state)


def main():
    rclpy.init()
    node = VariableImpedanceNode()
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
