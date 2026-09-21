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
Python helper for the F/T tare services exposed by rbpodo_hardware.

Background
----------
RBPodoHardwareInterface exposes three std_srvs/Trigger services:

* /rbpodo_ft_tare/tare_ft performs the normal, strict tare.
* /rbpodo_ft_tare/confirm_free_space_and_tare performs the explicitly
  supervised tare intended for a known-safe, stationary, fully unloaded
  roller when a large source-side sensor offset prevents the strict tare.
* /rbpodo_ft_tare/runtime_free_space_tare performs the repeatable,
  execution-scoped tare at the accepted non-contact pre-contact pose. It does
  not compare the raw bias against a fixed value; the hardware requires fresh
  stationary joints, low-noise finite F/T, inactive trajectory/force/
  compliance, safety-board clearance, and a post-tare residual check.

Both procedures operate against the RAW eft_* readings (deadband is applied
to the published wrench, but tare bypasses the deadband so the new bias really
reflects the current pose's zero-load measurement).

This module wraps that service two ways:
1. As a standalone node that re-exposes the calls under ~/tare_ft and
   ~/confirm_free_space_and_tare and ~/runtime_free_space_tare, so other code
   can talk to the helper instead of the hardware interface directly.
2. As an importable class whose .tare() and
   .confirm_free_space_and_tare() methods block until the underlying service
   responds, with the same callback-safe async-polling pattern used in
   admittance_reset.

Usage
-----
Run as node:
    ros2 run rbpodo_bringup ft_tare_node.py

Manual trigger:
    ros2 service call /rbpodo_ft_tare_helper/tare_ft std_srvs/srv/Trigger

Explicit supervised trigger (robot stationary, roller fully free):
    ros2 service call \
      /rbpodo_ft_tare_helper/confirm_free_space_and_tare \
      std_srvs/srv/Trigger

Programmatic use:
    from rbpodo_bringup.ft_tare_client import FtTareClient
    node = FtTareClient()
    # spin in another thread / executor, then:
    ok, msg = node.tare(timeout_sec=5.0)
    # Only after the operator has verified that the robot is stationary and
    # the roller is fully free of walls, workpieces, fixtures, and supports:
    ok, msg = node.confirm_free_space_and_tare(timeout_sec=5.0)
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class FtTareClient(Node):

    # Service published by RBPodoHardwareInterface (see on_init).
    HARDWARE_TARE_SERVICE = "/rbpodo_ft_tare/tare_ft"
    HARDWARE_CONFIRMED_TARE_SERVICE = (
        "/rbpodo_ft_tare/confirm_free_space_and_tare"
    )
    HARDWARE_RUNTIME_TARE_SERVICE = "/rbpodo_ft_tare/runtime_free_space_tare"

    def __init__(self):
        super().__init__("rbpodo_ft_tare_helper")
        # All client + service callbacks in the same reentrant group so the
        # service callback can synchronously wait on the client's response
        # via the polling pattern used by .tare().
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        self._tare_client = self.create_client(
            Trigger,
            self.HARDWARE_TARE_SERVICE,
            callback_group=self._cb,
        )
        self._confirmed_tare_client = self.create_client(
            Trigger,
            self.HARDWARE_CONFIRMED_TARE_SERVICE,
            callback_group=self._cb,
        )
        self._runtime_tare_client = self.create_client(
            Trigger,
            self.HARDWARE_RUNTIME_TARE_SERVICE,
            callback_group=self._cb,
        )

        # Re-expose the same call under our own node namespace so callers can
        # use either /rbpodo_ft_tare/tare_ft (hardware) or
        # /rbpodo_ft_tare_helper/tare_ft (this node). Useful when chaining
        # helpers under a single Python process.
        self._tare_srv = self.create_service(
            Trigger,
            "~/tare_ft",
            self._on_tare_request,
            callback_group=self._cb,
        )
        self._confirmed_tare_srv = self.create_service(
            Trigger,
            "~/confirm_free_space_and_tare",
            self._on_confirmed_tare_request,
            callback_group=self._cb,
        )
        self._runtime_tare_srv = self.create_service(
            Trigger,
            "~/runtime_free_space_tare",
            self._on_runtime_tare_request,
            callback_group=self._cb,
        )

        self.get_logger().info(
            "F/T tare helper ready. Strict trigger: /%s/tare_ft. "
            "Supervised trigger: /%s/confirm_free_space_and_tare (use only "
            "with the robot stationary and the roller fully free). Runtime "
            "pre-contact trigger: /%s/runtime_free_space_tare."
            % (self.get_name(), self.get_name(), self.get_name())
        )

    # ---- service callback --------------------------------------------------

    def _on_tare_request(self, request, response):
        ok, msg = self.tare()
        response.success = ok
        response.message = msg
        return response

    def _on_confirmed_tare_request(self, request, response):
        ok, msg = self.confirm_free_space_and_tare()
        response.success = ok
        response.message = msg
        return response

    def _on_runtime_tare_request(self, request, response):
        del request
        ok, msg = self.runtime_free_space_tare()
        response.success = ok
        response.message = msg
        return response

    # ---- public API --------------------------------------------------------

    def tare(self, timeout_sec: float = 10.0):
        """
        Trigger the normal strict hardware-side tare.

        Returns ``(success, message)``. This remains the default API and does
        not opt into the supervised large-offset tare path.
        """
        with self._lock:
            return self._call_trigger(
                self._tare_client,
                "strict tare",
                timeout_sec,
            )

    def confirm_free_space_and_tare(self, timeout_sec: float = 10.0):
        """
        Trigger the explicitly supervised hardware-side tare.

        The caller's use of this method is the confirmation that the robot is
        stationary and that the roller is fully free: it must not touch a wall,
        workpiece, fixture, table, support, or person. Returns
        ``(success, message)``.
        """
        self.get_logger().warning(
            "SUPERVISED F/T TARE REQUESTED: proceed only when the robot is "
            "stationary and the roller is fully free (no contact with wall, "
            "workpiece, fixture, table, support, or person)."
        )
        with self._lock:
            return self._call_trigger(
                self._confirmed_tare_client,
                "confirmed free-space tare",
                timeout_sec,
            )

    def runtime_free_space_tare(self, timeout_sec: float = 10.0):
        """
        Run the repeatable pre-contact tare after motion has stopped.

        The hardware service independently checks fresh false trajectory,
        force-enable and compliance states as well as joint stationarity,
        safety-board state, wrench noise and the post-tare residual.  The
        caller must additionally have proven the roller is at the accepted
        non-contact pre-contact pose.
        """
        self.get_logger().warning(
            "RUNTIME F/T TARE REQUESTED: the accepted pre-contact pose must "
            "leave the roller fully free; trajectory, force and compliance "
            "must remain off until the service succeeds."
        )
        with self._lock:
            return self._call_trigger(
                self._runtime_tare_client,
                "runtime pre-contact tare",
                timeout_sec,
            )

    def _call_trigger(self, client, operation: str, timeout_sec: float):
        """Call one Trigger client and synchronously poll its future."""
        try:
            future = client.call_async(Trigger.Request())
        except Exception as e:
            msg = "%s call could not be sent: %s" % (operation, e)
            self.get_logger().error(msg)
            return False, msg

        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while not future.done():
            if time.monotonic() >= deadline:
                msg = "%s call timed out (deadline %.1fs)" % (
                    operation,
                    timeout_sec,
                )
                self.get_logger().error(msg)
                return False, msg
            time.sleep(0.05)

        try:
            result = future.result()
        except Exception as e:
            msg = "%s call raised: %s" % (operation, e)
            self.get_logger().error(msg)
            return False, msg

        if result is None:
            msg = "%s returned no result" % operation
            self.get_logger().error(msg)
            return False, msg

        self.get_logger().info(
            "%s result: success=%s message=%r"
            % (operation, result.success, result.message)
        )
        return result.success, result.message


def main():
    rclpy.init()
    node = FtTareClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # SIGINT may already have shut down the default context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
