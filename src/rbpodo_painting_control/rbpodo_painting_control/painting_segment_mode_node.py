#!/usr/bin/env python3
"""Simple timed mode publisher for admittance painting tests.

This helper does not move the robot. It only publishes process mode and desired
force so an externally executed MoveIt/JTC trajectory can be synchronized with
the painting admittance wrench reference node during bench tests.
"""

import csv
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


NONCONTACT_MODES = {
    "IDLE",
    "APPROACH",
    "APPROACH_PRECONTACT",
    "CONTACT_SEARCH",
    "RETRACT",
    "TRAVEL",
    "NONCONTACT",
    "FINISH_RETRACT",
    "FINAL_RETRACT",
    "ABORT",
}


@dataclass
class ScheduledMode:
    mode: str
    force_n: float
    duration_s: float
    description: str
    row_number: int


class PaintingSegmentModeNode(Node):
    def __init__(self) -> None:
        super().__init__("painting_segment_mode")

        self.declare_parameter("schedule_csv", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_enable_force", False)
        self.declare_parameter("publish_enable_force", True)

        self._schedule_path = Path(str(self.get_parameter("schedule_csv").value))
        self._rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._loop = bool(self.get_parameter("loop").value)
        self._enable_force = bool(self.get_parameter("start_enable_force").value)
        self._publish_enable = bool(self.get_parameter("publish_enable_force").value)
        self._schedule = self._load_schedule(self._schedule_path)
        self._index = 0
        self._segment_started_at = time.monotonic()
        self._last_log_index = None

        self._mode_pub = self.create_publisher(String, "/painting_admittance/mode", 10)
        self._force_pub = self.create_publisher(
            Float64, "/painting_admittance/desired_force_n", 10
        )
        self._enable_pub = self.create_publisher(
            Bool, "/painting_admittance/enable_force", 10
        )
        self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info(
            "Loaded %d painting admittance mode rows from %s. This node does not move the robot."
            % (len(self._schedule), self._schedule_path)
        )

    def _load_schedule(self, path: Path) -> list[ScheduledMode]:
        if not path.exists():
            raise FileNotFoundError("schedule_csv does not exist: %s" % path)
        rows = []
        with path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ValueError("schedule CSV has no header")
            for row_number, row in enumerate(reader, start=2):
                mode = str(row.get("mode", "IDLE") or "IDLE").strip().upper()
                force_n = abs(float(row.get("force_n", "0.0") or 0.0))
                duration_s = max(0.0, float(row.get("duration_s", "0.0") or 0.0))
                description = str(row.get("description", "") or "")
                if mode in NONCONTACT_MODES and force_n > 1e-9:
                    self.get_logger().warn(
                        "row %d %s has nonzero force %.3f; publishing force 0.0"
                        % (row_number, mode, force_n)
                    )
                    force_n = 0.0
                rows.append(ScheduledMode(mode, force_n, duration_s, description, row_number))
        if not rows:
            raise ValueError("schedule CSV contains no rows")
        return rows

    def _advance_if_needed(self) -> None:
        now = time.monotonic()
        current = self._schedule[self._index]
        if now - self._segment_started_at < current.duration_s:
            return
        self._index += 1
        if self._index >= len(self._schedule):
            if self._loop:
                self._index = 0
            else:
                self._index = len(self._schedule) - 1
                return
        self._segment_started_at = now
        self._last_log_index = None

    def _on_timer(self) -> None:
        self._advance_if_needed()
        current = self._schedule[self._index]
        if self._last_log_index != self._index:
            self._last_log_index = self._index
            self.get_logger().info(
                "mode row %d: %s force=%.3f duration=%.3f %s"
                % (
                    current.row_number,
                    current.mode,
                    current.force_n,
                    current.duration_s,
                    current.description,
                )
            )

        mode = String()
        mode.data = current.mode
        self._mode_pub.publish(mode)

        force = Float64()
        force.data = current.force_n
        self._force_pub.publish(force)

        if self._publish_enable:
            enable = Bool()
            enable.data = self._enable_force
            self._enable_pub.publish(enable)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaintingSegmentModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
