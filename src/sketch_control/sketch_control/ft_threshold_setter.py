#!/usr/bin/env python3
"""AFT200 contact-force threshold setter — MoveIt 불필요.

직접교시(free-drive)로 로봇을 벽에 밀착시켜, "이 정도면 좋겠다" 하는 접촉력을
직접 느끼며 설정한다.

흐름
----
1) 벽 앞 접촉 직전 자세에서(안 닿게) 'z' + Enter 로 영점(bias) 잡기
   - bias = 그 자세에서의 EOAT 무게 + 센서 오프셋 (중력 성분 포함)
   - 자세를 크게 바꾸면 중력 성분이 달라지니, 접촉할 자세에서 영점을 잡을 것
2) 로봇을 벽에 밀착 → 표면 normal 이 있으면 Fn, 없으면 |F|(N) 가 표시됨
3) 원하는 힘에서 Enter → 그 값을 target 접촉력으로 캡처
4) 'done' → target_force_n / contact_threshold_n / abort_force_n 제안 + JSON 저장

MoveIt/move_group 없이 /aft200/ft 만 있으면 된다. 최소 실행:
  # 로봇 직접교시(free-drive) 모드 ON
  ros2 launch sketch_control rb10_calibration_tf.launch.py \
    robot_ip:=10.0.2.7 use_fake_hardware:=false use_isaac_sim:=false
  ros2 run sketch_control rbpodo_eft_bridge
  ros2 run sketch_control ft_threshold_setter
"""

import json
import select
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from sketch_control.rotation_utils import quat_apply


DEFAULT_WRENCH_TOPIC = "/aft200/ft"
DEFAULT_OUTPUT = str(Path.home() / "sketch_robot_ws" / "aft200_force_threshold.json")
WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane"
WORK_AREA_REFINED_PLANE_TOPIC = "/perception/work_area_plane_refined"

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _read_stdin_nonblock() -> Optional[str]:
    if not sys.stdin or not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    line = sys.stdin.readline().strip().lower()
    if line in ("done", "q", "quit", "finish", "end", "stop"):
        return "finish"
    if line in ("z", "zero", "tare"):
        return "zero"
    return "capture"   # 빈 Enter 포함


class FtThresholdSetter(Node):

    def __init__(self):
        super().__init__("ft_threshold_setter")
        self.wrench_topic = str(
            self.declare_parameter("wrench_topic", DEFAULT_WRENCH_TOPIC).value)
        self.base_frame = str(self.declare_parameter("base_frame", "link0").value)
        self.sensor_frame = str(self.declare_parameter("sensor_frame", "tcp").value)
        self.force_sign = float(self.declare_parameter("force_sign", 1.0).value)
        self.require_surface_normal = bool(
            self.declare_parameter("require_surface_normal", False).value)
        self.use_manual_normal = bool(
            self.declare_parameter("use_manual_normal", False).value)
        self.manual_normal = np.array([
            float(self.declare_parameter("manual_normal_x", 0.0).value),
            float(self.declare_parameter("manual_normal_y", 0.0).value),
            float(self.declare_parameter("manual_normal_z", 0.0).value),
        ], dtype=float)
        self.zero_samples = int(self.declare_parameter("zero_samples", 100).value)
        self.output_path = str(
            self.declare_parameter("output_path", DEFAULT_OUTPUT).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bias = None                    # (3,) force bias
        self.bias_mode = None               # "base" or "raw:<frame>"
        self.latest_force = None            # (3,) latest raw force
        self.latest_force_frame = None
        self.surface_normal = None          # (3,) base frame normal
        self.surface_source = "none"
        self.captures: List[Dict] = []      # captured force measurements
        self._last_print = 0.0
        self._last_tf_warn = 0.0

        self.create_subscription(
            WrenchStamped, self.wrench_topic, self._on_wrench,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, WORK_AREA_PLANE_TOPIC, self._on_surface, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, WORK_AREA_REFINED_PLANE_TOPIC,
            self._on_refined_surface, LATCHED_QOS)

        if self.use_manual_normal:
            norm = float(np.linalg.norm(self.manual_normal))
            if norm > 1e-9:
                self.surface_normal = self.manual_normal / norm
                self.surface_source = "manual"
            else:
                self.get_logger().warn(
                    "manual normal 사용 요청됐지만 벡터가 0입니다. plane topic 대기")

        self.get_logger().info(
            "AFT200 접촉력 threshold 설정 | wrench: "
            f"{self.wrench_topic}\n"
            f"  base={self.base_frame}, sensor={self.sensor_frame}\n"
            f"  surface normal: {WORK_AREA_REFINED_PLANE_TOPIC} 우선, "
            f"fallback={WORK_AREA_PLANE_TOPIC}")

    def _on_wrench(self, msg: WrenchStamped):
        f = msg.wrench.force
        self.latest_force = np.array([f.x, f.y, f.z], dtype=float)
        self.latest_force_frame = msg.header.frame_id or self.sensor_frame

    def _on_surface(self, msg: PoseStamped):
        if self.surface_source in ("d405_refined", "manual"):
            return
        self._set_surface(msg, "zed")

    def _on_refined_surface(self, msg: PoseStamped):
        if self.surface_source == "manual":
            return
        self._set_surface(msg, "d405_refined")

    def _set_surface(self, msg: PoseStamped, source: str):
        frame = msg.header.frame_id or self.base_frame
        q = msg.pose.orientation
        normal = quat_apply([q.x, q.y, q.z, q.w], [0.0, 0.0, 1.0])
        try:
            normal = self._transform_vector_to_base(normal, frame)
        except TransformException as exc:
            self.get_logger().warn(
                f"surface normal TF 실패 ({self.base_frame}<-{frame}): {exc}",
                throttle_duration_sec=2.0)
            return
        normal = np.asarray(normal, dtype=float)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            self.get_logger().warn("surface normal 크기가 0에 가까움 — 무시")
            return
        self.surface_normal = normal / norm
        self.surface_source = source
        self.get_logger().info(
            f"active surface={source}, normal_base=("
            f"{self.surface_normal[0]:+.2f},{self.surface_normal[1]:+.2f},"
            f"{self.surface_normal[2]:+.2f})",
            throttle_duration_sec=2.0)

    def _transform_vector_to_base(self, vector, frame: str):
        if frame in (self.base_frame, "world", "World"):
            return np.asarray(vector, dtype=float)
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, frame, Time(), timeout=Duration(seconds=0.05))
        q = tf.transform.rotation
        return quat_apply([q.x, q.y, q.z, q.w], vector)

    def _current_force_vector(self) -> Optional[Tuple[np.ndarray, str, Optional[str]]]:
        if self.latest_force is None:
            return None
        frame = self.latest_force_frame or self.sensor_frame
        try:
            force_base = self._transform_vector_to_base(self.latest_force, frame)
            return np.asarray(force_base, dtype=float), "base", None
        except TransformException as exc:
            now = time.time()
            if now - self._last_tf_warn > 2.0:
                self._last_tf_warn = now
                self.get_logger().warn(
                    f"wrench TF 실패 ({self.base_frame}<-{frame}). "
                    f"surface normal 캡처 불가, |F| fallback 사용: {exc}")
            return self.latest_force.copy(), f"raw:{frame}", str(exc)

    def _corrected(self):
        current = self._current_force_vector()
        if current is None:
            return None
        force, mode, err = current
        if self.bias is None:
            return force, mode, err
        if mode != self.bias_mode:
            self.get_logger().warn(
                f"영점 좌표계({self.bias_mode})와 현재 힘 좌표계({mode})가 다릅니다. "
                "접촉 없이 'z'로 영점을 다시 잡으세요.",
                throttle_duration_sec=2.0)
            return None
        return force - self.bias, mode, err

    def _measurement(self) -> Optional[Dict]:
        corrected = self._corrected()
        if corrected is None:
            return None
        c, mode, err = corrected
        mag = float(np.linalg.norm(c))
        dom = int(np.argmax(np.abs(c)))
        axis = "XYZ"[dom]

        if mode == "base" and self.surface_normal is not None:
            raw_normal = float(np.dot(c, self.surface_normal))
            signed_normal = self.force_sign * raw_normal
            sign_hint = 1.0 if raw_normal >= 0.0 else -1.0
            return {
                "basis": "surface_normal",
                "value_n": abs(signed_normal),
                "signed_normal_n": signed_normal,
                "raw_normal_n": raw_normal,
                "force_magnitude_n": mag,
                "force_vector": c.tolist(),
                "force_frame": self.base_frame,
                "dominant_axis": axis,
                "surface_source": self.surface_source,
                "surface_normal_base": self.surface_normal.tolist(),
                "force_sign": self.force_sign,
                "recommended_force_sign": sign_hint,
            }

        if self.require_surface_normal:
            self.get_logger().warn(
                "surface normal 대기 중 — /perception/work_area_plane_refined "
                "또는 manual_normal 파라미터가 필요합니다.",
                throttle_duration_sec=2.0)
            return None

        return {
            "basis": "force_magnitude_fallback",
            "value_n": mag,
            "force_magnitude_n": mag,
            "force_vector": c.tolist(),
            "force_frame": mode,
            "dominant_axis": axis,
            "surface_source": self.surface_source,
            "tf_error": err,
        }

    def _zero(self) -> bool:
        self.get_logger().info(
            f"영점 측정 중... 접촉 없이 정지 유지 ({self.zero_samples} samples)")
        samples = []
        deadline = time.time() + 6.0
        mode = None
        while rclpy.ok() and len(samples) < self.zero_samples and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self._current_force_vector()
            if current is None:
                continue
            force, force_mode, _err = current
            if mode is None:
                mode = force_mode
            if force_mode != mode:
                continue
            samples.append(force.copy())
        if len(samples) < 10:
            self.get_logger().warn(
                f"샘플 부족({len(samples)}) — /aft200/ft 발행 확인 후 다시 'z'")
            return False
        self.bias = np.mean(np.asarray(samples), axis=0)
        self.bias_mode = mode
        self.get_logger().info(
            f"영점(bias) 설정: ({self.bias[0]:+.2f},{self.bias[1]:+.2f},"
            f"{self.bias[2]:+.2f})N [{self.bias_mode}] — "
            "이제 벽에 밀착하고 원하는 힘에서 Enter")
        return True

    def _capture(self):
        if self.bias is None:
            self.get_logger().warn("먼저 접촉 없이 'z' + Enter 로 영점을 잡으세요")
            return
        meas = self._measurement()
        if meas is None:
            self.get_logger().warn("힘값 미수신")
            return
        self.captures.append(meas)
        f = meas["force_vector"]
        if meas["basis"] == "surface_normal":
            self.get_logger().info(
                f"[캡처 {len(self.captures)}] Fn={meas['signed_normal_n']:+.2f}N "
                f"target={meas['value_n']:.2f}N |F|={meas['force_magnitude_n']:.2f}N "
                f"Fbase=({f[0]:+.2f},{f[1]:+.2f},{f[2]:+.2f})N "
                f"(surface={meas['surface_source']}, "
                f"force_sign_hint={meas['recommended_force_sign']:+.0f})")
        else:
            self.get_logger().info(
                f"[캡처 {len(self.captures)}] |F|={meas['value_n']:.2f}N  "
                f"F=({f[0]:+.2f},{f[1]:+.2f},{f[2]:+.2f})N "
                f"(주 접촉축 {meas['dominant_axis']}, normal 없음 fallback)")

    def _print_live(self):
        now = time.time()
        if now - self._last_print < 0.25:
            return
        self._last_print = now
        meas = self._measurement()
        if meas is None:
            self.get_logger().info("waiting: /aft200/ft", throttle_duration_sec=2.0)
            return
        tag = "" if self.bias is not None else "  (영점 전 — 'z' 로 영점)"
        f = meas["force_vector"]
        if meas["basis"] == "surface_normal":
            lead = (
                f"Fn={meas['signed_normal_n']:+6.2f}N "
                f"target={meas['value_n']:6.2f}N "
                f"|F|={meas['force_magnitude_n']:6.2f}N "
                f"sign_hint={meas['recommended_force_sign']:+.0f}"
            )
        else:
            lead = f"|F|={meas['value_n']:6.2f}N normal=waiting/fallback"
        sys.stdout.write(
            f"\r{lead}  F=({f[0]:+6.2f},{f[1]:+6.2f},{f[2]:+6.2f})N"
            f"  captures={len(self.captures)}{tag}   ")
        sys.stdout.flush()

    def run(self) -> bool:
        self.get_logger().info(f"{self.wrench_topic} 대기 중...")
        deadline = time.time() + 20.0
        while rclpy.ok() and self.latest_force is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_force is None:
            self.get_logger().error(
                f"{self.wrench_topic} 미수신 — rbpodo hardware + rbpodo_eft_bridge "
                "실행 확인")
            return False

        print()
        print("=" * 60)
        print("사용법:")
        print("  z + Enter  : 영점(접촉 없이 정지 상태에서)")
        print("  Enter      : 현재 힘을 target 접촉력으로 캡처 (normal 있으면 Fn 기준)")
        print("  done       : 종료 + 결과 저장")
        print("  normal 없음: |F| fallback 사용. 엄밀한 값은 perception plane 또는 manual_normal 필요")
        print("=" * 60)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            cmd = _read_stdin_nonblock()
            if cmd == "finish":
                break
            elif cmd == "zero":
                print()
                self._zero()
            elif cmd == "capture":
                print()
                self._capture()
            else:
                self._print_live()

        if not self.captures:
            self.get_logger().warn("캡처된 값 없음 — 저장 생략")
            return False
        return self._finish()

    def _finish(self) -> bool:
        caps = np.asarray([float(c["value_n"]) for c in self.captures], dtype=float)
        # 마지막 캡처를 target 으로 (여러 번 캡처했다면 마지막이 최종 의도).
        target = float(caps[-1])
        contact = float(max(0.3, round(0.4 * target, 1)))
        abort = float(round(max(5.0, target * 2.0, target + 3.0), 1))
        last = self.captures[-1]
        normal_caps = [c for c in self.captures if c["basis"] == "surface_normal"]
        force_sign_hint = (
            float(normal_caps[-1]["recommended_force_sign"])
            if normal_caps
            else self.force_sign
        )
        force_sign_arg = f"{force_sign_hint:.1f}"

        print()
        print("=" * 60)
        print("AFT200 접촉력 설정 결과")
        print("=" * 60)
        print(f"캡처값 [N]: {[round(float(v),2) for v in caps]}")
        print(f"  최소={caps.min():.2f}  최대={caps.max():.2f}  평균={caps.mean():.2f}")
        print(f"선택 target(마지막 캡처): {target:.2f} N")
        if last["basis"] == "surface_normal":
            print(
                f"기준: surface normal force "
                f"(source={last['surface_source']}, "
                f"raw Fn={last['raw_normal_n']:+.2f}N)")
            print(f"권장 ft_force_sign: {force_sign_hint:+.0f}")
        else:
            print("기준: |F| fallback (surface normal 미사용)")
        print()
        print("제안 파라미터:")
        print(f"  target_force_n     = {target:.1f}   (원하는 접촉/도포 힘)")
        print(f"  contact_threshold_n= {contact:.1f}   (접촉 감지 임계, target 의 ~30%)")
        print(f"  abort_force_n      = {abort:.1f}   (과압 안전 정지)")
        print()
        print("적용 (ft_normal_controller):")
        print("  rb10_real_perception_sketch.launch.py 에 추가")
        print(
            f"    ft_force_sign:={force_sign_arg} "
            f"ft_target_force_n:={target:.1f} ft_abort_force_n:={abort:.1f}")
        print("  또는 노드 파라미터로")
        print(
            f"    -p force_sign:={force_sign_arg} "
            f"-p target_force_n:={target:.1f} "
            f"-p contact_threshold_n:={contact:.1f}")
        print("=" * 60)

        out = {
            "wrench_topic": self.wrench_topic,
            "base_frame": self.base_frame,
            "sensor_frame": self.sensor_frame,
            "bias_N": None if self.bias is None else self.bias.tolist(),
            "bias_mode": self.bias_mode,
            "captures_N": [float(v) for v in caps],
            "capture_details": self.captures,
            "force_basis": last["basis"],
            "force_sign": force_sign_hint,
            "surface_source": last.get("surface_source", "none"),
            "surface_normal_base": last.get("surface_normal_base"),
            "target_force_n": target,
            "contact_threshold_n": contact,
            "abort_force_n": abort,
        }
        path = Path(self.output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(out, indent=2))
        self.get_logger().info(f"저장 → {path}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FtThresholdSetter()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
