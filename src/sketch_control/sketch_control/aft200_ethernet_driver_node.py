"""
ROS 2 driver for the AIDIN Robotics AFT200-D80-EN Ethernet F/T sensor.

The sensor streams 13 big-endian float32 values per sample:
Fx, Fy, Fz, Tx, Ty, Tz, Ax, Ay, Az, Gx, Gy, Gz, temperature.
Only the 6-axis wrench is published here; IMU values are kept in status.
"""

import json
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


START_CMD = b"\x00\x03\x02"
STOP_CMD = b"\x00\x03\x03"
BIAS_CMD = b"\x00\x03\x04"
FRAME_SIZE = 52
VALUE_COUNT = 13


class AFT200EthernetDriver(Node):
    def __init__(self):
        super().__init__("aft200_ethernet_driver")

        self.declare_parameter("sensor_ip", "192.168.1.199")
        self.declare_parameter("sensor_port", 0)
        self.declare_parameter("transport", "tcp")
        self.declare_parameter("local_ip", "")
        self.declare_parameter("local_port", 0)
        self.declare_parameter("frame_id", "tcp")
        self.declare_parameter("wrench_topic", "/aft200/ft")
        self.declare_parameter("status_topic", "/aft200/status")
        self.declare_parameter("bias_topic", "/aft200/bias")
        self.declare_parameter("connect_timeout_s", 1.0)
        self.declare_parameter("read_timeout_s", 1.0)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("rate_limit_hz", 250.0)
        self.declare_parameter("bias_on_start", False)

        self.sensor_ip = str(self.get_parameter("sensor_ip").value).strip()
        self.sensor_port = int(self.get_parameter("sensor_port").value)
        self.transport = str(self.get_parameter("transport").value).strip().lower()
        self.local_ip = str(self.get_parameter("local_ip").value).strip()
        self.local_port = int(self.get_parameter("local_port").value)
        self.frame_id = str(self.get_parameter("frame_id").value).strip() or "tcp"
        self.wrench_topic = str(self.get_parameter("wrench_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.bias_topic = str(self.get_parameter("bias_topic").value).strip()
        self.connect_timeout_s = float(
            self.get_parameter("connect_timeout_s").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.reconnect_delay_s = float(
            self.get_parameter("reconnect_delay_s").value)
        self.rate_limit_hz = float(self.get_parameter("rate_limit_hz").value)
        self.bias_on_start = bool(self.get_parameter("bias_on_start").value)

        if self.transport not in ("tcp", "udp"):
            self.get_logger().warn(
                f"unknown transport={self.transport!r}; falling back to tcp")
            self.transport = "tcp"

        self.wrench_pub = self.create_publisher(
            WrenchStamped, self.wrench_topic, 10)
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10)
        self.create_subscription(Bool, self.bias_topic, self._on_bias, 10)

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._socket = None
        self._connected = False
        self._active_port = None
        self._samples = 0
        self._last_values = None
        self._last_error = ""
        self._last_publish_time = 0.0
        self._bias_requested = False

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._publish_status)

        ports = self._candidate_ports()
        self.get_logger().info(
            "AFT200 Ethernet driver 시작\n"
            f"  sensor={self.sensor_ip}:{ports if self.sensor_port == 0 else self.sensor_port}\n"
            f"  transport={self.transport}, frame={self.frame_id}\n"
            f"  wrench_topic={self.wrench_topic}")

    def destroy_node(self):
        self._stop_event.set()
        self._close_socket(send_stop=True)
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()

    def _candidate_ports(self):
        if self.sensor_port > 0:
            return [self.sensor_port]
        if self.transport == "udp":
            return [50000, 8890]
        return [50000, 80]

    def _on_bias(self, msg):
        if msg.data:
            self._bias_requested = True

    def _set_status(self, connected, error="", active_port=None):
        with self._lock:
            self._connected = bool(connected)
            self._last_error = str(error or "")
            if active_port is not None:
                self._active_port = int(active_port)

    def _publish_status(self):
        with self._lock:
            values = self._last_values
            payload = {
                "connected": self._connected,
                "transport": self.transport,
                "sensor_ip": self.sensor_ip,
                "sensor_port": self._active_port,
                "samples": self._samples,
                "last_error": self._last_error,
            }
        if values is not None:
            payload.update({
                "force_n": list(values[:3]),
                "torque_nm": list(values[3:6]),
                "temperature": values[12],
            })
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _worker(self):
        while rclpy.ok() and not self._stop_event.is_set():
            for port in self._candidate_ports():
                if self._stop_event.is_set():
                    return
                try:
                    if self.transport == "udp":
                        self._run_udp(port)
                    else:
                        self._run_tcp(port)
                except Exception as exc:
                    self._set_status(False, f"{type(exc).__name__}: {exc}", port)
                    self.get_logger().warn(
                        f"AFT200 연결 실패 {self.sensor_ip}:{port} "
                        f"({self.transport}) - {exc}",
                        throttle_duration_sec=2.0)
                finally:
                    self._close_socket(send_stop=False)
                if self.sensor_port > 0:
                    break
            self._stop_event.wait(max(self.reconnect_delay_s, 0.1))

    def _open_tcp(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(max(self.connect_timeout_s, 0.1))
        sock.connect((self.sensor_ip, port))
        sock.settimeout(max(self.read_timeout_s, 0.1))
        return sock

    def _open_udp(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(max(self.read_timeout_s, 0.1))
        if self.local_ip or self.local_port > 0:
            sock.bind((self.local_ip or "0.0.0.0", self.local_port))
        sock.connect((self.sensor_ip, port))
        return sock

    def _run_tcp(self, port):
        sock = self._open_tcp(port)
        with self._lock:
            self._socket = sock
        self._start_stream(sock, port)
        while not self._stop_event.is_set():
            payload = self._recv_exact(sock, FRAME_SIZE)
            self._handle_frame(payload, port)

    def _run_udp(self, port):
        sock = self._open_udp(port)
        with self._lock:
            self._socket = sock
        self._start_stream(sock, port)
        while not self._stop_event.is_set():
            payload = sock.recv(4096)
            if len(payload) < FRAME_SIZE:
                continue
            self._handle_frame(payload[:FRAME_SIZE], port)

    def _start_stream(self, sock, port):
        sock.send(START_CMD)
        if self.bias_on_start:
            sock.send(BIAS_CMD)
        self._set_status(True, "", port)
        self.get_logger().info(
            f"AFT200 stream connected: {self.sensor_ip}:{port} ({self.transport})")

    def _recv_exact(self, sock, size):
        chunks = []
        remaining = size
        while remaining > 0 and not self._stop_event.is_set():
            chunk = sock.recv(remaining)
            if not chunk:
                raise ConnectionError("socket closed")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _handle_frame(self, payload, port):
        if len(payload) != FRAME_SIZE:
            return
        values = struct.unpack(">13f", payload)
        now = time.monotonic()
        if self._bias_requested:
            self._send_bias()
            self._bias_requested = False

        period = 0.0
        if self.rate_limit_hz > 0.0:
            period = 1.0 / self.rate_limit_hz
        if period > 0.0 and now - self._last_publish_time < period:
            return
        self._last_publish_time = now

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = float(values[0])
        msg.wrench.force.y = float(values[1])
        msg.wrench.force.z = float(values[2])
        msg.wrench.torque.x = float(values[3])
        msg.wrench.torque.y = float(values[4])
        msg.wrench.torque.z = float(values[5])
        self.wrench_pub.publish(msg)

        with self._lock:
            self._connected = True
            self._active_port = int(port)
            self._samples += 1
            self._last_values = tuple(float(v) for v in values)
            self._last_error = ""

    def _send_bias(self):
        with self._lock:
            sock = self._socket
        if sock is None:
            return
        try:
            sock.send(BIAS_CMD)
            self.get_logger().info("AFT200 hardware bias command sent")
        except OSError as exc:
            self._set_status(False, f"bias failed: {exc}")

    def _close_socket(self, send_stop):
        with self._lock:
            sock = self._socket
            self._socket = None
        if sock is None:
            return
        try:
            if send_stop:
                sock.send(STOP_CMD)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AFT200EthernetDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
