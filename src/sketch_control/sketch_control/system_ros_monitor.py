"""ROS observation/abort adapter for the HTTP supervisor."""
import json
import threading
import time

class RosMonitor:
    """An independent ROS context: observe graph/readiness and request shutdown abort."""
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool, String
        self.rclpy, self.Bool = rclpy, Bool
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("sketch_system_api", context=self.context)
        self.lock = threading.Lock()
        self.nodes, self.readiness, self.execution = [], None, None
        self.updated, self.readiness_at, self.execution_at = 0.0, 0.0, 0.0
        self.abort = self.node.create_publisher(Bool, "/motion_abort", 10)
        self.force = self.node.create_publisher(Bool, "/painting_admittance/enable_force", 10)
        self.subscriptions = [self.node.create_subscription(
            String, topic, lambda msg, key=key: self._receive(key, msg), 10)
            for key, topic in (("readiness", "/painting_system/readiness"),
                               ("execution", "/painting_system/execution_status"))]
        self.node.create_timer(0.5, self._graph)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _receive(self, key, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except (TypeError, ValueError):
            return
        with self.lock:
            setattr(self, key, payload)
            setattr(self, key + "_at", time.monotonic())

    def _graph(self):
        names = self.node.get_node_names_and_namespaces()
        with self.lock:
            self.nodes = sorted({f"{ns.rstrip('/')}/{name}" for name, ns in names})
            self.updated = time.monotonic()

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            return {"graph_fresh": now - self.updated < 2,
                    "nodes": list(self.nodes),
                    "readiness": self.readiness if now - self.readiness_at < 2 else None,
                    "execution": self.execution if now - self.execution_at < 2 else None}

    def request_abort(self):
        self.abort.publish(self.Bool(data=True))
        self.force.publish(self.Bool(data=False))

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.context.try_shutdown()

