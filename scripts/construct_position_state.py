#!/usr/bin/env python3
"""Report actual ros2_control compliance mode to the unchanged spray gate."""


def observed_compliance(controllers):
    active = {c.name: c for c in controllers if c.state == 'active'}
    if not {'left_manipulator_controller', 'right_manipulator_controller'} <= active.keys():
        return None
    return any('admittance' in c.type.lower() or 'admittance' in c.name.lower()
               for c in active.values())


def create_poll_timer(node, callback):
    from rclpy.clock import Clock, ClockType
    # The original receiver's freshness window is wall time, also when Isaac
    # is slow or paused. Only actual controller responses refresh its state.
    return node.create_timer(.1, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


def main():
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from std_msgs.msg import Bool
    rclpy.init()
    node = rclpy.create_node('construct_position_controller_state')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    publisher = node.create_publisher(Bool, '/admittance_controller/compliance_active', 1)
    pending = None

    def completed(future):
        nonlocal pending
        pending = None
        try:
            state = observed_compliance(future.result().controller)
            if state is not None:
                publisher.publish(Bool(data=state))
        except Exception as exc:
            node.get_logger().warning(str(exc))

    def tick():
        nonlocal pending
        if pending is None and client.service_is_ready():
            pending = client.call_async(ListControllers.Request())
            pending.add_done_callback(completed)

    create_poll_timer(node, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
