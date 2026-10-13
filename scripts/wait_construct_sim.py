#!/usr/bin/env python3
"""Wait for complete state and advancing simulated time before starting controllers."""
import time
import rclpy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node('wait_construct_sim')
states = []
clocks = []
node.create_subscription(JointState,'/joint_states',lambda m:states.append(m),10)
node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec/1e9),10)
end=time.monotonic()+300
try:
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.2)
        if states and len(states[-1].name)==14 and len(clocks)>1 and clocks[-1]>clocks[0]:
            print('Construct simulation ready: 14 joints and advancing /clock',flush=True)
            break
    else:
        raise SystemExit('No running Construct simulation in ROS domain. Start run_construct_isaac.sh and press Play.')
finally:
    node.destroy_node()
    rclpy.shutdown()
