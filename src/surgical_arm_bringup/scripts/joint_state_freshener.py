#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateFreshener(Node):
    def __init__(self):
        super().__init__('joint_state_freshener')
        self.pub_normal = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_fresh = self.create_publisher(JointState, '/joint_states_fresh', 10)
        self.sub = self.create_subscription(JointState, '/raw_joint_states', self.cb, 10)
        self.get_logger().info('Freshening /raw_joint_states -> /joint_states and /joint_states_fresh')

    def cb(self, msg):
        fresh = JointState()
        fresh.header = msg.header
        fresh.header.stamp = self.get_clock().now().to_msg()
        fresh.name = msg.name
        fresh.position = msg.position
        fresh.velocity = msg.velocity
        fresh.effort = msg.effort
        self.pub_normal.publish(fresh)
        self.pub_fresh.publish(fresh)

def main():
    rclpy.init()
    node = JointStateFreshener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
