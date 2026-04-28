#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateFreshener(Node):
    def __init__(self):
        super().__init__('joint_state_freshener')
        self.pub = self.create_publisher(JointState, '/joint_states_fresh', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        out = JointState()
        out.header = msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = msg.name
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateFreshener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
