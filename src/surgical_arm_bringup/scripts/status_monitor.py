#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('surgical_arm_status_monitor')
        self.pub = self.create_publisher(String, '/surgical_arm/status', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.cmd_cb, 10)
        self.last_cmd = 'no trajectory received yet'

    def cmd_cb(self, msg):
        self.last_cmd = f"received trajectory: {len(msg.points)} points, joints={list(msg.joint_names)}"
        self.publish()

    def joint_cb(self, msg):
        self.publish()

    def publish(self):
        m = String()
        m.data = self.last_cmd
        self.pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(StatusMonitor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
