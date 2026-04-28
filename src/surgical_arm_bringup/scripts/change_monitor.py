#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

class ChangeMonitor(Node):
    def __init__(self):
        super().__init__('change_monitor')

        self.threshold = 0.001
        self.last_joint_state = None
        self.last_desired = None
        self.last_actual = None

        self.create_subscription(
            JointState,
            '/joint_states_fresh',
            self.joint_state_cb,
            10
        )

        self.create_subscription(
            JointTrajectoryControllerState,
            '/hardware_arm_controller/controller_state',
            self.controller_state_cb,
            10
        )

        self.get_logger().info('Change monitor started. Printing only changed values.')

    def changed(self, old, new):
        if old is None:
            return True
        if len(old) != len(new):
            return True
        return any(abs(a - b) > self.threshold for a, b in zip(old, new))

    def joint_state_cb(self, msg):
        pos = list(msg.position)

        if self.changed(self.last_joint_state, pos):
            self.last_joint_state = pos
            self.get_logger().info(
                'JOINT FEEDBACK CHANGED:\n' +
                '\n'.join([f'  {n}: {p:.4f}' for n, p in zip(msg.name, pos)])
            )

    def controller_state_cb(self, msg):
        desired = list(msg.desired.positions)
        actual = list(msg.actual.positions)

        if self.changed(self.last_desired, desired):
            self.last_desired = desired
            self.get_logger().info(
                'ROS2_CONTROL DESIRED CHANGED:\n' +
                '\n'.join([f'  {n}: {p:.4f}' for n, p in zip(msg.joint_names, desired)])
            )

        if self.changed(self.last_actual, actual):
            self.last_actual = actual
            self.get_logger().info(
                'ROS2_CONTROL ACTUAL CHANGED:\n' +
                '\n'.join([f'  {n}: {p:.4f}' for n, p in zip(msg.joint_names, actual)])
            )

def main():
    rclpy.init()
    node = ChangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
