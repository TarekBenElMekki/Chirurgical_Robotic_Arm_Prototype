#!/usr/bin/env python3
import math
import rclpy

def safe_shutdown():
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray


class PwmMapper(Node):
    def __init__(self):
        super().__init__('pwm_mapper')

        self.joint_names = [
            'base_yaw_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'arm_wrist_joint',
            'tool_roll_joint',
            'wrist_pitch_joint',
            'distal_tool_joint',
        ]

        self.min_pwm = 500
        self.max_pwm = 2500
        self.center_pwm = 1500
        self.max_angle = math.pi / 2.0

        self.pub = self.create_publisher(Int32MultiArray, '/surgical_arm/pwm_commands', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)

        self.get_logger().info('PWM mapper started: /joint_states -> /surgical_arm/pwm_commands')

    def angle_to_pwm(self, angle):
        angle = max(-self.max_angle, min(self.max_angle, angle))
        pwm = self.center_pwm + (angle / self.max_angle) * 1000
        return int(max(self.min_pwm, min(self.max_pwm, pwm)))

    def cb(self, msg):
        pwm_values = []

        for joint in self.joint_names:
            if joint in msg.name:
                idx = msg.name.index(joint)
                pwm_values.append(self.angle_to_pwm(msg.position[idx]))
            else:
                pwm_values.append(self.center_pwm)

        out = Int32MultiArray()
        out.data = pwm_values
        self.pub.publish(out)

        self.get_logger().info(f'PWM: {pwm_values}', throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = PwmMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        safe_shutdown()

if __name__ == '__main__':
    main()
