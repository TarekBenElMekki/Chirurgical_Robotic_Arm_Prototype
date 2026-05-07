#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class FinalPositionController(Node):
    def __init__(self):
        super().__init__("final_position_controller")

        self.joint_names = [
            "base_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "arm_wrist_joint",
            "tool_roll_joint",
            "wrist_pitch_joint",
            "distal_tool_joint",
        ]

        self.positions = [0.0] * len(self.joint_names)

        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_fresh_pub = self.create_publisher(JointState, "/joint_states_fresh", 10)

        self.motor_command_pub = self.create_publisher(Float32, "/motor/command_angle_deg", 10)
        self.motor_real_pub = self.create_publisher(Float32, "/motor/real_angle_deg", 10)
        self.motor_error_pub = self.create_publisher(Float32, "/motor/error_deg", 10)

        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info("Final position controller started")
        self.get_logger().info("MoveIt action: /arm_controller/follow_joint_trajectory")
        self.get_logger().info("Only the final trajectory point will be sent to the motor")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(msg)
        self.joint_fresh_pub.publish(msg)

    def base_yaw_rad_to_servo_deg(self, angle_rad):
        joint_min = -1.5708
        joint_max = 1.5708
        servo_min = 0.0
        servo_max = 180.0

        angle_rad = max(joint_min, min(joint_max, float(angle_rad)))
        ratio = (angle_rad - joint_min) / (joint_max - joint_min)
        return servo_min + ratio * (servo_max - servo_min)

    async def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        if len(trajectory.points) == 0:
            self.get_logger().error("Rejected empty trajectory")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty trajectory"
            return result

        final_point = trajectory.points[-1]

        for i, joint in enumerate(trajectory.joint_names):
            if joint in self.joint_names and i < len(final_point.positions):
                idx = self.joint_names.index(joint)
                self.positions[idx] = float(final_point.positions[i])

        base_idx = self.joint_names.index("base_yaw_joint")
        base_rad = self.positions[base_idx]
        motor_deg = self.base_yaw_rad_to_servo_deg(base_rad)

        self.motor_command_pub.publish(Float32(data=float(motor_deg)))

        # Until encoder feedback is added, this is estimated real angle.
        self.motor_real_pub.publish(Float32(data=float(motor_deg)))
        self.motor_error_pub.publish(Float32(data=0.0))

        self.get_logger().info(
            f"Received MoveIt trajectory with {len(trajectory.points)} points. "
            f"Used final point only. base_yaw={base_rad:.4f} rad -> motor={motor_deg:.1f} deg"
        )

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Final point accepted"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FinalPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
