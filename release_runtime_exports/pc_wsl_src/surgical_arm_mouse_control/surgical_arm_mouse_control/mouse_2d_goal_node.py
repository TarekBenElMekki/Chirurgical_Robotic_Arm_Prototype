#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Mouse2DGoalNode(Node):
    def __init__(self):
        super().__init__("mouse_2d_goal_node")

        self.declare_parameter("link_1_length", 0.46)
        self.declare_parameter("link_2_length", 0.38)
        self.declare_parameter("shoulder_x_world", 0.0)
        self.declare_parameter("shoulder_z_world", 0.38)

        self.declare_parameter("joint_1_min", -1.2)
        self.declare_parameter("joint_1_max", 1.2)
        self.declare_parameter("joint_2_min", -1.57)
        self.declare_parameter("joint_2_max", 1.57)

        self.declare_parameter("trajectory_time_sec", 2.0)
        self.declare_parameter("elbow_sign", 1.0)

        self.l1 = float(self.get_parameter("link_1_length").value)
        self.l2 = float(self.get_parameter("link_2_length").value)

        self.sx = float(self.get_parameter("shoulder_x_world").value)
        self.sz = float(self.get_parameter("shoulder_z_world").value)

        self.q1_min = float(self.get_parameter("joint_1_min").value)
        self.q1_max = float(self.get_parameter("joint_1_max").value)
        self.q2_min = float(self.get_parameter("joint_2_min").value)
        self.q2_max = float(self.get_parameter("joint_2_max").value)

        self.t = float(self.get_parameter("trajectory_time_sec").value)
        self.elbow_sign = float(self.get_parameter("elbow_sign").value)

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        self.sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.on_clicked_point,
            10,
        )

        self.get_logger().info("mouse_2d_goal_node ready.")
        self.get_logger().info("Use RViz Publish Point on an X-Z grid.")
        self.get_logger().info(f"Shoulder: x={self.sx:.3f}, z={self.sz:.3f}")
        self.get_logger().info(f"Links: L1={self.l1:.3f}, L2={self.l2:.3f}")

    def within_limits(self, q1, q2):
        return self.q1_min <= q1 <= self.q1_max and self.q2_min <= q2 <= self.q2_max

    def ik(self, x_world, z_world):
        # Robot zero pose extends toward negative X.
        u = -(x_world - self.sx)
        v = z_world - self.sz

        r = math.hypot(u, v)
        r_min = abs(self.l1 - self.l2)
        r_max = self.l1 + self.l2

        if r < r_min or r > r_max:
            raise ValueError(f"Unreachable target: r={r:.3f}, allowed=[{r_min:.3f}, {r_max:.3f}]")

        c2 = (u*u + v*v - self.l1*self.l1 - self.l2*self.l2) / (2.0*self.l1*self.l2)
        c2 = max(-1.0, min(1.0, c2))

        solutions = []
        for sign in [self.elbow_sign, -self.elbow_sign]:
            q2 = sign * math.acos(c2)
            q1 = math.atan2(v, u) - math.atan2(
                self.l2 * math.sin(q2),
                self.l1 + self.l2 * math.cos(q2),
            )
            solutions.append((q1, q2))

        for q1, q2 in solutions:
            if self.within_limits(q1, q2):
                return q1, q2, r

        raise ValueError(
            f"IK outside limits. Tried: "
            f"q1={solutions[0][0]:.3f}, q2={solutions[0][1]:.3f} and "
            f"q1={solutions[1][0]:.3f}, q2={solutions[1][1]:.3f}"
        )

    def on_clicked_point(self, msg):
        x = msg.point.x
        z = msg.point.z

        self.get_logger().info(
            f"Clicked point: frame={msg.header.frame_id}, x={x:.3f}, y={msg.point.y:.3f}, z={z:.3f}"
        )

        try:
            q1, q2, r = self.ik(x, z)
        except ValueError as e:
            self.get_logger().warn(str(e))
            return

        self.get_logger().info(
            f"IK OK: joint_1_y={q1:.3f}, joint_2_z={q2:.3f}, reach={r:.3f}"
        )

        self.send_goal(q1, q2)

    def send_goal(self, q1, q2):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/arm_controller/follow_joint_trajectory not available.")
            return

        traj = JointTrajectory()
        traj.joint_names = ["joint_1_y", "joint_2_z"]

        pt = JointTrajectoryPoint()
        pt.positions = [q1, q2]
        pt.time_from_start.sec = int(self.t)
        pt.time_from_start.nanosec = int((self.t - int(self.t)) * 1e9)

        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Trajectory rejected.")
            return

        self.get_logger().info("Trajectory accepted.")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Trajectory succeeded.")
        else:
            self.get_logger().error(
                f"Trajectory failed: code={result.error_code}, text={result.error_string}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Mouse2DGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
