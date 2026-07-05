#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import MoveItErrorCodes


class Workspace2DInteractiveNode(Node):
    def __init__(self):
        super().__init__("workspace_2d_interactive_node")

        self.joint_1_name = "joint_1_y"
        self.joint_2_name = "joint_2_z"

        self.q1_min = -0.31415926535
        self.q1_max = 0.31415926535
        self.q2_min = -0.31415926535
        self.q2_max = 0.31415926535

        # Y-Z matrix plane
        self.panel_x = -0.58

        self.y_min = -0.45
        self.y_max = 0.45
        self.z_min = 0.00
        self.z_max = 0.90
        self.grid_step = 0.05

        self.target_y = 0.0
        self.target_z = 0.45

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/surgical_arm/workspace_matrix",
            qos,
        )

        self.clicked_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10,
        )

        # MoveIt2 action server
        self.moveit_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        self.publish_matrix()
        self.timer = self.create_timer(0.5, self.publish_matrix)

        self.get_logger().info("MOVEIT PLANNER Y-Z joint matrix ready.")
        self.get_logger().info("Use RViz Publish Point: one click = MoveIt2 plans and executes.")

    def make_point(self, y, z, x=None):
        p = Point()
        p.x = self.panel_x if x is None else float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def base_marker(self, marker_id, ns, marker_type):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def map_value(self, value, in_min, in_max, out_min, out_max):
        value = max(in_min, min(in_max, value))
        ratio = (value - in_min) / (in_max - in_min)
        return out_min + ratio * (out_max - out_min)

    def point_is_matrix(self, y, z):
        margin = 0.03
        return (
            self.y_min - margin <= y <= self.y_max + margin
            and self.z_min - margin <= z <= self.z_max + margin
        )

    def publish_matrix(self):
        markers = MarkerArray()

        grid = self.base_marker(0, "yz_joint_matrix_grid", Marker.LINE_LIST)
        grid.scale.x = 0.002
        grid.color.r = 0.45
        grid.color.g = 0.45
        grid.color.b = 0.45
        grid.color.a = 0.85

        y = self.y_min
        while y <= self.y_max + 1e-9:
            grid.points.append(self.make_point(y, self.z_min))
            grid.points.append(self.make_point(y, self.z_max))
            y += self.grid_step

        z = self.z_min
        while z <= self.z_max + 1e-9:
            grid.points.append(self.make_point(self.y_min, z))
            grid.points.append(self.make_point(self.y_max, z))
            z += self.grid_step

        markers.markers.append(grid)

        border = self.base_marker(1, "yz_joint_matrix_border", Marker.LINE_LIST)
        border.scale.x = 0.008
        border.color.r = 0.0
        border.color.g = 0.9
        border.color.b = 1.0
        border.color.a = 1.0

        corners = [
            (self.y_min, self.z_min), (self.y_max, self.z_min),
            (self.y_max, self.z_min), (self.y_max, self.z_max),
            (self.y_max, self.z_max), (self.y_min, self.z_max),
            (self.y_min, self.z_max), (self.y_min, self.z_min),
        ]

        for y, z in corners:
            border.points.append(self.make_point(y, z))

        markers.markers.append(border)

        panel = self.base_marker(2, "yz_joint_matrix_surface", Marker.CUBE)
        panel.pose.position.x = self.panel_x
        panel.pose.position.y = (self.y_min + self.y_max) / 2.0
        panel.pose.position.z = (self.z_min + self.z_max) / 2.0
        panel.scale.x = 0.015
        panel.scale.y = abs(self.y_max - self.y_min)
        panel.scale.z = abs(self.z_max - self.z_min)
        panel.color.r = 0.0
        panel.color.g = 0.8
        panel.color.b = 1.0
        panel.color.a = 0.07
        markers.markers.append(panel)

        target = self.base_marker(3, "moveit_target_ball", Marker.SPHERE)
        target.pose.position.x = self.panel_x - 0.04
        target.pose.position.y = self.target_y
        target.pose.position.z = self.target_z
        target.scale.x = 0.12
        target.scale.y = 0.12
        target.scale.z = 0.12
        target.color.r = 0.0
        target.color.g = 0.95
        target.color.b = 1.0
        target.color.a = 1.0
        markers.markers.append(target)

        cross = self.base_marker(4, "moveit_crosshair", Marker.LINE_LIST)
        cross.scale.x = 0.004
        cross.color.r = 0.0
        cross.color.g = 0.95
        cross.color.b = 1.0
        cross.color.a = 1.0
        cross.points.append(self.make_point(self.y_min, self.target_z))
        cross.points.append(self.make_point(self.y_max, self.target_z))
        cross.points.append(self.make_point(self.target_y, self.z_min))
        cross.points.append(self.make_point(self.target_y, self.z_max))
        markers.markers.append(cross)

        # Final validated mapping:
        # Top -> joint_1_y positive
        # Bottom -> joint_1_y negative
        # Left -> joint_2_z positive
        # Right -> joint_2_z negative
        q1 = self.map_value(self.target_z, self.z_min, self.z_max, self.q1_min, self.q1_max)
        q2 = self.map_value(self.target_y, self.y_min, self.y_max, self.q2_max, self.q2_min)

        info = self.base_marker(5, "workspace_info", Marker.TEXT_VIEW_FACING)
        info.pose.position.x = self.panel_x - 0.03
        info.pose.position.y = self.y_min - 0.13
        info.pose.position.z = self.z_max - 0.05
        info.scale.z = 0.032
        info.color.r = 1.0
        info.color.g = 1.0
        info.color.b = 1.0
        info.color.a = 1.0
        info.text = (
            f"joint_1_y {q1:.2f} rad\n"
            f"joint_2_z {q2:.2f} rad\n"
            "input Publish Point\n"
            "mode MoveIt2 planner\n"
            "action plan and execute"
        )
        markers.markers.append(info)

        self.marker_pub.publish(markers)

    def clicked_point_callback(self, msg):
        y = msg.point.y
        z = msg.point.z

        if not self.point_is_matrix(y, z):
            self.get_logger().warn(
                f"Click ignored: y={y:.3f}, z={z:.3f}. Click inside the matrix."
            )
            return

        y = max(self.y_min, min(self.y_max, y))
        z = max(self.z_min, min(self.z_max, z))

        self.target_y = y
        self.target_z = z

        q1 = self.map_value(z, self.z_min, self.z_max, self.q1_min, self.q1_max)
        q2 = self.map_value(y, self.y_min, self.y_max, self.q2_max, self.q2_min)

        self.publish_matrix()

        self.get_logger().info(
            f"MOVEIT TARGET: y={y:.3f}, z={z:.3f} -> "
            f"joint_1_y={q1:.3f}, joint_2_z={q2:.3f}"
        )

        self.send_moveit_goal(q1, q2)

    def make_joint_constraint(self, joint_name, position):
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = float(position)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        return jc

    def send_moveit_goal(self, q1, q2):
        if not self.moveit_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("/move_action not available. MoveIt2 move_group is not ready.")
            return

        goal = MoveGroup.Goal()

        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20

        # Let MoveIt use the current monitored robot state as start state
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "matrix_joint_goal"
        constraints.joint_constraints.append(
            self.make_joint_constraint(self.joint_1_name, q1)
        )
        constraints.joint_constraints.append(
            self.make_joint_constraint(self.joint_2_name, q2)
        )

        goal.request.goal_constraints.append(constraints)

        # plan_only=False means MoveIt plans and executes through ros2_control
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info("Sending target to MoveIt2 planner...")
        future = self.moveit_client.send_goal_async(goal)
        future.add_done_callback(self.moveit_goal_response_callback)

    def moveit_goal_response_callback(self, future):
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("MoveIt2 rejected the goal.")
            return

        self.get_logger().info("MoveIt2 accepted the goal. Planning/executing...")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.moveit_result_callback)

    def moveit_result_callback(self, future):
        result = future.result().result
        code = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                f"MoveIt2 SUCCESS. Planning time: {result.planning_time:.3f} s"
            )
        else:
            self.get_logger().error(
                f"MoveIt2 failed. Error code: {code}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Workspace2DInteractiveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
