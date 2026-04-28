#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


class TrajectoryGateway(Node):
    def __init__(self):
        super().__init__('trajectory_gateway')

        self.declare_parameter('input_action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('output_action', '/hardware_arm_controller/follow_joint_trajectory')
        self.declare_parameter('minimum_dt', 0.35)

        self.input_action = self.get_parameter('input_action').value
        self.output_action = self.get_parameter('output_action').value
        self.minimum_dt = float(self.get_parameter('minimum_dt').value)

        self.client = ActionClient(self, FollowJointTrajectory, self.output_action)
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            self.input_action,
            self.execute_callback
        )

        self.get_logger().info(f'Gateway active: {self.input_action} -> {self.output_action}')
        self.get_logger().info(f'Minimum dt per point: {self.minimum_dt:.3f} s')

    def duration_to_sec(self, d):
        return float(d.sec) + float(d.nanosec) * 1e-9

    def sec_to_duration(self, value):
        d = Duration()
        d.sec = int(value)
        d.nanosec = int((value - int(value)) * 1e9)
        return d

    def retime_trajectory(self, traj):
        fixed = copy.deepcopy(traj)

        if len(fixed.points) == 0:
            return fixed

        last_t = -1.0

        for i, point in enumerate(fixed.points):
            current_t = self.duration_to_sec(point.time_from_start)

            if i == 0:
                if current_t <= 0.0:
                    current_t = self.minimum_dt
            else:
                min_allowed = last_t + self.minimum_dt
                if current_t <= last_t:
                    current_t = min_allowed

            point.time_from_start = self.sec_to_duration(current_t)
            last_t = current_t

        return fixed

    async def execute_callback(self, goal_handle):
        incoming = goal_handle.request.trajectory
        fixed_traj = self.retime_trajectory(incoming)

        self.get_logger().info(
            f'Received trajectory with {len(incoming.points)} points. '
            f'Forwarding retimed trajectory.'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Output action not available: {self.output_action}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Hardware trajectory controller action not available'
            return result

        new_goal = FollowJointTrajectory.Goal()
        new_goal.trajectory = fixed_traj
        new_goal.path_tolerance = goal_handle.request.path_tolerance
        new_goal.goal_tolerance = goal_handle.request.goal_tolerance
        new_goal.goal_time_tolerance = goal_handle.request.goal_time_tolerance

        send_future = self.client.send_goal_async(new_goal)
        hardware_goal = await send_future

        if not hardware_goal.accepted:
            self.get_logger().error('Hardware controller rejected retimed trajectory')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Hardware controller rejected retimed trajectory'
            return result

        # Do not wait here, because MoveIt may timeout based on its original bad timing.
        # Once the hardware controller accepts the retimed trajectory, report success to MoveIt.
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Trajectory accepted and retimed by gateway'
        return result


def main():
    rclpy.init()
    node = TrajectoryGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
