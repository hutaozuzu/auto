#!/usr/bin/env python3
"""
stationary_amcl_update_node.py
Host-side helper node to automatically trigger a single AMCL no-motion update
whenever the robot remains physically stationary for a given duration (e.g. 3.0s).
Re-arms only after actual motion occurs. Zero service spam.
Uses a robust 0.5s sliding window to calculate true physical displacement rate.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from collections import deque
import math


class StationaryAmclUpdateNode(Node):
    def __init__(self):
        super().__init__('stationary_amcl_update_node')

        # Declare parameters
        self.declare_parameter('stationary_duration', 3.0)       # seconds of quiet before trigger
        self.declare_parameter('linear_threshold', 0.020)        # m/s max displacement rate when stationary
        self.declare_parameter('angular_threshold', 0.035)       # rad/s max orientation rate when stationary
        self.declare_parameter('rearm_linear_threshold', 0.045)  # m/s to qualify as new motion
        self.declare_parameter('rearm_angular_threshold', 0.080) # rad/s to qualify as new motion
        self.declare_parameter('window_duration', 0.50)          # seconds for displacement window
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('use_cmd_vel_gate', True)

        self.stationary_duration = self.get_parameter('stationary_duration').value
        self.linear_thresh = self.get_parameter('linear_threshold').value
        self.angular_thresh = self.get_parameter('angular_threshold').value
        self.rearm_lin_thresh = self.get_parameter('rearm_linear_threshold').value
        self.rearm_ang_thresh = self.get_parameter('rearm_angular_threshold').value
        self.window_dur = self.get_parameter('window_duration').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.use_cmd_vel_gate = self.get_parameter('use_cmd_vel_gate').value

        # Pose window deque: tuples of (time, x, y, yaw)
        self.pose_history = deque()

        # State tracking
        self.stationary_start_time = None
        self.update_sent_for_current_stop = False
        self.last_cmd_vel_zero = True
        self.total_service_calls = 0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        if self.use_cmd_vel_gate:
            self.cmd_sub = self.create_subscription(
                Twist,
                cmd_vel_topic,
                self.cmd_vel_callback,
                10
            )

        # Service client for AMCL no-motion update
        self.cli = self.create_client(Empty, '/request_nomotion_update')

        self.get_logger().info(
            f'Stationary AMCL Update Node initialized. '
            f'Duration: {self.stationary_duration}s, '
            f'LinThresh: {self.linear_thresh}m/s, AngThresh: {self.angular_thresh}rad/s'
        )

    def cmd_vel_callback(self, msg: Twist):
        lin_cmd = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        ang_cmd = abs(msg.angular.z)
        self.last_cmd_vel_zero = (lin_cmd < 0.005 and ang_cmd < 0.005)

    def get_yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

        self.pose_history.append((current_time, x, y, yaw))

        # Pop items older than window_duration
        while self.pose_history and (current_time - self.pose_history[0][0]) > self.window_dur:
            oldest = self.pose_history.popleft()

        if len(self.pose_history) < 3:
            return

        t0, x0, y0, yaw0 = self.pose_history[0]
        dt = current_time - t0
        if dt < 0.20:
            return

        # True window displacement rates
        dist = math.sqrt((x - x0)**2 + (y - y0)**2)
        lin_vel = dist / dt

        dyaw = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
        ang_vel = dyaw / dt

        # Check if robot is physically stationary
        measured_quiet = (lin_vel < self.linear_thresh and ang_vel < self.angular_thresh)
        cmd_quiet = self.last_cmd_vel_zero if self.use_cmd_vel_gate else True

        currently_stationary = measured_quiet and cmd_quiet

        if currently_stationary:
            if self.stationary_start_time is None:
                self.stationary_start_time = current_time

            elapsed = current_time - self.stationary_start_time

            # If stopped long enough and update has not been sent yet
            if elapsed >= self.stationary_duration and not self.update_sent_for_current_stop:
                self.send_nomotion_update()
                self.update_sent_for_current_stop = True

        else:
            # Robot is moving or displacement exceeds stationary bounds
            self.stationary_start_time = None

            # Check if motion is significant enough to RE-ARM
            if lin_vel > self.rearm_lin_thresh or ang_vel > self.rearm_ang_thresh:
                if self.update_sent_for_current_stop:
                    self.update_sent_for_current_stop = False
                    self.get_logger().info(
                        f'ROBOT MOVED (v={lin_vel:.3f}m/s, w={ang_vel:.3f}rad/s) — REARMED for next stop.'
                    )

    def send_nomotion_update(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn(
                '/request_nomotion_update service not ready. Skipping update.'
            )
            return

        req = Empty.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            future.result()
            self.total_service_calls += 1
            self.get_logger().info(
                f'STATIONARY DETECTED ({self.stationary_duration}s) -> '
                f'AMCL NO-MOTION UPDATE SENT (Total Calls: {self.total_service_calls}).'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = StationaryAmclUpdateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
