#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SlamScanGateNode(Node):
    def __init__(self):
        super().__init__('slam_scan_gate_node')

        # Parameters
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_slam')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('angular_gate_threshold', 0.45)   # rad/s
        self.declare_parameter('linear_gate_threshold', 0.03)    # m/s
        self.declare_parameter('slam_resume_delay', 0.35)        # seconds settling delay
        self.declare_parameter('required_stable_scans', 2)       # fresh scans after settle
        self.declare_parameter('odom_omega_threshold', 0.12)     # rad/s physical settling threshold

        input_scan_topic = self.get_parameter('input_scan_topic').value
        output_scan_topic = self.get_parameter('output_scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.angular_gate_threshold = self.get_parameter('angular_gate_threshold').value
        self.linear_gate_threshold = self.get_parameter('linear_gate_threshold').value
        self.slam_resume_delay = self.get_parameter('slam_resume_delay').value
        self.required_stable_scans = self.get_parameter('required_stable_scans').value
        self.odom_omega_threshold = self.get_parameter('odom_omega_threshold').value

        # State Variables
        self.is_rotating = False
        self.is_gated = False
        self.settle_start_time = None
        self.stable_scan_count = 0
        self.current_odom_omega = 0.0
        self.total_scans_in = 0
        self.total_scans_out = 0
        self.total_scans_gated = 0

        # Publisher & Subscribers
        self.scan_pub = self.create_publisher(LaserScan, output_scan_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, input_scan_topic, self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'SlamScanGateNode initialized: {input_scan_topic} -> {output_scan_topic} '
            f'(gate: |omega| >= {self.angular_gate_threshold} rad/s, settle delay: {self.slam_resume_delay}s, '
            f'required stable scans: {self.required_stable_scans})'
        )

    def cmd_vel_callback(self, msg: Twist):
        abs_omega = abs(msg.angular.z)
        abs_v = abs(msg.linear.x)

        # Fast in-place rotation condition (ALIGN_TO_PATH / TERMINAL_ROTATE / ROTATE_RECOVERY)
        if abs_omega >= self.angular_gate_threshold and abs_v <= self.linear_gate_threshold:
            if not self.is_rotating:
                self.is_rotating = True
                self.is_gated = True
                self.settle_start_time = None
                self.stable_scan_count = 0
                self.get_logger().info(
                    f'Fast in-place rotation detected (omega={msg.angular.z:.2f} rad/s). GATING SLAM scans!'
                )
        else:
            if self.is_rotating:
                # Rotation command has stopped
                self.is_rotating = False
                self.settle_start_time = self.get_clock().now()
                self.stable_scan_count = 0
                self.get_logger().info(
                    f'Rotation command stopped. Settling timer started ({self.slam_resume_delay:.2f}s)...'
                )

    def odom_callback(self, msg: Odometry):
        self.current_odom_omega = abs(msg.twist.twist.angular.z)

    def scan_callback(self, msg: LaserScan):
        self.total_scans_in += 1
        now = self.get_clock().now()

        # Check un-gating / settling status
        if self.is_gated:
            if not self.is_rotating and self.settle_start_time is not None:
                elapsed_settle = (now - self.settle_start_time).nanoseconds / 1e9
                # Settle condition: timer expired AND physical robot angular velocity near zero
                if elapsed_settle >= self.slam_resume_delay and self.current_odom_omega <= self.odom_omega_threshold:
                    self.stable_scan_count += 1
                    if self.stable_scan_count >= self.required_stable_scans:
                        self.is_gated = False
                        self.settle_start_time = None
                        self.get_logger().info(
                            f'Robot settled (time={elapsed_settle:.2f}s, stable_scans={self.stable_scan_count}). '
                            f'RESUMING SLAM scans!'
                        )

        if self.is_gated:
            self.total_scans_gated += 1
            return  # Drop scan for SLAM Toolbox to prevent dynamic wall blur/duplication

        # Forward scan to SLAM Toolbox
        self.total_scans_out += 1
        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlamScanGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
