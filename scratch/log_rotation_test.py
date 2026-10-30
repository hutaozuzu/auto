#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf2_ros
import math
import time
import threading

class RotationTestLogger(Node):
    def __init__(self):
        super().__init__('rotation_test_logger')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 50)

        self.cmd_wz = 0.0
        self.odom_yaw0 = None
        self.last_odom_yaw = 0.0
        self.ekf_yaw0 = None
        self.last_ekf_yaw = 0.0
        self.tf_yaw0 = None
        self.last_tf_yaw = 0.0
        self.imu_integrated_yaw = 0.0
        self.last_imu_t = None
        self.lock = threading.Lock()

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd_wz = msg.angular.z

    def odom_cb(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self.lock:
            if self.odom_yaw0 is None:
                self.odom_yaw0 = yaw
            self.last_odom_yaw = yaw - self.odom_yaw0

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        with self.lock:
            if self.last_imu_t is not None:
                dt = t - self.last_imu_t
                if 0.0 < dt < 0.5:
                    self.imu_integrated_yaw += wz * dt
            self.last_imu_t = t

    def ekf_cb(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self.lock:
            if self.ekf_yaw0 is None:
                self.ekf_yaw0 = yaw
            self.last_ekf_yaw = yaw - self.ekf_yaw0

    def update_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            with self.lock:
                if self.tf_yaw0 is None:
                    self.tf_yaw0 = yaw
                self.last_tf_yaw = yaw - self.tf_yaw0
        except Exception:
            pass

def main():
    rclpy.init()
    node = RotationTestLogger()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("==========================================================================")
    print("MANDATORY ROTATION LOGGER (C1 / C2) — RELATIVE YAW FROM T0")
    print("==========================================================================")
    print("Time   | Cmd Wz (rad/s) | IMU Yaw (deg) | Odom Yaw (deg) | EKF Yaw (deg) | TF Yaw (deg)")
    print("--------------------------------------------------------------------------")

    try:
        while rclpy.ok():
            node.update_tf()
            with node.lock:
                cw = node.cmd_wz
                iy = math.degrees(node.imu_integrated_yaw)
                oy = math.degrees(node.last_odom_yaw)
                ey = math.degrees(node.last_ekf_yaw)
                ty = math.degrees(node.last_tf_yaw)
            print(f"{time.strftime('%H:%M:%S')} | Cmd: {cw:+6.2f} rad/s | IMU: {iy:+7.2f}° | Odom: {oy:+7.2f}° | EKF: {ey:+7.2f}° | TF: {ty:+7.2f}°")
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
