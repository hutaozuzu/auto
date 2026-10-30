#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time
import threading

class OdomImuMonitor(Node):
    def __init__(self):
        super().__init__('odom_imu_monitor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 50)

        self.last_odom_yaw = 0.0
        self.last_ekf_yaw = 0.0
        self.last_tf_yaw = 0.0
        self.imu_integrated_yaw = 0.0
        self.last_imu_time = None
        self.lock = threading.Lock()

    def odom_cb(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self.lock:
            self.last_odom_yaw = yaw

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        with self.lock:
            if self.last_imu_time is not None:
                dt = t - self.last_imu_time
                if 0.0 < dt < 0.5:
                    self.imu_integrated_yaw += wz * dt
            self.last_imu_time = t

    def ekf_cb(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self.lock:
            self.last_ekf_yaw = yaw

    def check_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            with self.lock:
                self.last_tf_yaw = yaw
        except Exception:
            pass

def main():
    rclpy.init()
    node = OdomImuMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("=====================================================================")
    print("LIVE ODOM / IMU / EKF / TF YAW MONITOR (SLAM OFF)")
    print("=====================================================================")
    print("Time | Odom Yaw (deg) | IMU Int Yaw (deg) | EKF Yaw (deg) | TF Yaw (deg)")
    print("---------------------------------------------------------------------")

    for _ in range(15):
        node.check_tf()
        with node.lock:
            oy = math.degrees(node.last_odom_yaw)
            iy = math.degrees(node.imu_integrated_yaw)
            ey = math.degrees(node.last_ekf_yaw)
            ty = math.degrees(node.last_tf_yaw)
        print(f"{time.strftime('%H:%M:%S')} | Odom: {oy:+7.2f}° | IMU: {iy:+7.2f}° | EKF: {ey:+7.2f}° | TF: {ty:+7.2f}°")
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
