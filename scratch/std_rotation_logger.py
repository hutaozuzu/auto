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
import numpy as np

class StandardizedRotationLogger(Node):
    def __init__(self, test_name="TEST"):
        super().__init__('std_rot_logger')
        self.test_name = test_name
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 50)

        self.records = []
        self.lock = threading.Lock()

        self.last_imu_t = None
        self.imu_cum_yaw = 0.0

        self.t0 = None
        self.odom_yaw0 = None
        self.ekf_yaw0 = None
        self.tf_yaw0 = None
        self.imu_yaw0 = None

    def unwrap_yaw(self, qz, qw):
        return 2.0 * math.atan2(qz, qw)

    def cmd_cb(self, msg):
        pass

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        with self.lock:
            if self.last_imu_t is not None:
                dt = t - self.last_imu_t
                if 0.0 < dt < 0.5:
                    self.imu_cum_yaw += wz * dt
            self.last_imu_t = t
            if self.imu_yaw0 is None:
                self.imu_yaw0 = self.imu_cum_yaw

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = self.unwrap_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        vyaw = msg.twist.twist.angular.z
        with self.lock:
            if self.odom_yaw0 is None:
                self.odom_yaw0 = yaw
            self.last_odom_yaw = yaw
            self.last_odom_vyaw = vyaw

    def ekf_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = self.unwrap_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        with self.lock:
            if self.ekf_yaw0 is None:
                self.ekf_yaw0 = yaw
            self.last_ekf_yaw = yaw

    def sample_all(self):
        t_now = time.time()
        with self.lock:
            if self.odom_yaw0 is None or self.ekf_yaw0 is None or self.imu_yaw0 is None:
                return None
            
            d_imu = math.degrees(self.imu_cum_yaw - self.imu_yaw0)
            d_odom = math.degrees(self.last_odom_yaw - self.odom_yaw0)
            d_ekf = math.degrees(self.last_ekf_yaw - self.ekf_yaw0)
            vyaw_raw = self.last_odom_vyaw

        tf_yaw = 0.0
        try:
            tr = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            yaw = self.unwrap_yaw(tr.transform.rotation.z, tr.transform.rotation.w)
            with self.lock:
                if self.tf_yaw0 is None:
                    self.tf_yaw0 = yaw
                tf_yaw = math.degrees(yaw - self.tf_yaw0)
        except Exception:
            pass

        return {
            't': t_now,
            'd_imu': d_imu,
            'd_odom': d_odom,
            'd_ekf': d_ekf,
            'd_tf': tf_yaw,
            'vyaw_raw': vyaw_raw
        }

def main():
    import sys
    test_label = sys.argv[1] if len(sys.argv) > 1 else "TEST"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    rclpy.init()
    node = StandardizedRotationLogger(test_label)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f"\n>> Waiting 2s for initial T0 alignment ({test_label})...")
    time.sleep(2.0)
    with node.lock:
        node.imu_yaw0 = node.imu_cum_yaw
        node.odom_yaw0 = None
        node.ekf_yaw0 = None
        node.tf_yaw0 = None

    time.sleep(0.5)

    print(f"==========================================================================")
    print(f"STANDARDIZED ROTATION LOG ({test_label}) — WINDOW: {duration}s")
    print(f"==========================================================================")
    print(f"Time     | IMU dYaw (deg) | Odom dYaw (deg) | EKF dYaw (deg) | TF dYaw (deg) | Odom raw vyaw")
    print(f"--------------------------------------------------------------------------")

    samples = []
    t_start = time.time()
    while time.time() - t_start < duration:
        s = node.sample_all()
        if s:
            samples.append(s)
            print(f"{time.strftime('%H:%M:%S')} | IMU: {s['d_imu']:+7.2f}° | Odom: {s['d_odom']:+7.2f}° | EKF: {s['d_ekf']:+7.2f}° | TF: {s['d_tf']:+7.2f}° | vyaw: {s['vyaw_raw']:+6.2f}")
        time.sleep(0.3)

    if samples:
        s_end = samples[-1]
        print(f"\n==========================================================================")
        print(f"FINAL DELTA SUMMARY FOR {test_label}:")
        print(f"  IMU delta Yaw:  {s_end['d_imu']:+7.2f}°")
        print(f"  Odom delta Yaw: {s_end['d_odom']:+7.2f}°")
        print(f"  EKF delta Yaw:  {s_end['d_ekf']:+7.2f}°")
        print(f"  TF delta Yaw:   {s_end['d_tf']:+7.2f}°")
        print(f"==========================================================================")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
