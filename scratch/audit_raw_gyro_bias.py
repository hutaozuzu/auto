#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import numpy as np
import threading
import math

class GyroStationaryAuditor(Node):
    def __init__(self):
        super().__init__('gyro_stationary_auditor')
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 500)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 500)
        self.imus = []
        self.odoms = []
        self.lock = threading.Lock()

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        with self.lock:
            self.imus.append((t, wz))

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        vyaw = msg.twist.twist.angular.z
        with self.lock:
            self.odoms.append((t, yaw, vyaw))

def main():
    rclpy.init()
    node = GyroStationaryAuditor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(">> Recording raw /imu and /odom for 20 seconds (ROBOT MUST BE COMPLETELY STATIONARY)...")
    t0 = time.time()
    while time.time() - t0 < 20.0:
        with node.lock:
            ni = len(node.imus)
            no = len(node.odoms)
        print(f"   Elapsed: {time.time()-t0:.1f}s / 20.0s | IMU samples: {ni} | Odom samples: {no}", end="\r")
        time.sleep(0.2)
    print()

    with node.lock:
        imus = list(node.imus)
        odoms = list(node.odoms)

    node.destroy_node()
    rclpy.shutdown()

    if not imus:
        print("ERROR: No /imu data received!")
        return

    wz_rad = np.array([x[1] for x in imus])
    wz_deg = np.degrees(wz_rad)
    dt_arr = np.diff(np.array([x[0] for x in imus]))

    # Trapezoidal integration of raw gyro
    int_yaw_rad = 0.0
    for i in range(len(dt_arr)):
        if 0.0 < dt_arr[i] < 0.5:
            int_yaw_rad += 0.5 * (wz_rad[i] + wz_rad[i+1]) * dt_arr[i]
    int_yaw_deg = math.degrees(int_yaw_rad)

    print("\n========================================================")
    print("RAW GYRO STATIONARY BIAS REPORT (20-SECOND WINDOW)")
    print("========================================================")
    print(f"Total IMU samples:   {len(wz_rad)} (rate = {len(wz_rad)/20.0:.2f} Hz)")
    print(f"Mean wz:             {np.mean(wz_rad):+.6f} rad/s  ({np.mean(wz_deg):+.4f} deg/s)")
    print(f"Median wz:           {np.median(wz_rad):+.6f} rad/s  ({np.median(wz_deg):+.4f} deg/s)")
    print(f"Std dev wz:          {np.std(wz_rad):.6f} rad/s  ({np.std(wz_deg):.4f} deg/s)")
    print(f"Min wz:              {np.min(wz_rad):+.6f} rad/s  ({np.min(wz_deg):+.4f} deg/s)")
    print(f"Max wz:              {np.max(wz_rad):+.6f} rad/s  ({np.max(wz_deg):+.4f} deg/s)")
    print(f"Total integrated drift over 20s: {int_yaw_deg:+.3f} deg  ({int_yaw_deg/20.0:+.4f} deg/s)")
    
    if odoms:
        odom_yaws = [x[1] for x in odoms]
        odom_vyaws = [x[2] for x in odoms]
        print("\n--- WHEEL ODOMETRY DURING STATIONARY ---")
        print(f"Odom pose yaw range: {math.degrees(np.max(odom_yaws) - np.min(odom_yaws)):.4f} deg")
        print(f"Odom twist.angular.z mean: {np.mean(odom_vyaws):+.6f} rad/s, std: {np.std(odom_vyaws):.6f} rad/s")

if __name__ == '__main__':
    main()
