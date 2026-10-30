#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import tf2_ros
import rclpy.time
import rclpy.duration
import time
import numpy as np
import threading
import subprocess
import json

class FullAuditNode(Node):
    def __init__(self):
        super().__init__('full_audit_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.scans = []
        self.odoms = []
        self.imus = []
        self.lock = threading.Lock()
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 100)

    def scan_cb(self, msg):
        host_now = self.get_clock().now()
        host_ns = host_now.nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        offset_ms = (host_ns - stamp_ns) / 1e6
        
        # Test TF odom -> laser_frame at exact scan stamp
        tf_odom_laser_ok = False
        tf_odom_laser_err = ""
        tf_map_laser_ok = False
        tf_map_laser_err = ""
        try:
            target = rclpy.time.Time(nanoseconds=stamp_ns)
            self.tf_buffer.lookup_transform('odom', 'laser_frame', target, timeout=rclpy.duration.Duration(seconds=0.05))
            tf_odom_laser_ok = True
        except Exception as e:
            tf_odom_laser_err = str(e)
            
        try:
            target = rclpy.time.Time(nanoseconds=stamp_ns)
            self.tf_buffer.lookup_transform('map', 'laser_frame', target, timeout=rclpy.duration.Duration(seconds=0.05))
            tf_map_laser_ok = True
        except Exception as e:
            tf_map_laser_err = str(e)

        with self.lock:
            self.scans.append({
                'host_ns': host_ns,
                'stamp_ns': stamp_ns,
                'offset_ms': offset_ms,
                'frame_id': msg.header.frame_id,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'scan_time': msg.scan_time,
                'time_increment': msg.time_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges_len': len(msg.ranges),
                'valid_ranges': sum(1 for r in msg.ranges if np.isfinite(r) and msg.range_min <= r <= msg.range_max),
                'tf_odom_laser_ok': tf_odom_laser_ok,
                'tf_odom_laser_err': tf_odom_laser_err,
                'tf_map_laser_ok': tf_map_laser_ok,
                'tf_map_laser_err': tf_map_laser_err,
            })

    def odom_cb(self, msg):
        host_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        with self.lock:
            self.odoms.append({
                'host_ns': host_ns,
                'stamp_ns': stamp_ns,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'qz': msg.pose.pose.orientation.z,
                'qw': msg.pose.pose.orientation.w,
                'vx': msg.twist.twist.linear.x,
                'wz': msg.twist.twist.angular.z,
            })

    def imu_cb(self, msg):
        host_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        with self.lock:
            self.imus.append({
                'host_ns': host_ns,
                'stamp_ns': stamp_ns,
                'wz': msg.angular_velocity.z,
            })

def main():
    rclpy.init()
    node = FullAuditNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(">> Listening for /scan, /odom, /imu for 10 seconds...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        with node.lock:
            n_scan = len(node.scans)
            n_odom = len(node.odoms)
            n_imu = len(node.imus)
        print(f"   Collected: scans={n_scan}, odoms={n_odom}, imus={n_imu}", end="\r")
        time.sleep(0.5)
    print()

    with node.lock:
        scans = list(node.scans)
        odoms = list(node.odoms)
        imus = list(node.imus)

    print("\n========================================================")
    print("TELEMETRY AND TIMING AUDIT REPORT")
    print("========================================================")
    print(f"Total scans: {len(scans)}, odoms: {len(odoms)}, imus: {len(imus)}")

    if scans:
        offsets = [s['offset_ms'] for s in scans]
        print(f"\n--- SCAN GEOMETRY & FRAMES ---")
        s0 = scans[-1]
        print(f"frame_id:        {s0['frame_id']}")
        print(f"angle_min:       {s0['angle_min']:.4f} rad ({s0['angle_min']*180/np.pi:.2f} deg)")
        print(f"angle_max:       {s0['angle_max']:.4f} rad ({s0['angle_max']*180/np.pi:.2f} deg)")
        print(f"angle_increment: {s0['angle_increment']:.6f} rad ({s0['angle_increment']*180/np.pi:.4f} deg)")
        print(f"scan_time:       {s0['scan_time']*1000:.2f} ms ({1.0/s0['scan_time']:.2f} Hz)")
        print(f"time_increment:  {s0['time_increment']*1e6:.2f} us/bin")
        print(f"ranges bins:     {s0['ranges_len']}")
        print(f"valid ranges:    {s0['valid_ranges']}")

        print(f"\n--- HOST vs SCAN TIMESTAMP OFFSET (N={len(offsets)}) ---")
        print(f"Mean offset:     {np.mean(offsets):+.2f} ms")
        print(f"Median offset:   {np.median(offsets):+.2f} ms")
        print(f"Min offset:      {np.min(offsets):+.2f} ms")
        print(f"Max offset:      {np.max(offsets):+.2f} ms")
        print(f"Std dev:         {np.std(offsets):.2f} ms")
        print(f"P-P jitter:      {np.max(offsets)-np.min(offsets):.2f} ms")

        # TF lookup success
        tf_odom_ok_cnt = sum(1 for s in scans if s['tf_odom_laser_ok'])
        print(f"\n--- TF AT SCAN STAMP (odom -> laser_frame) ---")
        print(f"Success count:   {tf_odom_ok_cnt}/{len(scans)} ({tf_odom_ok_cnt/len(scans)*100:.1f}%)")
        if tf_odom_ok_cnt < len(scans):
            errs = [s['tf_odom_laser_err'] for s in scans if not s['tf_odom_laser_ok']]
            print(f"Sample error:    {errs[0] if errs else 'None'}")

    if odoms:
        odom_hz = len(odoms) / 10.0
        print(f"\n--- ODOM RATE ---")
        print(f"Actual /odom Hz: {odom_hz:.2f} Hz")

    if imus:
        imu_hz = len(imus) / 10.0
        print(f"\n--- IMU RATE ---")
        print(f"Actual /imu Hz:  {imu_hz:.2f} Hz")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
