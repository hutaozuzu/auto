import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
import threading

WHEEL_RADIUS_M = 0.0325
L_PHYSICAL_M = 0.110
WHEEL_TRACK_HALF = L_PHYSICAL_M / 2.0

def calc_theoretical_rpm(v, w):
    v_l = v - WHEEL_TRACK_HALF * w
    v_r = v + WHEEL_TRACK_HALF * w
    rpm_l = (v_l / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    return rpm_l, rpm_r

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class AsymmetryAuditorNode(Node):
    def __init__(self):
        super().__init__('asymmetry_auditor_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 200)
        self.sub_rpm_l = self.create_subscription(Float32, '/wheel_rpm_left', self.rpm_l_cb, 100)
        self.sub_rpm_r = self.create_subscription(Float32, '/wheel_rpm_right', self.rpm_r_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        
        self.lock = threading.Lock()
        self.recording = False
        self.imu_records = []    # list of (header_t, arrival_t, gz)
        self.rpm_l_records = []  # list of (t, val)
        self.rpm_r_records = []  # list of (t, val)
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0
        self.odom_received = False

    def imu_cb(self, msg):
        header_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        arrival_t = time.time()
        gz = msg.angular_velocity.z
        with self.lock:
            if self.recording:
                self.imu_records.append((header_t, arrival_t, gz))

    def rpm_l_cb(self, msg):
        with self.lock:
            if self.recording:
                self.rpm_l_records.append((time.time(), msg.data))

    def rpm_r_cb(self, msg):
        with self.lock:
            if self.recording:
                self.rpm_r_records.append((time.time(), msg.data))

    def odom_cb(self, msg):
        with self.lock:
            self.last_odom_x = msg.pose.pose.position.x
            self.last_odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.last_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.odom_received = True

    def get_odom_pose(self):
        with self.lock:
            return self.last_odom_x, self.last_odom_y, self.last_odom_yaw

    def start_recording(self):
        with self.lock:
            self.imu_records.clear()
            self.rpm_l_records.clear()
            self.rpm_r_records.clear()
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            return list(self.imu_records), list(self.rpm_l_records), list(self.rpm_r_records)

def run_direct_cmd_test(node, v_cmd, w_cmd, duration_sec=0.9):
    # Stop / Rearm
    zero_twist = Twist()
    t_stop = time.time()
    while time.time() - t_stop < 1.0:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.2)
    x0, y0, yaw0 = node.get_odom_pose()
    
    twist = Twist()
    twist.linear.x = float(v_cmd)
    twist.angular.z = float(w_cmd)
    
    target_rpm_l, target_rpm_r = calc_theoretical_rpm(v_cmd, w_cmd)
    delta_rpm_target = target_rpm_r - target_rpm_l
    
    node.start_recording()
    t_start = time.time()
    
    while time.time() - t_start < duration_sec:
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
        
    t_end = time.time()
    imu_recs, rpm_l_recs, rpm_r_recs = node.stop_recording()
    
    # Safe stop
    t_stop = time.time()
    while time.time() - t_stop < 0.5:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    xf, yf, yawf = node.get_odom_pose()
    ekf_yaw_deg = math.degrees(wrap_to_pi(yawf - yaw0))
    
    dt_wall = t_end - t_start
    
    # Wheel RPM analysis
    act_rpm_l = [r[1] for r in rpm_l_recs] if rpm_l_recs else [0.0]
    act_rpm_r = [r[1] for r in rpm_r_recs] if rpm_r_recs else [0.0]
    
    mean_rpm_l = float(np.mean(act_rpm_l))
    mean_rpm_r = float(np.mean(act_rpm_r))
    delta_rpm_actual = mean_rpm_r - mean_rpm_l
    g_delta = (delta_rpm_actual / delta_rpm_target) if abs(delta_rpm_target) > 0.1 else 1.0
    
    # IMU Packet & Timing Analysis
    sample_count = len(imu_recs)
    
    if sample_count >= 2:
        header_ts = np.array([r[0] for r in imu_recs])
        arrival_ts = np.array([r[1] for r in imu_recs])
        gz_vals = np.array([r[2] for r in imu_recs])
        
        header_duration = float(header_ts[-1] - header_ts[0])
        arrival_duration = float(arrival_ts[-1] - arrival_ts[0])
        
        # Consecutive intervals between header timestamps
        dts = np.diff(header_ts)
        min_dt = float(np.min(dts)) if len(dts) > 0 else 0.0
        median_dt = float(np.median(dts)) if len(dts) > 0 else 0.0
        max_dt = float(np.max(dts)) if len(dts) > 0 else 0.0
        
        gaps_2x = int(np.sum(dts > 2.0 * median_dt)) if median_dt > 0 else 0
        gaps_3x = int(np.sum(dts > 3.0 * median_dt)) if median_dt > 0 else 0
        gaps_5x = int(np.sum(dts > 5.0 * median_dt)) if median_dt > 0 else 0
        
        arith_mean_gyro = float(np.mean(gz_vals))
        
        # Trapezoidal integration using header timestamps
        integrated_rad = float(np.trapz(gz_vals, header_ts))
        integrated_yaw_deg = math.degrees(integrated_rad)
        
        time_weighted_mean_gyro = integrated_rad / header_duration if header_duration > 0 else arith_mean_gyro
        
        min_gyro = float(np.min(gz_vals))
        max_gyro = float(np.max(gz_vals))
        median_gyro = float(np.median(gz_vals))
        raw_snippets = [f"{g:+.3f}" for g in gz_vals[:10]]
    else:
        header_duration, arrival_duration = 0.0, 0.0
        min_dt, median_dt, max_dt = 0.0, 0.0, 0.0
        gaps_2x, gaps_3x, gaps_5x = 0, 0, 0
        arith_mean_gyro, time_weighted_mean_gyro = 0.0, 0.0
        integrated_yaw_deg = 0.0
        min_gyro, max_gyro, median_gyro = 0.0, 0.0, 0.0
        raw_snippets = []
        
    return {
        'v_cmd': v_cmd, 'w_cmd': w_cmd,
        'dt_wall': dt_wall,
        'target_rpm_l': target_rpm_l, 'target_rpm_r': target_rpm_r,
        'mean_rpm_l': mean_rpm_l, 'mean_rpm_r': mean_rpm_r,
        'delta_rpm_target': delta_rpm_target, 'delta_rpm_actual': delta_rpm_actual,
        'g_delta': g_delta,
        'sample_count': sample_count,
        'header_duration': header_duration,
        'arrival_duration': arrival_duration,
        'min_dt': min_dt, 'median_dt': median_dt, 'max_dt': max_dt,
        'gaps_2x': gaps_2x, 'gaps_3x': gaps_3x, 'gaps_5x': gaps_5x,
        'arith_mean_gyro': arith_mean_gyro,
        'time_weighted_mean_gyro': time_weighted_mean_gyro,
        'integrated_yaw_deg': integrated_yaw_deg,
        'min_gyro': min_gyro, 'max_gyro': max_gyro, 'median_gyro': median_gyro,
        'raw_snippets': raw_snippets,
        'ekf_yaw_deg': ekf_yaw_deg
    }

def main():
    rclpy.init()
    node = AsymmetryAuditorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    time.sleep(1.0)
    
    print('========================================================================')
    print('PLAN C — QUICK K2A.3B CW/CCW ANGULAR ASYMMETRY AUDIT')
    print('========================================================================\n')
    
    # TEST A: +0.30 rad/s (CCW)
    print('>> Executing TEST A: v=0.08 m/s, w=+0.30 rad/s (CCW) ...')
    res_pos = run_direct_cmd_test(node, 0.08, +0.30, duration_sec=0.9)
    time.sleep(1.5)
    
    # TEST B: -0.30 rad/s (CW)
    print('>> Executing TEST B: v=0.08 m/s, w=-0.30 rad/s (CW) ...')
    res_neg = run_direct_cmd_test(node, 0.08, -0.30, duration_sec=0.9)
    time.sleep(1.0)
    
    print('\n========================================================================')
    print('DETAILED AUDIT RESULTS')
    print('========================================================================')
    
    for name, r in [('+0.30 CCW', res_pos), ('-0.30 CW', res_neg)]:
        print(f"\n--- {name} ---")
        print(f"  DT (Wall): {r['dt_wall']:.3f} s | Header Duration: {r['header_duration']:.3f} s | Arrival Duration: {r['arrival_duration']:.3f} s")
        print(f"  Wheel Target L/R: {r['target_rpm_l']:+.1f} / {r['target_rpm_r']:+.1f} RPM (DeltaTarget: {r['delta_rpm_target']:+.1f} RPM)")
        print(f"  Wheel Actual L/R: {r['mean_rpm_l']:+.1f} / {r['mean_rpm_r']:+.1f} RPM (DeltaActual: {r['delta_rpm_actual']:+.1f} RPM, G_delta: {r['g_delta']:.2f})")
        print(f"  IMU Samples: {r['sample_count']} | Median dt: {r['median_dt']*1000.0:.1f} ms | Max Gap: {r['max_dt']*1000.0:.1f} ms")
        print(f"  Gaps >2x: {r['gaps_2x']} | >3x: {r['gaps_3x']} | >5x: {r['gaps_5x']}")
        print(f"  Raw Snippet (first 10): {r['raw_snippets']}")
        print(f"  Gyro Arithmetic Mean: {r['arith_mean_gyro']:+.4f} rad/s")
        print(f"  Gyro Time-Weighted Mean: {r['time_weighted_mean_gyro']:+.4f} rad/s")
        print(f"  Gyro Min / Max / Median: {r['min_gyro']:+.4f} / {r['max_gyro']:+.4f} / {r['median_gyro']:+.4f} rad/s")
        print(f"  Integrated IMU Yaw: {r['integrated_yaw_deg']:+.2f} deg")
        print(f"  EKF Yaw: {r['ekf_yaw_deg']:+.2f} deg")
        
        # Consistency check: gyro_time_weighted * DT ≈ integrated_yaw
        pred_yaw_deg = math.degrees(r['time_weighted_mean_gyro'] * r['header_duration'])
        print(f"  Consistency Check: time_weighted_mean * DT = {pred_yaw_deg:+.2f} deg vs integrated = {r['integrated_yaw_deg']:+.2f} deg")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
