import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import time
import math
import numpy as np

WHEEL_RADIUS_M = 0.0325
L_PHYSICAL_M = 0.110
WHEEL_TRACK_HALF = L_PHYSICAL_M / 2.0

# Tuned Kinematic Controller Parameters (Phase K2A.1)
K1 = 0.8
K2 = 100.0
K3 = 25.0
V_NOMINAL = 0.05
V_MAX = 0.06
OMEGA_MAX = 0.60

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def compute_kinematic_errors(x, y, theta, xr, yr, thetar):
    dx = xr - x
    dy = yr - y
    e1 = math.cos(theta) * dx + math.sin(theta) * dy
    e2 = -math.sin(theta) * dx + math.cos(theta) * dy
    e3 = wrap_to_pi(thetar - theta)
    return e1, e2, e3

def compute_control_law(e1, e2, e3, vr=V_NOMINAL, wr=0.0, k1=K1, k2=K2, k3=K3, v_max=V_MAX, omega_max=OMEGA_MAX):
    # Tracking law from Plan C / K1 specification
    v_raw = vr * math.cos(e3) + k1 * e1
    omega_raw = wr + k2 * vr * e2 + k3 * vr * math.sin(e3)
    
    # Saturation bounds
    v_cmd = max(-v_max, min(v_max, v_raw))
    omega_cmd = max(-omega_max, min(omega_max, omega_raw))
    
    # Forward-only preference for normal path tracking
    if v_cmd < 0.0:
        v_cmd = 0.0
        
    # Inverse kinematics
    v_l = v_cmd - WHEEL_TRACK_HALF * omega_cmd
    v_r = v_cmd + WHEEL_TRACK_HALF * omega_cmd
    rpm_l = (v_l / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    
    return v_raw, omega_raw, v_cmd, omega_cmd, rpm_l, rpm_r

class PhaseK2A1ExecutorNode(Node):
    def __init__(self):
        super().__init__('phase_k2a1_executor')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.sub_rpm_l = self.create_subscription(Float32, '/wheel_rpm_left', self.rpm_l_cb, 10)
        self.sub_rpm_r = self.create_subscription(Float32, '/wheel_rpm_right', self.rpm_r_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.last_gyro_z = 0.0
        self.last_rpm_l = 0.0
        self.last_rpm_r = 0.0
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0
        
        self.timestamps = []
        self.v_cmd_history = []
        self.omega_cmd_history = []
        self.gyro_z_history = []

    def imu_cb(self, msg):
        self.last_gyro_z = msg.angular_velocity.z

    def rpm_l_cb(self, msg):
        self.last_rpm_l = msg.data

    def rpm_r_cb(self, msg):
        self.last_rpm_r = msg.data

    def odom_cb(self, msg):
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.last_odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def run_k2a1_test(self, xr, yr, thetar, vr=V_NOMINAL, wr=0.0, duration_sec=0.8, rearm_pause_sec=2.0):
        self.stop_robot(rearm_pause_sec)
        
        self.timestamps.clear()
        self.v_cmd_history.clear()
        self.omega_cmd_history.clear()
        self.gyro_z_history.clear()
        
        rclpy.spin_once(self, timeout_sec=0.05)
        x0, y0, yaw0 = self.last_odom_x, self.last_odom_y, self.last_odom_yaw
        
        e1_0, e2_0, e3_0 = compute_kinematic_errors(0.0, 0.0, 0.0, xr, yr, thetar)
        
        start_time = time.time()
        prev_time = start_time
        integrated_yaw = 0.0
        max_v_seen = 0.0
        max_omega_seen = 0.0
        
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - prev_time
            prev_time = now
            
            integrated_yaw += self.last_gyro_z * dt
            
            curr_x = self.last_odom_x - x0
            curr_y = self.last_odom_y - y0
            curr_yaw = self.last_odom_yaw - yaw0
            
            e1, e2, e3 = compute_kinematic_errors(curr_x, curr_y, curr_yaw, xr, yr, thetar)
            
            v_raw, w_raw, v_cmd, w_cmd, rpm_l, rpm_r = compute_control_law(e1, e2, e3, vr, wr)
            max_v_seen = max(max_v_seen, abs(v_cmd))
            max_omega_seen = max(max_omega_seen, abs(w_cmd))
            
            twist = Twist()
            twist.linear.x = float(v_cmd)
            twist.angular.z = float(w_cmd)
            self.cmd_pub.publish(twist)
            
            self.timestamps.append(now - start_time)
            self.v_cmd_history.append(v_cmd)
            self.omega_cmd_history.append(w_cmd)
            self.gyro_z_history.append(self.last_gyro_z)
            time.sleep(0.02)
            
        self.stop_robot(0.5)
        rclpy.spin_once(self, timeout_sec=0.05)
        
        final_x = self.last_odom_x - x0
        final_y = self.last_odom_y - y0
        final_yaw = self.last_odom_yaw - yaw0
        
        e1_f, e2_f, e3_f = compute_kinematic_errors(final_x, final_y, final_yaw, xr, yr, thetar)
        
        delta_abs_e1 = abs(e1_0) - abs(e1_f)
        delta_abs_e2 = abs(e2_0) - abs(e2_f)
        delta_abs_e3 = abs(e3_0) - abs(e3_f)
        
        mean_gyro = float(np.mean(self.gyro_z_history)) if len(self.gyro_z_history) > 0 else 0.0
        
        return {
            'e1_0': e1_0, 'e2_0': e2_0, 'e3_0': e3_0,
            'e1_f': e1_f, 'e2_f': e2_f, 'e3_f': e3_f,
            'delta_abs_e1': delta_abs_e1,
            'delta_abs_e2': delta_abs_e2,
            'delta_abs_e3': delta_abs_e3,
            'max_v': max_v_seen,
            'max_omega': max_omega_seen,
            'mean_gyro': mean_gyro,
            'yaw_int_deg': math.degrees(integrated_yaw),
            'v_history': self.v_cmd_history,
            'w_history': self.omega_cmd_history
        }

    def stop_robot(self, pause_sec=0.5):
        zero_twist = Twist()
        t0 = time.time()
        while time.time() - t0 < pause_sec:
            self.cmd_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.04)
            time.sleep(0.02)

def main():
    rclpy.init()
    node = PhaseK2A1ExecutorNode()
    
    print('========================================================================')
    print('PLAN C — PHASE K2A.1 REAL-ROBOT GAIN TUNING & ANGULAR AUTHORITY CLOSURE')
    print('========================================================================\n')
    
    # 1. K3 GAIN ANALYSIS
    print('--- 1. K3 (HEADING GAIN) CANDIDATE ANALYSIS (vr=0.05 m/s, |e3|=0.25 rad) ---')
    for k3_cand in [5.0, 15.0, 25.0, 30.0]:
        omega_pred = k3_cand * 0.05 * math.sin(0.25)
        print(f"  k3 = {k3_cand:4.1f} -> Predicted omega = {omega_pred:+.3f} rad/s (Executability: {'STRONG' if omega_pred >= 0.25 else 'MODERATE' if omega_pred >= 0.15 else 'WEAK'})")
    print("  -> Selected K3 = 25.0 (Predicted omega = +0.309 rad/s)\n")

    # 2. K2 GAIN ANALYSIS
    print('--- 2. K2 (LATERAL GAIN) CANDIDATE ANALYSIS (vr=0.05 m/s, |e2|=0.05 m) ---')
    for k2_cand in [20.0, 60.0, 100.0, 140.0]:
        omega_pred = k2_cand * 0.05 * 0.05
        print(f"  k2 = {k2_cand:5.1f} -> Predicted omega = {omega_pred:+.3f} rad/s (Executability: {'STRONG' if omega_pred >= 0.25 else 'MODERATE' if omega_pred >= 0.15 else 'WEAK'})")
    print("  -> Selected K2 = 100.0 (Predicted omega = +0.250 rad/s)\n")

    # 3. PHYSICAL VALIDATION TEST SET (TEST A - G)
    print('--- 3. PHYSICAL VALIDATION TEST SET (K2A.1 FINAL GAINS) ---')
    tests = [
        ('TEST A — STRAIGHT (e1>0)', +0.15, 0.00, 0.00),
        ('TEST B — POSITIVE LATERAL (e2=+0.05m / Left)', +0.15, +0.05, 0.00),
        ('TEST C — NEGATIVE LATERAL (e2=-0.05m / Right)', +0.15, -0.05, 0.00),
        ('TEST D — POSITIVE HEADING (e3=+0.25rad / CCW)', +0.15, 0.00, +0.25),
        ('TEST E — NEGATIVE HEADING (e3=-0.25rad / CW)', +0.15, 0.00, -0.25),
        ('TEST F — COMBINED POSITIVE (e2=+0.04m, e3=+0.20rad)', +0.15, +0.04, +0.20),
        ('TEST G — COMBINED NEGATIVE (e2=-0.04m, e3=-0.20rad)', +0.15, -0.04, -0.20)
    ]
    
    all_v_cmds = []
    all_w_cmds = []
    
    for label, xr, yr, thetar in tests:
        print(f">> Executing {label} ...")
        res = node.run_k2a1_test(xr, yr, thetar, duration_sec=0.8, rearm_pause_sec=2.0)
        all_v_cmds.extend(res['v_history'])
        all_w_cmds.extend(res['w_history'])
        
        print(f"   Initial Errors: e1={res['e1_0']:+.3f}m, e2={res['e2_0']:+.3f}m, e3={res['e3_0']:+.3f}rad")
        print(f"   Final Errors:   e1={res['e1_f']:+.3f}m, e2={res['e2_f']:+.3f}m, e3={res['e3_f']:+.3f}rad")
        print(f"   Error Reductions: Delta|e1|={res['delta_abs_e1']:+.3f}m | Delta|e2|={res['delta_abs_e2']:+.3f}m | Delta|e3|={res['delta_abs_e3']:+.3f}rad")
        print(f"   Kinematics: Max v={res['max_v']:.3f} m/s | Max omega={res['max_omega']:+.3f} rad/s | GyroMean={res['mean_gyro']:+.3f} rad/s | Yaw={res['yaw_int_deg']:+.2f} deg")
        time.sleep(1.0)

    # 4. COMMAND DISTRIBUTION ANALYSIS
    v_arr = np.array(all_v_cmds)
    w_arr = np.array(all_w_cmds)
    total_samples = len(v_arr)
    
    pct_w_0 = float(np.sum(np.abs(w_arr) < 0.05) / total_samples * 100.0)
    pct_w_1 = float(np.sum((np.abs(w_arr) >= 0.05) & (np.abs(w_arr) < 0.10)) / total_samples * 100.0)
    pct_w_2 = float(np.sum((np.abs(w_arr) >= 0.10) & (np.abs(w_arr) < 0.20)) / total_samples * 100.0)
    pct_w_3 = float(np.sum((np.abs(w_arr) >= 0.20) & (np.abs(w_arr) < 0.30)) / total_samples * 100.0)
    pct_w_4 = float(np.sum(np.abs(w_arr) >= 0.30) / total_samples * 100.0)
    sat_rate = float(np.sum(np.abs(w_arr) >= OMEGA_MAX - 0.001) / total_samples * 100.0)
    
    print('\n--- 4. COMMAND DISTRIBUTION SUMMARY ---')
    print(f"  Total Controller Cycles Logged: {total_samples}")
    print(f"  |omega| < 0.05 rad/s:         {pct_w_0:.1f} %")
    print(f"  0.05 <= |omega| < 0.10 rad/s: {pct_w_1:.1f} %")
    print(f"  0.10 <= |omega| < 0.20 rad/s: {pct_w_2:.1f} %")
    print(f"  0.20 <= |omega| < 0.30 rad/s: {pct_w_3:.1f} %")
    print(f"  |omega| >= 0.30 rad/s:        {pct_w_4:.1f} %")
    print(f"  Omega Saturation Rate:        {sat_rate:.1f} %")

    # 5. WATCHDOG TEST
    print('\n--- 5. SAFETY REGRESSION CHECK ---')
    twist = Twist()
    twist.linear.x = 0.05
    node.cmd_pub.publish(twist)
    time.sleep(0.1)
    time.sleep(1.2)
    rclpy.spin_once(node, timeout_sec=0.2)
    watchdog_pass = (abs(node.last_rpm_l) < 1.0 and abs(node.last_rpm_r) < 1.0)
    print(f"  -> Watchdog Test Result: {'PASS' if watchdog_pass else 'FAIL'} (Measured RPM: L={node.last_rpm_l:.1f}, R={node.last_rpm_r:.1f})")

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
