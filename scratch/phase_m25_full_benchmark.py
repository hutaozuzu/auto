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
WHEEL_SEPARATION_M = 0.110

def calc_theoretical_rpm(v, w, r=WHEEL_RADIUS_M, b=0.055):
    v_l = v - b * w
    v_r = v + b * w
    rpm_l = (v_l / (2.0 * math.pi * r)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * r)) * 60.0
    return rpm_l, rpm_r

def calc_omega_wheels(rpm_l_signed, rpm_r_signed, r=WHEEL_RADIUS_M, L=WHEEL_SEPARATION_M):
    w_l = (rpm_l_signed * 2.0 * math.pi) / 60.0
    w_r = (rpm_r_signed * 2.0 * math.pi) / 60.0
    return (r / L) * (w_r - w_l)

class DrivetrainBenchmarkM25Node(Node):
    def __init__(self):
        super().__init__('phase_m25_benchmark')
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
        
        self.gyro_z_history = []
        self.rpm_l_history = []
        self.rpm_r_history = []

    def imu_cb(self, msg):
        self.last_gyro_z = msg.angular_velocity.z

    def rpm_l_cb(self, msg):
        self.last_rpm_l = msg.data

    def rpm_r_cb(self, msg):
        self.last_rpm_r = msg.data

    def odom_cb(self, msg):
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y

    def run_motion_test(self, v_cmd, w_cmd, duration_sec=3.0, rearm_pause_sec=1.5):
        self.stop_robot(rearm_pause_sec)
        self.gyro_z_history.clear()
        self.rpm_l_history.clear()
        self.rpm_r_history.clear()
        
        start_odom_x = self.last_odom_x
        start_odom_y = self.last_odom_y
        
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        
        start_time = time.time()
        prev_time = start_time
        integrated_yaw = 0.0
        
        while time.time() - start_time < duration_sec:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.04)
            now = time.time()
            dt = now - prev_time
            prev_time = now
            integrated_yaw += self.last_gyro_z * dt
            self.gyro_z_history.append(self.last_gyro_z)
            self.rpm_l_history.append(self.last_rpm_l)
            self.rpm_r_history.append(self.last_rpm_r)
            time.sleep(0.01)
            
        self.stop_robot(0.5)
        disp = math.hypot(self.last_odom_x - start_odom_x, self.last_odom_y - start_odom_y)
        
        gz_arr = np.array(self.gyro_z_history) if self.gyro_z_history else np.array([0.0])
        rl_arr = np.array(self.rpm_l_history) if self.rpm_l_history else np.array([0.0])
        rr_arr = np.array(self.rpm_r_history) if self.rpm_r_history else np.array([0.0])
        
        gz_mean = float(np.mean(gz_arr))
        mean_rpm_l = float(np.mean(rl_arr))
        mean_rpm_r = float(np.mean(rr_arr))
        
        th_l, th_r = calc_theoretical_rpm(v_cmd, w_cmd)
        omega_wheels = calc_omega_wheels(mean_rpm_l, mean_rpm_r)
        
        delta_rpm_target = th_r - th_l
        delta_rpm_measured = mean_rpm_r - mean_rpm_l
        diff_tracking_ratio = (delta_rpm_measured / delta_rpm_target) if abs(delta_rpm_target) > 0.1 else 1.0
        
        g_omega = (omega_wheels / w_cmd) if abs(w_cmd) > 0.01 else 1.0
        g_gyro = (gz_mean / w_cmd) if abs(w_cmd) > 0.01 else 1.0
        
        is_stall = (abs(mean_rpm_l) < 3.0 and abs(th_l) > 8.0) or (abs(mean_rpm_r) < 3.0 and abs(th_r) > 8.0)
        
        sign_correct = True
        if abs(w_cmd) > 0.05:
            if (w_cmd > 0 and gz_mean < -0.02) or (w_cmd < 0 and gz_mean > 0.02):
                sign_correct = False
                
        result = "STALL" if is_stall else ("PASS" if sign_correct else "SIGN_MISMATCH")
        if not is_stall and sign_correct and (abs(mean_rpm_l) < 0.35 * abs(th_l) or abs(mean_rpm_r) < 0.35 * abs(th_r)):
            result = "PARTIAL"
            
        return {
            'v_cmd': v_cmd, 'w_cmd': w_cmd,
            'th_rpm_l': th_l, 'th_rpm_r': th_r,
            'mean_rpm_l': mean_rpm_l, 'mean_rpm_r': mean_rpm_r,
            'delta_rpm_target': delta_rpm_target, 'delta_rpm_measured': delta_rpm_measured,
            'diff_tracking_ratio': diff_tracking_ratio,
            'omega_cmd': w_cmd, 'omega_wheels': omega_wheels, 'gz_mean': gz_mean,
            'g_omega': g_omega, 'g_gyro': g_gyro,
            'yaw_deg': math.degrees(integrated_yaw), 'disp': disp,
            'result': result
        }

    def stop_robot(self, pause_sec=0.5):
        zero_twist = Twist()
        t0 = time.time()
        while time.time() - t0 < pause_sec:
            self.cmd_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)

def main():
    rclpy.init()
    node = DrivetrainBenchmarkM25Node()
    
    print('========================================================================')
    print('PLAN C — PHASE M2.5 FULL BENCHMARK & VELOCITY ESTIMATOR AUDIT')
    print('========================================================================\n')
    
    # 1. Floor Curve Tests A through F
    print('--- 1. FLOOR CURVE TESTS (A through F) ---')
    test_cases = [
        ('A', 0.10, +0.20),
        ('B', 0.10, -0.20),
        ('C', 0.10, +0.30),
        ('D', 0.10, -0.30),
        ('E', 0.10, +0.40),
        ('F', 0.10, -0.40),
    ]
    curve_results = {}
    for label, v, w in test_cases:
        print(f"\n>> Executing Case {label}: v={v:.2f} m/s, w={w:+.2f} rad/s ...")
        res = node.run_motion_test(v, w, duration_sec=3.0, rearm_pause_sec=1.5)
        curve_results[label] = res
        print(f"   -> Result: {res['result']}")
        print(f"   -> Target RPM: L={res['th_rpm_l']:+.1f}, R={res['th_rpm_r']:+.1f} | DeltaRPM: Target={res['delta_rpm_target']:+.1f}, Measured={res['delta_rpm_measured']:+.1f} (Ratio: {res['diff_tracking_ratio']:.2f})")
        print(f"   -> Measured RPM: L={res['mean_rpm_l']:+.1f}, R={res['mean_rpm_r']:+.1f}")
        print(f"   -> Omega: Cmd={res['omega_cmd']:+.3f}, Wheels={res['omega_wheels']:+.3f} (G_w={res['g_omega']:.2f}), GyroMean={res['gz_mean']:+.3f} rad/s (G_gyro={res['g_gyro']:.2f})")
        print(f"   -> Integrated Yaw: {res['yaw_deg']:+.2f} deg | Displacement: {res['disp']:.3f} m")

    # 2. Pure Rotation Tests
    print('\n--- 2. PURE ROTATION TESTS ---')
    rot_cases = [+0.60, -0.60, +0.80, -0.80]
    rot_results = {}
    for w in rot_cases:
        print(f"\n>> Executing Pure Spin w={w:+.2f} rad/s ...")
        res = node.run_motion_test(0.0, w, duration_sec=2.5, rearm_pause_sec=1.5)
        rot_results[w] = res
        print(f"   -> Result: {res['result']}")
        print(f"   -> Measured RPM: L={res['mean_rpm_l']:+.1f}, R={res['mean_rpm_r']:+.1f}")
        print(f"   -> Omega: Cmd={res['omega_cmd']:+.3f}, Wheels={res['omega_wheels']:+.3f} (G_w={res['g_omega']:.2f}), GyroMean={res['gz_mean']:+.3f} rad/s (G_gyro={res['g_gyro']:.2f})")
        print(f"   -> Integrated Yaw: {res['yaw_deg']:+.2f} deg")

    g_pos_60 = abs(rot_results[+0.60]['gz_mean'])
    g_neg_60 = abs(rot_results[-0.60]['gz_mean'])
    asym_60 = (abs(g_pos_60 - g_neg_60) / (0.5 * (g_pos_60 + g_neg_60))) * 100.0 if (g_pos_60 + g_neg_60) > 0 else 0.0

    g_pos_80 = abs(rot_results[+0.80]['gz_mean'])
    g_neg_80 = abs(rot_results[-0.80]['gz_mean'])
    asym_80 = (abs(g_pos_80 - g_neg_80) / (0.5 * (g_pos_80 + g_neg_80))) * 100.0 if (g_pos_80 + g_neg_80) > 0 else 0.0

    print('\n--- CW/CCW ASYMMETRY RESULTS ---')
    print(f"  Asymmetry @ +/-0.60 rad/s: {asym_60:.1f} % (CCW={g_pos_60:.3f}, CW={g_neg_60:.3f})")
    print(f"  Asymmetry @ +/-0.80 rad/s: {asym_80:.1f} % (CCW={g_pos_80:.3f}, CW={g_neg_80:.3f})")

    # 3. Watchdog Test
    print('\n--- 3. WATCHDOG TIMEOUT TEST ---')
    twist = Twist()
    twist.linear.x = 0.10
    node.cmd_pub.publish(twist)
    time.sleep(0.1)
    time.sleep(0.7)
    rclpy.spin_once(node, timeout_sec=0.1)
    watchdog_rpm_l = node.last_rpm_l
    watchdog_rpm_r = node.last_rpm_r
    watchdog_pass = (abs(watchdog_rpm_l) < 2.0 and abs(watchdog_rpm_r) < 2.0)
    print(f"  -> Watchdog Test Result: {'PASS' if watchdog_pass else 'FAIL'} (Measured RPM: L={watchdog_rpm_l:.1f}, R={watchdog_rpm_r:.1f})")

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
