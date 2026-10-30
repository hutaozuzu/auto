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
WHEEL_DIAMETER_M = 0.065
L_PHYSICAL_M = 0.110
L_EFFECTIVE_M = 0.065
PPR = 38

def calc_theoretical_rpm(v, w, r=WHEEL_RADIUS_M, b=L_PHYSICAL_M/2.0):
    v_l = v - b * w
    v_r = v + b * w
    rpm_l = (v_l / (2.0 * math.pi * r)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * r)) * 60.0
    return rpm_l, rpm_r

def calc_omega_wheels(rpm_l_signed, rpm_r_signed, r=WHEEL_RADIUS_M, L=L_PHYSICAL_M):
    w_l = (rpm_l_signed * 2.0 * math.pi) / 60.0
    w_r = (rpm_r_signed * 2.0 * math.pi) / 60.0
    return (r / L) * (w_r - w_l)

class KinematicCalibratorM26CNode(Node):
    def __init__(self):
        super().__init__('phase_m26c_calibrator')
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
        
        self.timestamps = []
        self.gyro_z_history = []
        self.rpm_l_history = []
        self.rpm_r_history = []
        self.odom_x_history = []
        self.odom_y_history = []

    def imu_cb(self, msg):
        self.last_gyro_z = msg.angular_velocity.z

    def rpm_l_cb(self, msg):
        self.last_rpm_l = msg.data

    def rpm_r_cb(self, msg):
        self.last_rpm_r = msg.data

    def odom_cb(self, msg):
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y

    def run_calibrated_trial(self, v_cmd, w_cmd, duration_sec=1.0, transient_cutoff_sec=0.3, rearm_pause_sec=2.5):
        self.stop_robot(rearm_pause_sec)
        
        self.timestamps.clear()
        self.gyro_z_history.clear()
        self.rpm_l_history.clear()
        self.rpm_r_history.clear()
        self.odom_x_history.clear()
        self.odom_y_history.clear()
        
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        
        start_time = time.time()
        prev_time = start_time
        integrated_yaw = 0.0
        
        while time.time() - start_time < duration_sec:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.03)
            now = time.time()
            dt = now - prev_time
            prev_time = now
            
            integrated_yaw += self.last_gyro_z * dt
            
            self.timestamps.append(now - start_time)
            self.gyro_z_history.append(self.last_gyro_z)
            self.rpm_l_history.append(self.last_rpm_l)
            self.rpm_r_history.append(self.last_rpm_r)
            self.odom_x_history.append(self.last_odom_x)
            self.odom_y_history.append(self.last_odom_y)
            time.sleep(0.01)
            
        self.stop_robot(0.5)
        
        t_arr = np.array(self.timestamps)
        gz_arr = np.array(self.gyro_z_history)
        rl_arr = np.array(self.rpm_l_history)
        rr_arr = np.array(self.rpm_r_history)
        ox_arr = np.array(self.odom_x_history)
        oy_arr = np.array(self.odom_y_history)
        
        steady_mask = (t_arr >= transient_cutoff_sec) & (t_arr <= duration_sec - 0.05)
        if np.sum(steady_mask) < 2:
            steady_mask = np.ones_like(t_arr, dtype=bool)
            
        t_steady = t_arr[steady_mask]
        steady_dt = float(t_steady[-1] - t_steady[0]) if len(t_steady) > 1 else duration_sec
        
        encoder_l = float(np.mean(rl_arr[steady_mask]))
        encoder_r = float(np.mean(rr_arr[steady_mask]))
        gz_mean = float(np.mean(gz_arr[steady_mask]))
        
        th_l, th_r = calc_theoretical_rpm(v_cmd, w_cmd)
        
        omega_encoder_nominal = calc_omega_wheels(encoder_l, encoder_r, L=L_PHYSICAL_M)
        omega_encoder_effective = calc_omega_wheels(encoder_l, encoder_r, L=L_EFFECTIVE_M)
        
        yaw_rad = integrated_yaw
        omega_physical = (yaw_rad / duration_sec) if duration_sec > 0.0 else gz_mean
        
        g_v = ((math.hypot(ox_arr[-1] - ox_arr[0], oy_arr[-1] - oy_arr[0]) / duration_sec) / v_cmd) if abs(v_cmd) > 0.01 else 1.0
        g_omega_phys = (omega_physical / w_cmd) if abs(w_cmd) > 0.01 else 1.0
        g_gyro = (gz_mean / w_cmd) if abs(w_cmd) > 0.01 else 1.0
        
        total_disp = math.hypot(ox_arr[-1] - ox_arr[0], oy_arr[-1] - oy_arr[0])
        v_physical = total_disp / duration_sec
        
        is_stall = (abs(encoder_l) < 3.0 and abs(th_l) > 8.0) or (abs(encoder_r) < 3.0 and abs(th_r) > 8.0)
        
        sign_correct = True
        if abs(w_cmd) > 0.05:
            if (w_cmd > 0 and gz_mean < -0.02) or (w_cmd < 0 and gz_mean > 0.02):
                sign_correct = False
                
        result = "STALL" if is_stall else ("PASS" if sign_correct else "SIGN_MISMATCH")
        if not is_stall and sign_correct and (abs(encoder_l) < 0.35 * abs(th_l) or abs(encoder_r) < 0.35 * abs(th_r)):
            result = "PARTIAL"
            
        return {
            'v_cmd': v_cmd, 'w_cmd': w_cmd,
            'th_l': th_l, 'th_r': th_r,
            'encoder_l': encoder_l, 'encoder_r': encoder_r,
            'omega_encoder_nom': omega_encoder_nominal,
            'omega_encoder_eff': omega_encoder_effective,
            'gz_mean': gz_mean, 'omega_physical': omega_physical,
            'g_v': g_v, 'g_omega_phys': g_omega_phys, 'g_gyro': g_gyro,
            'yaw_deg': math.degrees(integrated_yaw),
            'disp': total_disp, 'v_physical': v_physical,
            'duration_sec': duration_sec, 'steady_dt': steady_dt,
            'result': result
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
    node = KinematicCalibratorM26CNode()
    
    print('========================================================================')
    print('PLAN C — PHASE M2.6C PHYSICAL KINEMATIC & TRACK-WIDTH CALIBRATION')
    print('========================================================================\n')
    
    # 1. Pure Rotation Angular Scale Test (+/-0.60, +/-0.80)
    print('--- 1. PURE ROTATION ANGULAR SCALE & THREE-WAY COMPARISON ---')
    for w in [+0.60, -0.60, +0.80, -0.80]:
        print(f">> Executing Pure Rotation w={w:+.2f} rad/s ...")
        res = node.run_calibrated_trial(0.0, w, duration_sec=1.0, rearm_pause_sec=2.5)
        print(f"   w={w:+.2f} rad/s: Result={res['result']}")
        print(f"       Encoder L/R: L={res['encoder_l']:+.1f}, R={res['encoder_r']:+.1f}")
        print(f"       Omega Nominal (L=0.110m):   {res['omega_encoder_nom']:+.3f} rad/s")
        print(f"       Omega Effective (L=0.065m): {res['omega_encoder_eff']:+.3f} rad/s")
        print(f"       Omega IMU (GyroMean):       {res['gz_mean']:+.3f} rad/s (G_gyro={res['g_gyro']:.2f})")
        print(f"       Omega Physical (Yaw/dt):    {res['omega_physical']:+.3f} rad/s (G_phys={res['g_omega_phys']:.2f})")
        print(f"       Integrated Yaw: {res['yaw_deg']:+.2f} deg | dt={res['duration_sec']:.2f}s")
        time.sleep(1.0)

    # 2. Straight Motion Physical Velocity (v=0.10, w=0.0)
    print('\n--- 2. STRAIGHT MOTION LINEAR VELOCITY CALIBRATION (v=0.10 m/s) ---')
    for i in range(1, 4):
        print(f">> Straight Trial {i} ...")
        res = node.run_calibrated_trial(0.10, 0.0, duration_sec=1.0, rearm_pause_sec=2.5)
        print(f"   Trial {i}: Target={res['th_l']:+.1f} | Encoder: L={res['encoder_l']:+.1f}, R={res['encoder_r']:+.1f}")
        print(f"       v_cmd={res['v_cmd']:.2f} m/s | v_physical={res['v_physical']:.3f} m/s | G_v={res['g_v']:.2f}")
        print(f"       Disp={res['disp']:.3f}m | Yaw Drift={res['yaw_deg']:+.2f} deg")
        time.sleep(1.0)

    # 3. Floor Curve Benchmarks (A through F)
    print('\n--- 3. COMPLETE FLOOR CURVE BENCHMARKS (A through F) ---')
    curves = [
        ('A', 0.10, +0.20),
        ('B', 0.10, -0.20),
        ('C', 0.10, +0.30),
        ('D', 0.10, -0.30),
        ('E', 0.10, +0.40),
        ('F', 0.10, -0.40)
    ]
    for label, v, w in curves:
        print(f">> Executing Curve {label}: v={v:.2f} m/s, w={w:+.2f} rad/s ...")
        res = node.run_calibrated_trial(v, w, duration_sec=1.0, rearm_pause_sec=2.5)
        print(f"   [{label}] Result: {res['result']}")
        print(f"       Encoder L/R: L={res['encoder_l']:+.1f}, R={res['encoder_r']:+.1f}")
        print(f"       Omega Encoder Nom: {res['omega_encoder_nom']:+.3f} rad/s")
        print(f"       Omega Encoder Eff: {res['omega_encoder_eff']:+.3f} rad/s")
        print(f"       Omega IMU:         {res['gz_mean']:+.3f} rad/s")
        print(f"       Omega Physical:    {res['omega_physical']:+.3f} rad/s (G_phys={res['g_omega_phys']:.2f})")
        print(f"       Yaw: {res['yaw_deg']:+.2f} deg | Disp: {res['disp']:.3f}m")
        time.sleep(1.0)

    # 4. Watchdog Timeout Test
    print('\n--- 4. WATCHDOG TIMEOUT TEST ---')
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
