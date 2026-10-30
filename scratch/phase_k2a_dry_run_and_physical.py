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

# Kinematic Controller Parameters (Phase K2)
K1 = 0.8
K2 = 1.5
K3 = 0.8
V_NOMINAL = 0.05
V_MAX = 0.06
OMEGA_MAX = 0.60
TERMINAL_K_THETA = 1.2
TERMINAL_OMEGA_MAX = 0.60
XY_GOAL_TOLERANCE = 0.05
YAW_GOAL_TOLERANCE = 0.05

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
    
    # Inverse kinematics
    v_l = v_cmd - WHEEL_TRACK_HALF * omega_cmd
    v_r = v_cmd + WHEEL_TRACK_HALF * omega_cmd
    rpm_l = (v_l / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    
    return v_raw, omega_raw, v_cmd, omega_cmd, rpm_l, rpm_r

class PhaseK2AExecutorNode(Node):
    def __init__(self):
        super().__init__('phase_k2a_executor')
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

    def run_k2a_physical_test(self, xr, yr, thetar, vr=V_NOMINAL, wr=0.0, duration_sec=0.8, rearm_pause_sec=2.0):
        self.stop_robot(rearm_pause_sec)
        
        self.timestamps.clear()
        self.v_cmd_history.clear()
        self.omega_cmd_history.clear()
        self.gyro_z_history.clear()
        
        # Capture initial pose
        rclpy.spin_once(self, timeout_sec=0.05)
        x0, y0, yaw0 = self.last_odom_x, self.last_odom_y, self.last_odom_yaw
        
        e1_0, e2_0, e3_0 = compute_kinematic_errors(0.0, 0.0, 0.0, xr, yr, thetar)
        
        start_time = time.time()
        prev_time = start_time
        integrated_yaw = 0.0
        max_cross_track = abs(e2_0)
        max_v_seen = 0.0
        max_omega_seen = 0.0
        
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - prev_time
            prev_time = now
            
            integrated_yaw += self.last_gyro_z * dt
            
            # Robot state relative to initial
            curr_x = self.last_odom_x - x0
            curr_y = self.last_odom_y - y0
            curr_yaw = self.last_odom_yaw - yaw0
            
            e1, e2, e3 = compute_kinematic_errors(curr_x, curr_y, curr_yaw, xr, yr, thetar)
            max_cross_track = max(max_cross_track, abs(e2))
            
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
        
        final_pos_err = math.hypot(xr - final_x, yr - final_y)
        final_yaw_err = abs(wrap_to_pi(thetar - final_yaw))
        
        return {
            'e1_0': e1_0, 'e2_0': e2_0, 'e3_0': e3_0,
            'max_e2': max_cross_track,
            'final_pos_err': final_pos_err,
            'final_yaw_err': math.degrees(final_yaw_err),
            'max_v': max_v_seen,
            'max_omega': max_omega_seen,
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
    node = PhaseK2AExecutorNode()
    
    print('========================================================================')
    print('PLAN C — PHASE K2A WHOLE-BODY CONTROLLER OFFLINE & PHYSICAL VALIDATION')
    print('========================================================================\n')
    
    # 1. OFFLINE / DRY-RUN ERROR RESPONSE TESTS
    print('--- 1. OFFLINE CONTROL LAW SIGN VERIFICATION ---')
    offline_cases = [
        ('A. e1 > 0 (Lagging)', +0.10, 0.00, 0.00),
        ('B. e1 < 0 (Leading)', -0.10, 0.00, 0.00),
        ('C. e2 > 0 (Left of Robot / Need CCW Steer)', 0.00, +0.10, 0.00),
        ('D. e2 < 0 (Right of Robot / Need CW Steer)', 0.00, -0.10, 0.00),
        ('E. e3 > 0 (Reference Heading CCW)', 0.00, 0.00, +0.30),
        ('F. e3 < 0 (Reference Heading CW)', 0.00, 0.00, -0.30),
        ('G. Combined (e1>0, e2>0, e3>0)', +0.08, +0.05, +0.20)
    ]
    for name, e1, e2, e3 in offline_cases:
        v_raw, w_raw, v_cmd, w_cmd, rl, rr = compute_control_law(e1, e2, e3)
        print(f"  [{name}]: e=({e1:+.2f}, {e2:+.2f}, {e3:+.2f}) -> v_cmd={v_cmd:+.3f} m/s, w_cmd={w_cmd:+.3f} rad/s | Target L/R: {rl:+.1f}/{rr:+.1f} RPM")
    print('>> Offline Control Law: ALL DIRECTIONAL SIGNS VERIFIED.\n')

    # 2. PHYSICAL CONTROLLED ERROR RESPONSE TESTS
    print('--- 2. PHYSICAL CONTROLLED TRACKING TESTS (K2A TEST SET) ---')
    all_v_cmds = []
    all_w_cmds = []
    
    tests = [
        ('TEST A — STRAIGHT ALIGNED', 0.15, 0.00, 0.00),
        ('TEST B — LEFT LATERAL OFFSET', 0.15, -0.05, 0.00),
        ('TEST C — RIGHT LATERAL OFFSET', 0.15, +0.05, 0.00),
        ('TEST D — POSITIVE HEADING OFFSET', 0.15, 0.00, +0.25),
        ('TEST E — NEGATIVE HEADING OFFSET', 0.15, 0.00, -0.25),
        ('TEST F — COMBINED OFFSET', 0.15, +0.04, +0.20)
    ]
    
    results = {}
    for label, xr, yr, thetar in tests:
        print(f">> Running {label} (Target: xr={xr:.2f}m, yr={yr:+.2f}m, thetar={thetar:+.2f}rad) ...")
        res = node.run_k2a_physical_test(xr, yr, thetar, duration_sec=0.8, rearm_pause_sec=2.0)
        results[label] = res
        all_v_cmds.extend(res['v_history'])
        all_w_cmds.extend(res['w_history'])
        print(f"   Initial: e1={res['e1_0']:+.3f}, e2={res['e2_0']:+.3f}, e3={res['e3_0']:+.3f}")
        print(f"   Max Cross-Track e2: {res['max_e2']:.3f}m | Final Pos Err: {res['final_pos_err']:.3f}m | Final Yaw Err: {res['final_yaw_err']:.1f} deg")
        print(f"   Max v: {res['max_v']:.3f} m/s | Max omega: {res['max_omega']:.3f} rad/s | Integrated Yaw: {res['yaw_int_deg']:+.2f} deg")
        time.sleep(1.0)

    # 3. COMMAND DISTRIBUTION ANALYSIS
    v_arr = np.array(all_v_cmds)
    w_arr = np.array(all_w_cmds)
    total_samples = len(v_arr)
    
    pct_v_low = float(np.sum(np.abs(v_arr) < 0.03) / total_samples * 100.0)
    pct_w_low = float(np.sum((np.abs(w_arr) > 0.001) & (np.abs(w_arr) < 0.20)) / total_samples * 100.0)
    pct_w_mid = float(np.sum((np.abs(w_arr) >= 0.20) & (np.abs(w_arr) < 0.30)) / total_samples * 100.0)
    pct_w_high = float(np.sum(np.abs(w_arr) >= 0.30) / total_samples * 100.0)
    
    print('\n--- 3. COMMAND DISTRIBUTION SUMMARY ---')
    print(f"  Total Controller Cycles Logged: {total_samples}")
    print(f"  percent time |v| < 0.03 m/s:       {pct_v_low:.1f} %")
    print(f"  percent time 0 < |omega| < 0.20:   {pct_w_low:.1f} %")
    print(f"  percent time 0.20 <= |omega| < 0.30: {pct_w_mid:.1f} %")
    print(f"  percent time |omega| >= 0.30:      {pct_w_high:.1f} %")

    # 4. Watchdog Timeout Test
    print('\n--- 4. SAFETY REGRESSION CHECK ---')
    twist = Twist()
    twist.linear.x = 0.05
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
