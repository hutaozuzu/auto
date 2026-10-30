import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import time
import math
import numpy as np

def calc_theoretical_rpm(v, w, r=0.0325, b=0.055):
    v_l = v - b * w
    v_r = v + b * w
    rpm_l = (v_l / (2.0 * math.pi * r)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * r)) * 60.0
    return rpm_l, rpm_r

class DrivetrainBenchmarkM23Node(Node):
    def __init__(self):
        super().__init__('phase_m23_benchmark')
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
        
        gz_min = float(np.min(gz_arr))
        gz_max = float(np.max(gz_arr))
        gz_mean = float(np.mean(gz_arr))
        gz_max_abs = float(np.max(np.abs(gz_arr)))
        
        mean_rpm_l = float(np.mean(np.abs(rl_arr)))
        max_rpm_l = float(np.max(np.abs(rl_arr)))
        min_rpm_l = float(np.min(np.abs(rl_arr)))
        
        mean_rpm_r = float(np.mean(np.abs(rr_arr)))
        max_rpm_r = float(np.max(np.abs(rr_arr)))
        min_rpm_r = float(np.min(np.abs(rr_arr)))
        
        th_l, th_r = calc_theoretical_rpm(v_cmd, w_cmd)
        
        is_stall = (mean_rpm_l < 5.0 and abs(th_l) > 8.0) or (mean_rpm_r < 5.0 and abs(th_r) > 8.0)
        result = "STALL" if is_stall else "PASS"
        if not is_stall and (mean_rpm_l < 0.5 * abs(th_l) or mean_rpm_r < 0.5 * abs(th_r)):
            result = "PARTIAL"
            
        return {
            'v_cmd': v_cmd, 'w_cmd': w_cmd,
            'th_rpm_l': th_l, 'th_rpm_r': th_r,
            'mean_rpm_l': mean_rpm_l, 'max_rpm_l': max_rpm_l, 'min_rpm_l': min_rpm_l,
            'mean_rpm_r': mean_rpm_r, 'max_rpm_r': max_rpm_r, 'min_rpm_r': min_rpm_r,
            'gz_min': gz_min, 'gz_max': gz_max, 'gz_mean': gz_mean, 'gz_max_abs': gz_max_abs,
            'integrated_yaw_deg': math.degrees(integrated_yaw),
            'displacement_m': disp,
            'duration_sec': duration_sec,
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
    node = DrivetrainBenchmarkM23Node()
    
    print('========================================================================')
    print('PLAN C — PHASE M2.3 BENCHMARK & SUSTAIN TRANSITION VALIDATION')
    print('========================================================================\n')
    
    # 1. Theoretical Targets Table
    print('--- THEORETICAL TARGET RPM TABLE ---')
    test_cases = [
        ('A', 0.10, +0.20),
        ('B', 0.10, -0.20),
        ('C', 0.10, +0.30),
        ('D', 0.10, -0.30),
        ('E', 0.10, +0.40),
        ('F', 0.10, -0.40),
    ]
    for label, v, w in test_cases:
        tl, tr = calc_theoretical_rpm(v, w)
        print(f"  Case {label} (v={v:.2f}, w={w:+.2f}): Target Left = {tl:+.2f} RPM | Target Right = {tr:+.2f} RPM")

    # 2. Physical Curved Motion Tests
    print('\n--- RUNNING PHYSICAL CURVED MOTION TESTS (1-6) ---')
    results = []
    for label, v, w in test_cases:
        print(f"\n>> Executing Case {label}: v={v:.2f} m/s, w={w:+.2f} rad/s ...")
        res = node.run_motion_test(v, w, duration_sec=3.0, rearm_pause_sec=1.5)
        results.append((label, res))
        print(f"   -> Result: {res['result']}")
        print(f"   -> Target RPM: Left={res['th_rpm_l']:.1f}, Right={res['th_rpm_r']:.1f}")
        print(f"   -> Measured RPM: Left={res['mean_rpm_l']:.1f} (max {res['max_rpm_l']:.1f}), Right={res['mean_rpm_r']:.1f} (max {res['max_rpm_r']:.1f})")
        print(f"   -> Gyro Z: Min={res['gz_min']:+.3f}, Max={res['gz_max']:+.3f}, Mean={res['gz_mean']:+.3f}, MaxAbs={res['gz_max_abs']:.3f} rad/s")
        print(f"   -> Integrated Yaw: {res['integrated_yaw_deg']:+.2f} deg | Displacement: {res['displacement_m']:.3f} m")

    # 3. Pure Rotation Tests
    print('\n--- RUNNING PURE ROTATION TESTS (v=0.0) ---')
    rot_cases = [
        (+0.60), (-0.60),
        (+0.80), (-0.80)
    ]
    for w in rot_cases:
        print(f"\n>> Executing Pure Spin w={w:+.2f} rad/s ...")
        res = node.run_motion_test(0.0, w, duration_sec=2.5, rearm_pause_sec=1.5)
        overshoot = (res['gz_max_abs'] / abs(w)) if abs(w) > 0 else 1.0
        print(f"   -> Result: {res['result']}")
        print(f"   -> Measured RPM: Left={res['mean_rpm_l']:.1f}, Right={res['mean_rpm_r']:.1f}")
        print(f"   -> Gyro Z: Mean={res['gz_mean']:+.3f} rad/s | MaxAbs={res['gz_max_abs']:.3f} rad/s | Overshoot: {overshoot:.2f}")
        print(f"   -> Integrated Yaw: {res['integrated_yaw_deg']:+.2f} deg")

    # 4. Watchdog Test
    print('\n--- EXECUTING WATCHDOG TIMEOUT TEST ---')
    twist = Twist()
    twist.linear.x = 0.10
    node.cmd_pub.publish(twist)
    time.sleep(0.1)
    time.sleep(0.7)
    rclpy.spin_once(node, timeout_sec=0.1)
    watchdog_rpm_l = node.last_rpm_l
    watchdog_rpm_r = node.last_rpm_r
    watchdog_pass = (abs(watchdog_rpm_l) < 2.0 and abs(watchdog_rpm_r) < 2.0)
    print(f"  -> Watchdog Test Result: {'PASS' if watchdog_pass else 'FAIL'}")

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
