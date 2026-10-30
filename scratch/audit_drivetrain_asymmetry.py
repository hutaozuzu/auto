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

# Firmware FF Table Constants (src/speed_controller.cpp)
FF_PWM = [70, 75, 80, 85, 90]
FF_LEFT_RPM = [10.8, 15.6, 21.1, 26.8, 32.5]
FF_RIGHT_RPM = [10.4, 15.2, 20.6, 26.1, 31.8]

def compute_firmware_ff(target_rpm, is_left):
    if target_rpm <= 0:
        return 0
    rpm_table = FF_LEFT_RPM if is_left else FF_RIGHT_RPM
    min_floor = 64.0 if is_left else 66.0
    if target_rpm <= rpm_table[0]:
        ratio = target_rpm / rpm_table[0]
        ff = min_floor + ratio * (90.0 - min_floor)
        return int(np.clip(ff, 0, 255))
    if target_rpm >= rpm_table[-1]:
        return int(FF_PWM[-1])
    for i in range(len(rpm_table) - 1):
        if target_rpm >= rpm_table[i] and target_rpm <= rpm_table[i + 1]:
            slope = (FF_PWM[i + 1] - FF_PWM[i]) / (rpm_table[i + 1] - rpm_table[i])
            ff = FF_PWM[i] + slope * (target_rpm - rpm_table[i])
            return int(np.clip(ff, 0, 255))
    return 0

def simulate_firmware_pi(target_rpm, measured_rpm, is_left, kp=0.45, ki=0.8, dt=0.05):
    ff = compute_firmware_ff(target_rpm, is_left)
    error = target_rpm - measured_rpm
    p_term = kp * error
    # estimated steady integral
    i_term = np.clip(ki * error * 0.5, -20.0, 20.0)
    raw_pwm = ff + p_term + i_term
    min_sustain = 72 if is_left else 76
    adaptive_sustain = max(min_sustain, ff + 4)
    final_pwm = int(np.clip(max(raw_pwm, adaptive_sustain if measured_rpm < 5.0 else raw_pwm), 0, 255))
    state = "KICK" if measured_rpm < 1.0 else ("SUSTAIN_HOLD" if final_pwm == adaptive_sustain else "NORMAL_PI")
    return {
        'ff': ff, 'p': p_term, 'i': i_term, 'pwm': final_pwm, 'state': state
    }

def rpm_to_twist(rpm_l, rpm_r):
    v_l = (rpm_l / 60.0) * (2.0 * math.pi * WHEEL_RADIUS_M)
    v_r = (rpm_r / 60.0) * (2.0 * math.pi * WHEEL_RADIUS_M)
    v = (v_l + v_r) / 2.0
    w = (v_r - v_l) / L_PHYSICAL_M
    return v, w

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class DrivetrainAuditorNode(Node):
    def __init__(self):
        super().__init__('drivetrain_auditor_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 200)
        self.sub_rpm_l = self.create_subscription(Float32, '/wheel_rpm_left', self.rpm_l_cb, 100)
        self.sub_rpm_r = self.create_subscription(Float32, '/wheel_rpm_right', self.rpm_r_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        
        self.lock = threading.Lock()
        self.recording = False
        self.imu_records = []
        self.rpm_l_records = []
        self.rpm_r_records = []
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0

    def imu_cb(self, msg):
        header_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        with self.lock:
            if self.recording:
                self.imu_records.append((header_t, gz))

    def rpm_l_cb(self, msg):
        with self.lock:
            if self.recording:
                self.rpm_l_records.append(msg.data)

    def rpm_r_cb(self, msg):
        with self.lock:
            if self.recording:
                self.rpm_r_records.append(msg.data)

    def odom_cb(self, msg):
        with self.lock:
            self.last_odom_x = msg.pose.pose.position.x
            self.last_odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.last_odom_yaw = math.atan2(siny_cosp, cosy_cosp)

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

def run_rpm_trial(node, target_l, target_r, duration_sec=0.7):
    zero_twist = Twist()
    t_stop = time.time()
    while time.time() - t_stop < 0.8:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    x0, y0, yaw0 = node.get_odom_pose()
    
    v_cmd, w_cmd = rpm_to_twist(target_l, target_r)
    twist = Twist()
    twist.linear.x = float(v_cmd)
    twist.angular.z = float(w_cmd)
    
    node.start_recording()
    t_start = time.time()
    
    while time.time() - t_start < duration_sec:
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
        
    t_end = time.time()
    imu_recs, act_l_list, act_r_list = node.stop_recording()
    
    t_stop = time.time()
    while time.time() - t_stop < 0.5:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    xf, yf, yawf = node.get_odom_pose()
    ekf_yaw_deg = math.degrees(wrap_to_pi(yawf - yaw0))
    
    act_l = float(np.mean(act_l_list)) if act_l_list else 0.0
    act_r = float(np.mean(act_r_list)) if act_r_list else 0.0
    
    delta_target = target_r - target_l
    delta_actual = act_r - act_l
    g_delta = (delta_actual / delta_target) if abs(delta_target) > 0.1 else 1.0
    
    pi_l = simulate_firmware_pi(target_l, act_l, is_left=True)
    pi_r = simulate_firmware_pi(target_r, act_r, is_left=False)
    
    if len(imu_recs) >= 2:
        header_ts = np.array([r[0] for r in imu_recs])
        gz_vals = np.array([r[1] for r in imu_recs])
        integrated_rad = float(np.trapz(gz_vals, header_ts))
        integrated_yaw_deg = math.degrees(integrated_rad)
    else:
        integrated_yaw_deg = 0.0
        
    return {
        'target_l': target_l, 'target_r': target_r,
        'act_l': act_l, 'act_r': act_r,
        'err_l': target_l - act_l, 'err_r': target_r - act_r,
        'pi_l': pi_l, 'pi_r': pi_r,
        'delta_target': delta_target, 'delta_actual': delta_actual,
        'g_delta': g_delta,
        'imu_yaw': integrated_yaw_deg,
        'ekf_yaw': ekf_yaw_deg
    }

def main():
    rclpy.init()
    node = DrivetrainAuditorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    
    print('========================================================================')
    print('PLAN C — QUICK DRIVETRAIN ASYMMETRY AUDIT (L-HIGH vs R-HIGH)')
    print('========================================================================\n')
    
    # 1. PAIR A: L=19, R=28 (3 trials)
    print('>> PAIR A — L19 / R28 (Right High) [3 trials] ...')
    pair_a_results = []
    for i in range(1, 4):
        res = run_rpm_trial(node, 19.0, 28.0, duration_sec=0.7)
        pair_a_results.append(res)
        print(f"  Trial {i}: L target/actual={res['target_l']:.1f}/{res['act_l']:+.1f} | R target/actual={res['target_r']:.1f}/{res['act_r']:+.1f}")
        print(f"           PWM L/R={res['pi_l']['pwm']}/{res['pi_r']['pwm']} | FF L/R={res['pi_l']['ff']}/{res['pi_r']['ff']} | P L/R={res['pi_l']['p']:+.1f}/{res['pi_r']['p']:+.1f} | I L/R={res['pi_l']['i']:+.1f}/{res['pi_r']['i']:+.1f}")
        print(f"           state L/R={res['pi_l']['state']}/{res['pi_r']['state']} | DeltaTarget={res['delta_target']:+.1f} | DeltaActual={res['delta_actual']:+.1f} | G_delta={res['g_delta']:.2f}")
        print(f"           IMU Yaw={res['imu_yaw']:+.2f} deg | EKF Yaw={res['ekf_yaw']:+.2f} deg\n")
        time.sleep(1.5)
        
    # 2. PAIR B: L=28, R=19 (3 trials)
    print('>> PAIR B — L28 / R19 (Left High) [3 trials] ...')
    pair_b_results = []
    for i in range(1, 4):
        res = run_rpm_trial(node, 28.0, 19.0, duration_sec=0.7)
        pair_b_results.append(res)
        print(f"  Trial {i}: L target/actual={res['target_l']:.1f}/{res['act_l']:+.1f} | R target/actual={res['target_r']:.1f}/{res['act_r']:+.1f}")
        print(f"           PWM L/R={res['pi_l']['pwm']}/{res['pi_r']['pwm']} | FF L/R={res['pi_l']['ff']}/{res['pi_r']['ff']} | P L/R={res['pi_l']['p']:+.1f}/{res['pi_r']['p']:+.1f} | I L/R={res['pi_l']['i']:+.1f}/{res['pi_r']['i']:+.1f}")
        print(f"           state L/R={res['pi_l']['state']}/{res['pi_r']['state']} | DeltaTarget={res['delta_target']:+.1f} | DeltaActual={res['delta_actual']:+.1f} | G_delta={res['g_delta']:.2f}")
        print(f"           IMU Yaw={res['imu_yaw']:+.2f} deg | EKF Yaw={res['ekf_yaw']:+.2f} deg\n")
        time.sleep(1.5)

    # 3. SAME-WHEEL TARGET SWEEP (19, 22, 25, 28, 31 RPM)
    print('>> SAME-WHEEL TARGET SWEEPS (Straight Motion) ...')
    rpm_sweep = [19.0, 22.0, 25.0, 28.0, 31.0]
    sweep_results = []
    for rpm in rpm_sweep:
        res = run_rpm_trial(node, rpm, rpm, duration_sec=0.7)
        sweep_results.append(res)
        print(f"  Target {rpm:.1f} RPM: L Actual={res['act_l']:+.1f} (PWM={res['pi_l']['pwm']}, {res['pi_l']['state']}) | R Actual={res['act_r']:+.1f} (PWM={res['pi_r']['pwm']}, {res['pi_r']['state']}) | IMU Yaw={res['imu_yaw']:+.2f} deg")
        time.sleep(1.2)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
