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

class MechanicalIsolationNode(Node):
    def __init__(self):
        super().__init__('mechanical_isolation_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 200)
        self.sub_rpm_l = self.create_subscription(Float32, '/wheel_rpm_left', self.rpm_l_cb, 100)
        self.sub_rpm_r = self.create_subscription(Float32, '/wheel_rpm_right', self.rpm_r_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        
        self.lock = threading.Lock()
        self.recording = False
        self.imu_records = []    # (header_t, arrival_t, gz)
        self.rpm_l_records = []
        self.rpm_r_records = []
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0

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

def run_single_trial(node, v_cmd, w_cmd, duration_sec=0.7):
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
    
    t_stop = time.time()
    while time.time() - t_stop < 0.5:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    xf, yf, yawf = node.get_odom_pose()
    ekf_yaw_deg = math.degrees(wrap_to_pi(yawf - yaw0))
    
    act_rpm_l = [r[1] for r in rpm_l_recs] if rpm_l_recs else [0.0]
    act_rpm_r = [r[1] for r in rpm_r_recs] if rpm_r_recs else [0.0]
    
    mean_rpm_l = float(np.mean(act_rpm_l))
    mean_rpm_r = float(np.mean(act_rpm_r))
    delta_rpm_actual = mean_rpm_r - mean_rpm_l
    
    if len(imu_recs) >= 2:
        header_ts = np.array([r[0] for r in imu_recs])
        gz_vals = np.array([r[2] for r in imu_recs])
        integrated_rad = float(np.trapz(gz_vals, header_ts))
        integrated_yaw_deg = math.degrees(integrated_rad)
    else:
        integrated_yaw_deg = 0.0
        
    return {
        'target_l': target_rpm_l, 'target_r': target_rpm_r,
        'mean_l': mean_rpm_l, 'mean_r': mean_rpm_r,
        'delta_rpm_target': delta_rpm_target,
        'delta_rpm_actual': delta_rpm_actual,
        'imu_yaw': integrated_yaw_deg,
        'ekf_yaw': ekf_yaw_deg
    }

def run_block(node, label, condition_name):
    print(f"\n==================================================")
    print(f"BLOCK: {condition_name}")
    print(f"==================================================")
    
    ccw_yaws = []
    cw_yaws = []
    
    print(">> 3x CCW Trials (w = +0.30 rad/s) ...")
    for i in range(1, 4):
        res = run_single_trial(node, 0.08, +0.30, duration_sec=0.7)
        ccw_yaws.append(res['imu_yaw'])
        print(f"   CCW Trial {i}: L/R={res['mean_l']:+.1f}/{res['mean_r']:+.1f} (Delta={res['delta_rpm_actual']:+.1f}) | IMU Yaw={res['imu_yaw']:+.2f} deg | EKF Yaw={res['ekf_yaw']:+.2f} deg")
        time.sleep(1.5)
        
    print("\n>> 3x CW Trials (w = -0.30 rad/s) ...")
    for i in range(1, 4):
        res = run_single_trial(node, 0.08, -0.30, duration_sec=0.7)
        cw_yaws.append(res['imu_yaw'])
        print(f"   CW Trial {i}:  L/R={res['mean_l']:+.1f}/{res['mean_r']:+.1f} (Delta={res['delta_rpm_actual']:+.1f}) | IMU Yaw={res['imu_yaw']:+.2f} deg | EKF Yaw={res['ekf_yaw']:+.2f} deg")
        time.sleep(1.5)
        
    mean_ccw = float(np.mean(ccw_yaws))
    mean_cw = float(np.mean(cw_yaws))
    ratio = (abs(mean_ccw) / abs(mean_cw)) if abs(mean_cw) > 0.01 else 999.0
    
    print(f"\n>> {condition_name} Summary: CCW Mean={mean_ccw:+.2f} deg | CW Mean={mean_cw:+.2f} deg | Ratio={ratio:.2f}")
    return mean_ccw, mean_cw, ratio

def main():
    rclpy.init()
    node = MechanicalIsolationNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    
    # 1. NORMAL
    c1_ccw, c1_cw, c1_ratio = run_block(node, "TEST 1", "NORMAL CASTER ORIENTATION")
    
    # 2. CASTER PRE-ALIGNED
    time.sleep(2.0)
    c2_ccw, c2_cw, c2_ratio = run_block(node, "TEST 2", "CASTER PRE-ALIGNED")
    
    # 3. REVERSE STARTING ORIENTATION (Robot 180 deg)
    time.sleep(2.0)
    c3_ccw, c3_cw, c3_ratio = run_block(node, "TEST 3", "ROBOT ROTATED 180 DEG")
    
    print('\n========================================================================')
    print('FINAL ISOLATION COMPARISON')
    print('========================================================================')
    print(f"NORMAL:             CCW Mean={c1_ccw:+.2f} deg | CW Mean={c1_cw:+.2f} deg | Ratio={c1_ratio:.2f}")
    print(f"CASTER PRE-ALIGNED: CCW Mean={c2_ccw:+.2f} deg | CW Mean={c2_cw:+.2f} deg | Ratio={c2_ratio:.2f}")
    print(f"ROBOT ROTATED 180:  CCW Mean={c3_ccw:+.2f} deg | CW Mean={c3_cw:+.2f} deg | Ratio={c3_ratio:.2f}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
