import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
import threading

WHEEL_RADIUS_M = 0.0325
L_PHYSICAL_M = 0.110
WHEEL_TRACK_HALF = L_PHYSICAL_M / 2.0

# Fixed Kinematic Parameters (Phase K2A.3 Audit)
K1 = 0.8
K2 = 60.0
K3 = 15.0
V_NOMINAL = 0.08
V_MAX = 0.10
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
    v_raw = vr * math.cos(e3) + k1 * e1
    omega_e2 = k2 * vr * e2
    omega_e3 = k3 * vr * math.sin(e3)
    omega_raw = wr + omega_e2 + omega_e3
    
    v_cmd = max(-v_max, min(v_max, v_raw))
    omega_cmd = max(-omega_max, min(omega_max, omega_raw))
    
    if v_cmd < 0.0:
        v_cmd = 0.0
        
    v_l = v_cmd - WHEEL_TRACK_HALF * omega_cmd
    v_r = v_cmd + WHEEL_TRACK_HALF * omega_cmd
    rpm_l = (v_l / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    rpm_r = (v_r / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
    
    return {
        'v_raw': v_raw, 'omega_raw': omega_raw,
        'omega_e2': omega_e2, 'omega_e3': omega_e3,
        'v_cmd': v_cmd, 'omega_cmd': omega_cmd,
        'rpm_l': rpm_l, 'rpm_r': rpm_r
    }

class ThreadedIMUAuditNode(Node):
    def __init__(self):
        super().__init__('threaded_imu_audit')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0
        self.odom_received = False
        
        self.lock = threading.Lock()
        self.recording = False
        self.imu_samples = []  # (t_sec, gz)

    def imu_cb(self, msg):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        with self.lock:
            if self.recording:
                self.imu_samples.append((t_sec, gz))

    def odom_cb(self, msg):
        with self.lock:
            self.last_odom_x = msg.pose.pose.position.x
            self.last_odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.last_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.odom_received = True

    def get_pose(self):
        with self.lock:
            return self.last_odom_x, self.last_odom_y, self.last_odom_yaw

    def start_recording(self):
        with self.lock:
            self.imu_samples.clear()
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            return list(self.imu_samples)

def run_test(node, thetar, duration_sec=0.9):
    # Rearm stop
    zero_twist = Twist()
    t_stop = time.time()
    while time.time() - t_stop < 1.0:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    # Wait for fresh odom
    time.sleep(0.2)
    x0, y0, yaw0 = node.get_pose()
    
    xr = 0.15
    yr = 0.00
    
    node.start_recording()
    t_start = time.time()
    
    while time.time() - t_start < duration_sec:
        curr_x_raw, curr_y_raw, curr_yaw_raw = node.get_pose()
        dx_world = curr_x_raw - x0
        dy_world = curr_y_raw - y0
        curr_x = math.cos(-yaw0) * dx_world - math.sin(-yaw0) * dy_world
        curr_y = math.sin(-yaw0) * dx_world + math.cos(-yaw0) * dy_world
        curr_yaw = wrap_to_pi(curr_yaw_raw - yaw0)
        
        e1, e2, e3 = compute_kinematic_errors(curr_x, curr_y, curr_yaw, xr, yr, thetar)
        ctrl = compute_control_law(e1, e2, e3)
        
        twist = Twist()
        twist.linear.x = float(ctrl['v_cmd'])
        twist.angular.z = float(ctrl['omega_cmd'])
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
        
    t_end = time.time()
    samples = node.stop_recording()
    
    # Send stop
    t_stop = time.time()
    while time.time() - t_stop < 0.5:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    xf, yf, yawf = node.get_pose()
    ekf_yaw_deg = math.degrees(wrap_to_pi(yawf - yaw0))
    
    dt_total = t_end - t_start
    num_samples = len(samples)
    sample_rate = num_samples / dt_total if dt_total > 0 else 0.0
    
    if num_samples >= 2:
        t_arr = np.array([s[0] for s in samples])
        gz_arr = np.array([s[1] for s in samples])
        t_norm = t_arr - t_arr[0]
        
        gz_min = float(np.min(gz_arr))
        gz_max = float(np.max(gz_arr))
        gz_mean = float(np.mean(gz_arr))
        
        integrated_rad = float(np.trapz(gz_arr, t_norm))
        integrated_yaw_deg = math.degrees(integrated_rad)
        raw_snippets = [f"{g:+.3f}" for g in gz_arr[:8]]
    else:
        gz_min, gz_max, gz_mean, integrated_yaw_deg = 0.0, 0.0, 0.0, 0.0
        raw_snippets = []
        
    return {
        'thetar': thetar,
        'dt': dt_total,
        'omega_cmd_first': float(compute_control_law(0, 0, thetar)['omega_cmd']),
        'num_samples': num_samples,
        'sample_rate': sample_rate,
        'gz_min': gz_min,
        'gz_max': gz_max,
        'gz_mean': gz_mean,
        'integrated_yaw_deg': integrated_yaw_deg,
        'ekf_yaw_deg': ekf_yaw_deg,
        'raw_snippets': raw_snippets
    }

def main():
    rclpy.init()
    node = ThreadedIMUAuditNode()
    
    # Spin executor in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Wait for first odom
    time.sleep(1.0)
    
    print('========================================================================')
    print('PLAN C — QUICK K2A.3 THREADED IMU AUDIT')
    print('========================================================================\n')
    
    # 1. POSITIVE HEADING (+e3 = +0.25 rad)
    print('>> Executing +HEADING (+0.25 rad / +14.3 deg CCW) ...')
    pos = run_test(node, +0.25, duration_sec=0.9)
    print(f"   DT: {pos['dt']:.3f}s | omega_cmd: {pos['omega_cmd_first']:+.3f} rad/s")
    print(f"   Samples: {pos['num_samples']} (Effective Rate: {pos['sample_rate']:.1f} Hz)")
    print(f"   Raw Samples (first 8): {pos['raw_snippets']}")
    print(f"   Gyro Min: {pos['gz_min']:+.4f} | Gyro Max: {pos['gz_max']:+.4f} | Gyro Mean: {pos['gz_mean']:+.4f} rad/s")
    print(f"   Integrated IMU Yaw: {pos['integrated_yaw_deg']:+.2f} deg | EKF Yaw: {pos['ekf_yaw_deg']:+.2f} deg | Consistent: {'YES' if pos['integrated_yaw_deg']*pos['ekf_yaw_deg'] > 0 else 'NO'}\n")
    time.sleep(1.5)
    
    # 2. NEGATIVE HEADING (-e3 = -0.25 rad)
    print('>> Executing -HEADING (-0.25 rad / -14.3 deg CW) ...')
    neg = run_test(node, -0.25, duration_sec=0.9)
    print(f"   DT: {neg['dt']:.3f}s | omega_cmd: {neg['omega_cmd_first']:+.3f} rad/s")
    print(f"   Samples: {neg['num_samples']} (Effective Rate: {neg['sample_rate']:.1f} Hz)")
    print(f"   Raw Samples (first 8): {neg['raw_snippets']}")
    print(f"   Gyro Min: {neg['gz_min']:+.4f} | Gyro Max: {neg['gz_max']:+.4f} | Gyro Mean: {neg['gz_mean']:+.4f} rad/s")
    print(f"   Integrated IMU Yaw: {neg['integrated_yaw_deg']:+.2f} deg | EKF Yaw: {neg['ekf_yaw_deg']:+.2f} deg | Consistent: {'YES' if neg['integrated_yaw_deg']*neg['ekf_yaw_deg'] > 0 else 'NO'}\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
