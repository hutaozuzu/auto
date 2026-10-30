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

class IMUAuditNode(Node):
    def __init__(self):
        super().__init__('imu_audit_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_yaw = 0.0
        self.odom_received = False
        
        self.recording = False
        self.imu_samples = []  # tuples: (t_sec, gz)

    def imu_cb(self, msg):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        if self.recording:
            self.imu_samples.append((t_sec, gz))

    def odom_cb(self, msg):
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.last_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def wait_for_odom(self, timeout_sec=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom_received:
                return True
            time.sleep(0.02)
        return False

    def run_imu_audit_test(self, thetar, duration_sec=0.9, rearm_pause_sec=2.0):
        self.stop_robot(rearm_pause_sec)
        self.wait_for_odom()
        
        xr = 0.15
        yr = 0.00
        
        x0 = self.last_odom_x
        y0 = self.last_odom_y
        yaw0 = self.last_odom_yaw
        
        self.imu_samples.clear()
        self.recording = True
        
        t_start = time.time()
        
        while time.time() - t_start < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            dx_world = self.last_odom_x - x0
            dy_world = self.last_odom_y - y0
            curr_x = math.cos(-yaw0) * dx_world - math.sin(-yaw0) * dy_world
            curr_y = math.sin(-yaw0) * dx_world + math.cos(-yaw0) * dy_world
            curr_yaw = wrap_to_pi(self.last_odom_yaw - yaw0)
            
            e1, e2, e3 = compute_kinematic_errors(curr_x, curr_y, curr_yaw, xr, yr, thetar)
            ctrl = compute_control_law(e1, e2, e3)
            
            twist = Twist()
            twist.linear.x = float(ctrl['v_cmd'])
            twist.angular.z = float(ctrl['omega_cmd'])
            self.cmd_pub.publish(twist)
            time.sleep(0.01)
            
        t_end = time.time()
        self.recording = False
        self.stop_robot(0.5)
        rclpy.spin_once(self, timeout_sec=0.05)
        
        final_yaw = wrap_to_pi(self.last_odom_yaw - yaw0)
        ekf_yaw_deg = math.degrees(final_yaw)
        
        dt_total = t_end - t_start
        samples = list(self.imu_samples)
        
        num_samples = len(samples)
        sample_rate = num_samples / dt_total if dt_total > 0 else 0.0
        
        if num_samples >= 2:
            t_arr = np.array([s[0] for s in samples])
            gz_arr = np.array([s[1] for s in samples])
            
            # Normalize timestamps
            t_arr = t_arr - t_arr[0]
            
            gz_min = float(np.min(gz_arr))
            gz_max = float(np.max(gz_arr))
            gz_mean_signed = float(np.mean(gz_arr))
            
            # Integrate via trapezoidal rule
            integrated_rad = float(np.trapz(gz_arr, t_arr))
            integrated_yaw_deg = math.degrees(integrated_rad)
            raw_sample_snippets = [f"{g:+.3f}" for g in gz_arr[:8]]
        else:
            gz_min = 0.0
            gz_max = 0.0
            gz_mean_signed = 0.0
            integrated_yaw_deg = 0.0
            raw_sample_snippets = []
            
        return {
            'thetar': thetar,
            'dt': dt_total,
            'num_samples': num_samples,
            'sample_rate': sample_rate,
            'gz_min': gz_min,
            'gz_max': gz_max,
            'gz_mean_signed': gz_mean_signed,
            'integrated_yaw_deg': integrated_yaw_deg,
            'ekf_yaw_deg': ekf_yaw_deg,
            'raw_snippets': raw_sample_snippets
        }

    def stop_robot(self, pause_sec=0.5):
        zero_twist = Twist()
        t0 = time.time()
        while time.time() - t0 < pause_sec:
            self.cmd_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.02)

def main():
    rclpy.init()
    node = IMUAuditNode()
    
    print('========================================================================')
    print('PLAN C — QUICK K2A.3 IMU PIPELINE & SYNCHRONIZATION AUDIT')
    print('========================================================================\n')
    
    # 1. POSITIVE HEADING TEST (e3 = +0.25 rad)
    print('>> Executing +HEADING AUDIT (e3 = +0.25 rad / +14.3 deg CCW) ...')
    pos_res = node.run_imu_audit_test(+0.25, duration_sec=0.9, rearm_pause_sec=2.0)
    print(f"   DT: {pos_res['dt']:.3f} s | Samples: {pos_res['num_samples']} (Effective Rate: {pos_res['sample_rate']:.1f} Hz)")
    print(f"   Raw Samples (first 8): {pos_res['raw_snippets']}")
    print(f"   Gyro Min: {pos_res['gz_min']:+.4f} rad/s | Gyro Max: {pos_res['gz_max']:+.4f} rad/s | Gyro Mean: {pos_res['gz_mean_signed']:+.4f} rad/s")
    print(f"   IMU Integrated Yaw: {pos_res['integrated_yaw_deg']:+.2f} deg | EKF Yaw: {pos_res['ekf_yaw_deg']:+.2f} deg\n")
    time.sleep(1.0)
    
    # 2. NEGATIVE HEADING TEST (e3 = -0.25 rad)
    print('>> Executing -HEADING AUDIT (e3 = -0.25 rad / -14.3 deg CW) ...')
    neg_res = node.run_imu_audit_test(-0.25, duration_sec=0.9, rearm_pause_sec=2.0)
    print(f"   DT: {neg_res['dt']:.3f} s | Samples: {neg_res['num_samples']} (Effective Rate: {neg_res['sample_rate']:.1f} Hz)")
    print(f"   Raw Samples (first 8): {neg_res['raw_snippets']}")
    print(f"   Gyro Min: {neg_res['gz_min']:+.4f} rad/s | Gyro Max: {neg_res['gz_max']:+.4f} rad/s | Gyro Mean: {neg_res['gz_mean_signed']:+.4f} rad/s")
    print(f"   IMU Integrated Yaw: {neg_res['integrated_yaw_deg']:+.2f} deg | EKF Yaw: {neg_res['ekf_yaw_deg']:+.2f} deg\n")

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
