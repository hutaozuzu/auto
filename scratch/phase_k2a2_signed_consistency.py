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

# Tuned Kinematic Parameters for Proven Operating Envelope (Phase K2A.2)
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

class PhaseK2A2ExecutorNode(Node):
    def __init__(self):
        super().__init__('phase_k2a2_executor')
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
        self.odom_received = False
        
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
        self.odom_received = True

    def wait_for_odom(self, timeout_sec=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom_received:
                return True
            time.sleep(0.02)
        return False

    def run_k2a2_test(self, xr, yr, thetar, vr=V_NOMINAL, wr=0.0, duration_sec=0.9, rearm_pause_sec=2.0):
        self.stop_robot(rearm_pause_sec)
        self.wait_for_odom()
        
        self.timestamps.clear()
        self.v_cmd_history.clear()
        self.omega_cmd_history.clear()
        self.gyro_z_history.clear()
        
        x0 = self.last_odom_x
        y0 = self.last_odom_y
        yaw0 = self.last_odom_yaw
        
        e1_0, e2_0, e3_0 = compute_kinematic_errors(0.0, 0.0, 0.0, xr, yr, thetar)
        
        t_start = time.time()
        prev_time = t_start
        integrated_gyro_yaw = 0.0
        
        first_ctrl = compute_control_law(e1_0, e2_0, e3_0, vr, wr)
        omega_first = first_ctrl['omega_cmd']
        
        while time.time() - t_start < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - prev_time
            prev_time = now
            
            integrated_gyro_yaw += self.last_gyro_z * dt
            
            dx_world = self.last_odom_x - x0
            dy_world = self.last_odom_y - y0
            curr_x = math.cos(-yaw0) * dx_world - math.sin(-yaw0) * dy_world
            curr_y = math.sin(-yaw0) * dx_world + math.cos(-yaw0) * dy_world
            curr_yaw = wrap_to_pi(self.last_odom_yaw - yaw0)
            
            e1, e2, e3 = compute_kinematic_errors(curr_x, curr_y, curr_yaw, xr, yr, thetar)
            
            ctrl = compute_control_law(e1, e2, e3, vr, wr)
            v_cmd = ctrl['v_cmd']
            w_cmd = ctrl['omega_cmd']
            
            twist = Twist()
            twist.linear.x = float(v_cmd)
            twist.angular.z = float(w_cmd)
            self.cmd_pub.publish(twist)
            
            self.timestamps.append(now - t_start)
            self.v_cmd_history.append(v_cmd)
            self.omega_cmd_history.append(w_cmd)
            self.gyro_z_history.append(self.last_gyro_z)
            time.sleep(0.02)
            
        t_end = time.time()
        self.stop_robot(0.5)
        rclpy.spin_once(self, timeout_sec=0.05)
        
        dx_world_f = self.last_odom_x - x0
        dy_world_f = self.last_odom_y - y0
        final_x = math.cos(-yaw0) * dx_world_f - math.sin(-yaw0) * dy_world_f
        final_y = math.sin(-yaw0) * dx_world_f + math.cos(-yaw0) * dy_world_f
        final_yaw = wrap_to_pi(self.last_odom_yaw - yaw0)
        
        e1_1, e2_1, e3_1 = compute_kinematic_errors(final_x, final_y, final_yaw, xr, yr, thetar)
        
        recomp_e1, recomp_e2, recomp_e3 = compute_kinematic_errors(final_x, final_y, final_yaw, xr, yr, thetar)
        recomp_agree = (abs(e1_1 - recomp_e1) < 1e-4 and abs(e2_1 - recomp_e2) < 1e-4 and abs(e3_1 - recomp_e3) < 1e-4)
        
        delta_abs_e1 = abs(e1_0) - abs(e1_1)
        delta_abs_e2 = abs(e2_0) - abs(e2_1)
        delta_abs_e3 = abs(e3_0) - abs(e3_1)
        
        w_arr = np.array(self.omega_cmd_history)
        gz_arr = np.array(self.gyro_z_history)
        
        omega_min_signed = float(np.min(w_arr))
        omega_max_signed = float(np.max(w_arr))
        omega_mean_signed = float(np.mean(w_arr))
        omega_peak_abs = float(np.max(np.abs(w_arr)))
        gyro_mean_signed = float(np.mean(gz_arr))
        
        pose_disp = math.hypot(final_x, final_y)
        max_plausible_disp = V_MAX * (t_end - t_start) + 0.03
        
        return {
            'dt': t_end - t_start,
            'e1_0': e1_0, 'e2_0': e2_0, 'e3_0': e3_0,
            'final_x': final_x, 'final_y': final_y, 'final_yaw': final_yaw,
            'e1_1': e1_1, 'e2_1': e2_1, 'e3_1': e3_1,
            'delta_abs_e1': delta_abs_e1,
            'delta_abs_e2': delta_abs_e2,
            'delta_abs_e3': delta_abs_e3,
            'recomp_agree': recomp_agree,
            'omega_first': omega_first,
            'omega_min_signed': omega_min_signed,
            'omega_max_signed': omega_max_signed,
            'omega_mean_signed': omega_mean_signed,
            'omega_peak_abs': omega_peak_abs,
            'gyro_mean_signed': gyro_mean_signed,
            'imu_yaw_deg': math.degrees(integrated_gyro_yaw),
            'ekf_yaw_deg': math.degrees(final_yaw),
            'pose_disp': pose_disp,
            'max_plausible_disp': max_plausible_disp
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
    node = PhaseK2A2ExecutorNode()
    
    print('========================================================================')
    print('PLAN C — PHASE K2A.2 OPERATING ENVELOPE (vr=0.08 m/s, k1=0.8, k2=60, k3=15)')
    print('========================================================================\n')
    
    tests = [
        ('TEST A (+HEADING / CCW)', 0.15, 0.00, +0.25),
        ('TEST B (-HEADING / CW)', 0.15, 0.00, -0.25),
        ('TEST C (+LATERAL / Left)', 0.15, +0.05, 0.00),
        ('TEST D (-LATERAL / Right)', 0.15, -0.05, 0.00),
        ('TEST E (LONGITUDINAL)', 0.15, 0.00, 0.00),
        ('TEST F (+COMBINED)', 0.15, +0.04, +0.20),
        ('TEST G (-COMBINED)', 0.15, -0.04, -0.20)
    ]
    
    for label, xr, yr, thetar in tests:
        print(f">> Executing {label} (Reference: xr={xr:.2f}, yr={yr:+.2f}, thetar={thetar:+.2f}) ...")
        res = node.run_k2a2_test(xr, yr, thetar, duration_sec=0.9, rearm_pause_sec=2.0)
        
        print(f"   dt={res['dt']:.2f}s | RecompAgree={res['recomp_agree']}")
        print(f"   Start: e1={res['e1_0']:+.3f}m, e2={res['e2_0']:+.3f}m, e3={res['e3_0']:+.3f}rad")
        print(f"   End:   e1={res['e1_1']:+.3f}m, e2={res['e2_1']:+.3f}m, e3={res['e3_1']:+.3f}rad")
        print(f"   Delta: |e1|={res['delta_abs_e1']:+.3f}m | |e2|={res['delta_abs_e2']:+.3f}m | |e3|={res['delta_abs_e3']:+.3f}rad")
        print(f"   Cmd Stats: First={res['omega_first']:+.3f} | Range=[{res['omega_min_signed']:+.3f}, {res['omega_max_signed']:+.3f}] | Mean={res['omega_mean_signed']:+.3f} rad/s")
        print(f"   Physical: GyroMean={res['gyro_mean_signed']:+.3f} rad/s | IMU Yaw={res['imu_yaw_deg']:+.2f} deg | EKF Yaw={res['ekf_yaw_deg']:+.2f} deg | Disp={res['pose_disp']:.3f}m\n")
        time.sleep(1.0)

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
