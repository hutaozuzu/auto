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

class QuickCurveCheckerM26CNode(Node):
    def __init__(self):
        super().__init__('phase_m26c_quick_curves')
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

    def run_curve_trial(self, v_cmd, w_cmd, duration_sec=0.8, transient_cutoff_sec=0.25, rearm_pause_sec=2.0):
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
            
        encoder_l = float(np.mean(rl_arr[steady_mask]))
        encoder_r = float(np.mean(rr_arr[steady_mask]))
        gz_mean = float(np.mean(gz_arr[steady_mask]))
        
        omega_encoder_nom = calc_omega_wheels(encoder_l, encoder_r, L=L_PHYSICAL_M)
        omega_encoder_eff = calc_omega_wheels(encoder_l, encoder_r, L=L_EFFECTIVE_M)
        omega_physical = (integrated_yaw / duration_sec) if duration_sec > 0.0 else gz_mean
        
        total_disp = math.hypot(ox_arr[-1] - ox_arr[0], oy_arr[-1] - oy_arr[0])
        
        return {
            'v_cmd': v_cmd, 'w_cmd': w_cmd,
            'encoder_l': encoder_l, 'encoder_r': encoder_r,
            'omega_nom': omega_encoder_nom,
            'omega_eff': omega_encoder_eff,
            'gz_mean': gz_mean,
            'omega_physical': omega_physical,
            'g_phys': (omega_physical / w_cmd) if abs(w_cmd) > 0.01 else 1.0,
            'yaw_deg': math.degrees(integrated_yaw),
            'disp': total_disp
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
    node = QuickCurveCheckerM26CNode()
    
    print('========================================================================')
    print('PLAN C — PHASE M2.6C QUICK CURVE CHECK (0.8s COMPACT)')
    print('========================================================================\n')
    
    curves = [
        ('+.20', 0.10, +0.20),
        ('-.20', 0.10, -0.20),
        ('+.30', 0.10, +0.30),
        ('-.30', 0.10, -0.30),
        ('+.40', 0.10, +0.40),
        ('-.40', 0.10, -0.40)
    ]
    for label, v, w in curves:
        print(f">> Executing Curve {label} ...")
        res = node.run_curve_trial(v, w, duration_sec=0.8, rearm_pause_sec=2.0)
        print(f"   [{label}]: Encoder L={res['encoder_l']:+.1f}, R={res['encoder_r']:+.1f}")
        print(f"       Omega Nom: {res['omega_nom']:+.3f} rad/s | Omega Eff: {res['omega_eff']:+.3f} rad/s")
        print(f"       Omega IMU: {res['gz_mean']:+.3f} rad/s | Omega Phys: {res['omega_physical']:+.3f} rad/s (G_phys={res['g_phys']:.2f})")
        print(f"       Yaw: {res['yaw_deg']:+.2f} deg | Disp: {res['disp']:.3f}m")
        time.sleep(1.0)

    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
