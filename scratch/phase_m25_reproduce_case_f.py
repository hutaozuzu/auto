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

def calc_omega_wheels(rpm_l_signed, rpm_r_signed, r=WHEEL_RADIUS_M, L=WHEEL_SEPARATION_M):
    w_l = (rpm_l_signed * 2.0 * math.pi) / 60.0
    w_r = (rpm_r_signed * 2.0 * math.pi) / 60.0
    return (r / L) * (w_r - w_l)

class CaseFReproducerNode(Node):
    def __init__(self):
        super().__init__('case_f_reproducer')
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

    def run_case_f_trial(self, trial_num, duration_sec=3.0):
        self.stop_robot(1.0)
        self.gyro_z_history.clear()
        self.rpm_l_history.clear()
        self.rpm_r_history.clear()
        
        start_odom_x = self.last_odom_x
        start_odom_y = self.last_odom_y
        
        twist = Twist()
        twist.linear.x = 0.10
        twist.angular.z = -0.40
        
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
        omega_wheels = calc_omega_wheels(mean_rpm_l, mean_rpm_r)
        
        print(f"  Trial {trial_num}: L={mean_rpm_l:+.1f} RPM, R={mean_rpm_r:+.1f} RPM | Omega: Cmd=-0.400, Wheels={omega_wheels:+.3f}, GyroMean={gz_mean:+.3f} rad/s | Yaw={math.degrees(integrated_yaw):+.2f} deg | Disp={disp:.3f}m")
        return {
            'mean_rpm_l': mean_rpm_l, 'mean_rpm_r': mean_rpm_r,
            'omega_wheels': omega_wheels, 'gz_mean': gz_mean,
            'yaw_deg': math.degrees(integrated_yaw), 'disp': disp
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
    node = CaseFReproducerNode()
    print('========================================================================')
    print('PLAN C — PHASE M2.5 CASE F REPRODUCTION (3 INDEPENDENT TRIALS)')
    print('Command: v = 0.10 m/s, w = -0.40 rad/s (Target L=+35.8 RPM, R=+22.9 RPM)')
    print('========================================================================\n')
    
    trials = []
    for i in range(1, 4):
        res = node.run_case_f_trial(i)
        trials.append(res)
        time.sleep(1.0)
        
    node.stop_robot(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
