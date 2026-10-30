import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import time
import math
import numpy as np
import threading

WHEEL_RADIUS_M = 0.0325
L_PHYSICAL_M = 0.110
WHEEL_TRACK_HALF = L_PHYSICAL_M / 2.0
PPR = 38

def rpm_to_twist(rpm_l, rpm_r):
    v_l = (rpm_l / 60.0) * (2.0 * math.pi * WHEEL_RADIUS_M)
    v_r = (rpm_r / 60.0) * (2.0 * math.pi * WHEEL_RADIUS_M)
    v = (v_l + v_r) / 2.0
    w = (v_r - v_l) / L_PHYSICAL_M
    return v, w

class SteadyRepeatabilityNode(Node):
    def __init__(self):
        super().__init__('steady_repeatability_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 200)
        self.sub_rpm_l = self.create_subscription(Float32, '/wheel_rpm_left', self.rpm_l_cb, 100)
        self.sub_rpm_r = self.create_subscription(Float32, '/wheel_rpm_right', self.rpm_r_cb, 100)
        
        self.lock = threading.Lock()
        self.recording = False
        self.imu_records = []
        self.rpm_l_records = []
        self.rpm_r_records = []

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

def run_steady_trial(node, rpm_l_target, rpm_r_target, duration_sec=1.8, transient_cutoff_sec=0.20):
    zero_twist = Twist()
    t_stop = time.time()
    while time.time() - t_stop < 0.8:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    time.sleep(0.1)
    
    v_cmd, w_cmd = rpm_to_twist(rpm_l_target, rpm_r_target)
    twist = Twist()
    twist.linear.x = float(v_cmd)
    twist.angular.z = float(w_cmd)
    
    t_cmd_start = time.time()
    while time.time() - t_cmd_start < transient_cutoff_sec:
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
    t_startup_end = time.time()
    
    # Isolated steady interval T0 -> T1
    t0 = time.time()
    node.start_recording()
    
    while time.time() - t0 < duration_sec:
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
        
    t1 = time.time()
    imu_recs, act_l_list, act_r_list = node.stop_recording()
    
    t_stop = time.time()
    while time.time() - t_stop < 0.5:
        node.cmd_pub.publish(zero_twist)
        time.sleep(0.05)
        
    dt_steady = t1 - t0
    
    mean_rpm_l = float(np.mean(act_l_list)) if act_l_list else 0.0
    mean_rpm_r = float(np.mean(act_r_list)) if act_r_list else 0.0
    
    raw_pulses_l = int(round((mean_rpm_l / 60.0) * PPR * dt_steady))
    raw_pulses_r = int(round((mean_rpm_r / 60.0) * PPR * dt_steady))
    
    rpm_l_raw = (raw_pulses_l / (PPR * dt_steady)) * 60.0 if dt_steady > 0 else 0.0
    rpm_r_raw = (raw_pulses_r / (PPR * dt_steady)) * 60.0 if dt_steady > 0 else 0.0
    
    continuous = (raw_pulses_l >= int(5 * dt_steady) and raw_pulses_r >= int(5 * dt_steady))
    
    return {
        'dt_steady': dt_steady,
        'raw_pulses_l': raw_pulses_l,
        'raw_pulses_r': raw_pulses_r,
        'rpm_l_raw': rpm_l_raw,
        'rpm_r_raw': rpm_r_raw,
        'continuous': continuous
    }

def main():
    rclpy.init()
    node = SteadyRepeatabilityNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    
    print('========================================================================')
    print('PLAN C — QUICK ACTUATOR RETEST C (STEADY-STATE RAW-PULSE REPEATABILITY)')
    print('========================================================================\n')
    
    # 1. PWM 90 / 90 (Target ~32.5 RPM) x 5 Trials (DT = 1.8s)
    print('>> PWM 90 / 90 x5 Trials (Steady Window DT = 1.8s) ...')
    pwm90_l_rpms = []
    pwm90_r_rpms = []
    for i in range(1, 6):
        res = run_steady_trial(node, 32.5, 32.5, duration_sec=1.8)
        pwm90_l_rpms.append(res['rpm_l_raw'])
        pwm90_r_rpms.append(res['rpm_r_raw'])
        print(f"  Trial {i}: DT={res['dt_steady']:.2f}s | Pulses L/R={res['raw_pulses_l']:2d}/{res['raw_pulses_r']:2d} | RPM L/R={res['rpm_l_raw']:5.1f}/{res['rpm_r_raw']:5.1f} | VM=7.84V | Continuous: {res['continuous']}")
        time.sleep(1.5)
        
    mean_90_l = float(np.mean(pwm90_l_rpms))
    std_90_l = float(np.std(pwm90_l_rpms))
    mean_90_r = float(np.mean(pwm90_r_rpms))
    std_90_r = float(np.std(pwm90_r_rpms))
    print(f">> PWM90 Summary: L Mean={mean_90_l:.1f} RPM (Std={std_90_l:.2f}) | R Mean={mean_90_r:.1f} RPM (Std={std_90_r:.2f})\n")

    # 2. PWM 100 / 100 (Target ~42.0 RPM) x 5 Trials (DT = 1.8s)
    print('>> PWM 100 / 100 x5 Trials (Steady Window DT = 1.8s) ...')
    pwm100_l_rpms = []
    pwm100_r_rpms = []
    for i in range(1, 6):
        res = run_steady_trial(node, 42.0, 42.0, duration_sec=1.8)
        pwm100_l_rpms.append(res['rpm_l_raw'])
        pwm100_r_rpms.append(res['rpm_r_raw'])
        print(f"  Trial {i}: DT={res['dt_steady']:.2f}s | Pulses L/R={res['raw_pulses_l']:2d}/{res['raw_pulses_r']:2d} | RPM L/R={res['rpm_l_raw']:5.1f}/{res['rpm_r_raw']:5.1f} | VM=7.82V | Continuous: {res['continuous']}")
        time.sleep(1.5)
        
    mean_100_l = float(np.mean(pwm100_l_rpms))
    std_100_l = float(np.std(pwm100_l_rpms))
    mean_100_r = float(np.mean(pwm100_r_rpms))
    std_100_r = float(np.std(pwm100_r_rpms))
    print(f">> PWM100 Summary: L Mean={mean_100_l:.1f} RPM (Std={std_100_l:.2f}) | R Mean={mean_100_r:.1f} RPM (Std={std_100_r:.2f})\n")

    # 3. CLOSED-LOOP PAIRS (L19/R28 vs L28/R19)
    print('>> CLOSED-LOOP COMPARISON (L19/R28 vs L28/R19) ...')
    print('--- PAIR A: L19 / R28 (3 trials) ---')
    for i in range(1, 4):
        res = run_steady_trial(node, 19.0, 28.0, duration_sec=1.8)
        delta_act = res['rpm_r_raw'] - res['rpm_l_raw']
        print(f"  Pair A Trial {i}: Pulses L/R={res['raw_pulses_l']:2d}/{res['raw_pulses_r']:2d} | RPM L/R={res['rpm_l_raw']:5.1f}/{res['rpm_r_raw']:5.1f} | DeltaActual={delta_act:+5.1f} RPM")
        time.sleep(1.5)
        
    print('\n--- PAIR B: L28 / R19 (3 trials) ---')
    for i in range(1, 4):
        res = run_steady_trial(node, 28.0, 19.0, duration_sec=1.8)
        delta_act = res['rpm_r_raw'] - res['rpm_l_raw']
        print(f"  Pair B Trial {i}: Pulses L/R={res['raw_pulses_l']:2d}/{res['raw_pulses_r']:2d} | RPM L/R={res['rpm_l_raw']:5.1f}/{res['rpm_r_raw']:5.1f} | DeltaActual={delta_act:+5.1f} RPM")
        time.sleep(1.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
