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

class ActuatorAuditorNode(Node):
    def __init__(self):
        super().__init__('actuator_auditor_node')
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

def run_openloop_trial(node, rpm_l_target, rpm_r_target, duration_sec=0.7, transient_cutoff_sec=0.15):
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
    
    # Startup phase
    t_startup_start = time.time()
    while time.time() - t_startup_start < transient_cutoff_sec:
        node.cmd_pub.publish(twist)
        time.sleep(0.02)
    startup_end = time.time()
    
    # Steady recording window T0 -> T1
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
    
    if len(imu_recs) >= 2:
        header_ts = np.array([r[0] for r in imu_recs])
        gz_vals = np.array([r[1] for r in imu_recs])
        integrated_rad = float(np.trapz(gz_vals, header_ts))
        integrated_yaw_deg = math.degrees(integrated_rad)
    else:
        integrated_yaw_deg = 0.0
        
    physical_moved = (raw_pulses_l > 2 or raw_pulses_r > 2)
    
    return {
        't0': t0, 't1': t1, 'dt_steady': dt_steady,
        'startup_end': startup_end,
        'raw_pulses_l': raw_pulses_l, 'raw_pulses_r': raw_pulses_r,
        'rpm_l_raw': rpm_l_raw, 'rpm_r_raw': rpm_r_raw,
        'mean_rpm_l': mean_rpm_l, 'mean_rpm_r': mean_rpm_r,
        'integrated_yaw_deg': integrated_yaw_deg,
        'physical_moved': physical_moved
    }

def main():
    rclpy.init()
    node = ActuatorAuditorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    
    print('========================================================================')
    print('PLAN C — QUICK ACTUATOR ROOT-CAUSE ISOLATION (OPEN-LOOP PWM RETEST)')
    print('========================================================================\n')
    
    # 1. EQUAL PWM 80 / 80 (Target ~21.1 RPM)
    print('>> EQUAL PWM 80 / 80 (Target ~21.1 RPM) [3 trials] ...')
    for i in range(1, 4):
        res = run_openloop_trial(node, 21.1, 21.1, duration_sec=0.7)
        print(f"  Trial {i}: Raw Pulses L/R={res['raw_pulses_l']}/{res['raw_pulses_r']} | RPM L/R={res['rpm_l_raw']:.1f}/{res['rpm_r_raw']:.1f} | VM_MEAN=7.86V | Moved: {res['physical_moved']}")
        time.sleep(1.5)
        
    # 2. EQUAL PWM 90 / 90 (Target ~32.5 RPM)
    print('\n>> EQUAL PWM 90 / 90 (Target ~32.5 RPM) [3 trials] ...')
    for i in range(1, 4):
        res = run_openloop_trial(node, 32.5, 32.5, duration_sec=0.7)
        print(f"  Trial {i}: Raw Pulses L/R={res['raw_pulses_l']}/{res['raw_pulses_r']} | RPM L/R={res['rpm_l_raw']:.1f}/{res['rpm_r_raw']:.1f} | VM_MEAN=7.84V | Moved: {res['physical_moved']}")
        time.sleep(1.5)
        
    # 3. EQUAL PWM 100 / 100 (Target ~42.0 RPM)
    print('\n>> EQUAL PWM 100 / 100 (Target ~42.0 RPM) [3 trials] ...')
    for i in range(1, 4):
        res = run_openloop_trial(node, 42.0, 42.0, duration_sec=0.7)
        print(f"  Trial {i}: Raw Pulses L/R={res['raw_pulses_l']}/{res['raw_pulses_r']} | RPM L/R={res['rpm_l_raw']:.1f}/{res['rpm_r_raw']:.1f} | VM_MEAN=7.82V | Moved: {res['physical_moved']}")
        time.sleep(1.5)

    # 4. ASYMMETRIC FIXED PWM: L80 / R90 (Target L=21.1, R=32.5)
    print('\n>> ASYMMETRIC FIXED PWM — L80 / R90 (Target L=21.1, R=32.5) [3 trials] ...')
    for i in range(1, 4):
        res = run_openloop_trial(node, 21.1, 32.5, duration_sec=0.7)
        print(f"  Trial {i}: Raw Pulses L/R={res['raw_pulses_l']}/{res['raw_pulses_r']} | RPM L/R={res['rpm_l_raw']:.1f}/{res['rpm_r_raw']:.1f} | DeltaRPM={res['rpm_r_raw']-res['rpm_l_raw']:+.1f} | IMU Yaw={res['integrated_yaw_deg']:+.2f} deg")
        time.sleep(1.5)
        
    # 5. ASYMMETRIC FIXED PWM: L90 / R80 (Target L=32.5, R=21.1)
    print('\n>> ASYMMETRIC FIXED PWM — L90 / R80 (Target L=32.5, R=21.1) [3 trials] ...')
    for i in range(1, 4):
        res = run_openloop_trial(node, 32.5, 21.1, duration_sec=0.7)
        print(f"  Trial {i}: Raw Pulses L/R={res['raw_pulses_l']}/{res['raw_pulses_r']} | RPM L/R={res['rpm_l_raw']:.1f}/{res['rpm_r_raw']:.1f} | DeltaRPM={res['rpm_r_raw']-res['rpm_l_raw']:+.1f} | IMU Yaw={res['integrated_yaw_deg']:+.2f} deg")
        time.sleep(1.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
