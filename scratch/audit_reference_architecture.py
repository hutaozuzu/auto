import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import tf2_ros
import time
import numpy as np
import threading

class ReferenceArchitectureAuditor(Node):
    def __init__(self):
        super().__init__('ref_arch_auditor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        
        self.lock = threading.Lock()
        self.scan_records = []
        self.odom_times = []
        self.ekf_times = []
        self.imu_times = []

    def scan_cb(self, msg):
        host_now = self.get_clock().now()
        host_time_sec = host_now.nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        offset_sec = host_time_sec - stamp_sec
        
        # Test TF lookup at exact scan timestamp
        tf_success = False
        tf_age = 0.0
        try:
            target_time = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            t = self.tf_buffer.lookup_transform('odom', 'base_link', target_time, timeout=rclpy.duration.Duration(seconds=0.05))
            tf_success = True
            t_stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tf_age = stamp_sec - t_stamp
        except Exception as e:
            tf_success = False
            
        with self.lock:
            self.scan_records.append({
                'host_time': host_time_sec,
                'stamp_sec': stamp_sec,
                'offset_sec': offset_sec,
                'scan_time': msg.scan_time,
                'time_increment': msg.time_increment,
                'ranges_count': len(msg.ranges),
                'tf_success': tf_success,
                'tf_age': tf_age
            })

    def odom_cb(self, msg):
        with self.lock:
            self.odom_times.append(time.time())

    def ekf_cb(self, msg):
        with self.lock:
            self.ekf_times.append(time.time())

    def imu_cb(self, msg):
        with self.lock:
            self.imu_times.append(time.time())

def main():
    rclpy.init()
    node = ReferenceArchitectureAuditor()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print('========================================================================')
    print('PLAN C — REFERENCE ARCHITECTURE & TIMESTAMP AUDIT')
    print('========================================================================\n')
    
    print('>> Collecting live topic and TF statistics for 10 seconds ...')
    t0 = time.time()
    time.sleep(10.0)
    
    with node.lock:
        scans = list(node.scan_records)
        odom_ts = list(node.odom_times)
        ekf_ts = list(node.ekf_times)
        imu_ts = list(node.imu_times)
        
    dur = time.time() - t0
    
    scan_rate = len(scans) / dur if dur > 0 else 0.0
    odom_rate = len(odom_ts) / dur if dur > 0 else 0.0
    ekf_rate = len(ekf_ts) / dur if dur > 0 else 0.0
    imu_rate = len(imu_ts) / dur if dur > 0 else 0.0
    
    print(f">> Measured Topic Rates over {dur:.1f}s:")
    print(f"   /scan rate:             {scan_rate:.1f} Hz (Total: {len(scans)})")
    print(f"   /odom rate:             {odom_rate:.1f} Hz (Total: {len(odom_ts)})")
    print(f"   /odometry/filtered:     {ekf_rate:.1f} Hz (Total: {len(ekf_ts)})")
    print(f"   /imu rate:              {imu_rate:.1f} Hz (Total: {len(imu_ts)})\n")
    
    if scans:
        offsets = [s['offset_sec'] for s in scans]
        tf_successes = [s['tf_success'] for s in scans]
        tf_success_rate = (sum(tf_successes) / len(tf_successes)) * 100.0
        
        print('>> Scan Timestamp & TF Synchronization Analysis:')
        print(f"   Mean Host-Scan Offset:  {np.mean(offsets)*1000.0:+.2f} ms")
        print(f"   Min Offset:             {np.min(offsets)*1000.0:+.2f} ms")
        print(f"   Max Offset:             {np.max(offsets)*1000.0:+.2f} ms")
        print(f"   Jitter (Std):           {np.std(offsets)*1000.0:.2f} ms")
        print(f"   Scan Time (header):     {scans[-1]['scan_time']*1000.0:.2f} ms")
        print(f"   Time Increment:         {scans[-1]['time_increment']*1e6:.2f} us")
        print(f"   Ranges Bins:            {scans[-1]['ranges_count']}")
        print(f"   TF Lookup at Stamp:     {tf_success_rate:.1f}% SUCCESS ({sum(tf_successes)}/{len(tf_successes)})\n")
    else:
        print('>> WARNING: No /scan messages received during audit window!')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
