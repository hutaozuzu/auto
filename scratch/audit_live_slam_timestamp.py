import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import rclpy.time
import rclpy.duration
import time
import numpy as np
import threading

class LiveSlamAuditor(Node):
    def __init__(self):
        super().__init__('live_slam_auditor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 50)
        self.lock = threading.Lock()
        self.scan_records = []

    def scan_cb(self, msg):
        host_now = self.get_clock().now()
        host_ns = host_now.nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        offset_ms = (host_ns - stamp_ns) / 1e6

        # Test: map -> laser_frame at exact scan timestamp
        map_laser_success = False
        map_laser_age_ms = 0.0
        map_laser_err = ""
        try:
            target = rclpy.time.Time(nanoseconds=stamp_ns)
            t = self.tf_buffer.lookup_transform(
                'map', 'laser_frame', target,
                timeout=rclpy.duration.Duration(seconds=0.1))
            map_laser_success = True
            t_stamp_ns = t.header.stamp.sec * 10**9 + t.header.stamp.nanosec
            map_laser_age_ms = (stamp_ns - t_stamp_ns) / 1e6
        except tf2_ros.LookupException as e:
            map_laser_err = f"Lookup: {e}"
        except tf2_ros.ExtrapolationException as e:
            map_laser_err = f"Extrapolation: {e}"
        except Exception as e:
            map_laser_err = f"Other: {e}"

        # Test: odom -> base_link at exact scan timestamp
        odom_bl_success = False
        try:
            target = rclpy.time.Time(nanoseconds=stamp_ns)
            self.tf_buffer.lookup_transform(
                'odom', 'base_link', target,
                timeout=rclpy.duration.Duration(seconds=0.05))
            odom_bl_success = True
        except Exception:
            pass

        with self.lock:
            self.scan_records.append({
                'frame_id': msg.header.frame_id,
                'host_ns': host_ns,
                'stamp_ns': stamp_ns,
                'offset_ms': offset_ms,
                'scan_time_ms': msg.scan_time * 1000.0,
                'time_increment_us': msg.time_increment * 1e6,
                'ranges_count': len(msg.ranges),
                'map_laser_ok': map_laser_success,
                'map_laser_age_ms': map_laser_age_ms,
                'map_laser_err': map_laser_err,
                'odom_bl_ok': odom_bl_success,
            })

def main():
    rclpy.init()
    node = LiveSlamAuditor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('========================================================================')
    print('CRITICAL LIVE-SLAM AUDIT — LASERSCAN TIMESTAMP & TF CHAIN')
    print('========================================================================\n')
    print('>> Collecting 30+ scans for timestamp and TF audit ...')
    t0 = time.time()
    while True:
        with node.lock:
            n = len(node.scan_records)
        if n >= 35 or time.time() - t0 > 20.0:
            break
        time.sleep(0.2)

    with node.lock:
        scans = list(node.scan_records)

    if not scans:
        print('>> ERROR: No /scan messages received. Aborting.')
        node.destroy_node()
        rclpy.shutdown()
        return

    offsets = np.array([s['offset_ms'] for s in scans])
    map_ok = [s['map_laser_ok'] for s in scans]
    odom_ok = [s['odom_bl_ok'] for s in scans]
    
    print(f">> Scans collected:         {len(scans)}")
    print(f">> frame_id:                '{scans[-1]['frame_id']}'")
    print(f">> ranges_count:            {scans[-1]['ranges_count']} bins")
    print(f">> scan_time:               {scans[-1]['scan_time_ms']:.2f} ms")
    print(f">> time_increment:          {scans[-1]['time_increment_us']:.2f} us/bin\n")

    print('--- HOST-SCAN CLOCK OFFSET ANALYSIS ---')
    print(f">> Mean Offset:             {np.mean(offsets):+.2f} ms")
    print(f">> Min Offset:              {np.min(offsets):+.2f} ms")
    print(f">> Max Offset:              {np.max(offsets):+.2f} ms")
    print(f">> Std (Jitter):            {np.std(offsets):.2f} ms")
    print(f">> Range (Max-Min):         {np.max(offsets)-np.min(offsets):.2f} ms")
    
    # Classify timestamp
    if abs(np.mean(offsets)) < 50.0 and np.std(offsets) < 20.0:
        print(f">> CLOCK SYNC STATUS:       GOOD (mean offset < 50ms, jitter < 20ms)")
    elif abs(np.mean(offsets)) < 500.0 and np.std(offsets) < 50.0:
        print(f">> CLOCK SYNC STATUS:       ACCEPTABLE (offset large but stable - micro-ROS synchronized)")
    else:
        print(f">> CLOCK SYNC STATUS:       *** FLAG: TIMESTAMP DRIFT DETECTED ***")

    print(f"\n--- TF CHAIN AVAILABILITY AT SCAN STAMP ---")
    map_sr = sum(map_ok) / len(map_ok) * 100
    odom_sr = sum(odom_ok) / len(odom_ok) * 100
    print(f">> map->laser_frame at stamp: {map_sr:.1f}% SUCCESS ({sum(map_ok)}/{len(map_ok)})")
    print(f">> odom->base_link at stamp:  {odom_sr:.1f}% SUCCESS ({sum(odom_ok)}/{len(odom_ok)})")

    # Print first few errors if any
    errors = [(s['offset_ms'], s['map_laser_err']) for s in scans if not s['map_laser_ok']]
    if errors:
        print(f"\n>> FAILED TF LOOKUPS (first 3):")
        for offset, err in errors[:3]:
            print(f"   offset={offset:+.1f}ms | {err}")

    # Check stamp continuity (detect frozen/repeating stamps)
    stamp_diffs_ms = np.diff(np.array([s['stamp_ns'] for s in scans])) / 1e6
    print(f"\n--- SCAN STAMP CONTINUITY ---")
    print(f">> Mean inter-scan stamp dt: {np.mean(stamp_diffs_ms):.2f} ms")
    print(f">> Std inter-scan stamp dt:  {np.std(stamp_diffs_ms):.2f} ms")
    frozen = np.sum(stamp_diffs_ms < 1.0)
    print(f">> Frozen/repeated stamps:   {frozen} occurrences")

    # Check if stamps are monotonically increasing
    mono = np.all(np.diff(np.array([s['stamp_ns'] for s in scans])) > 0)
    print(f">> Stamps monotonic:         {'YES' if mono else '*** NO - CLOCK REGRESSION ***'}")

    # Summary
    print('\n========================================================================')
    print('DIAGNOSIS SUMMARY')
    print('========================================================================')
    print(f"SCAN FRAME:                 {scans[-1]['frame_id']}")
    
    if np.std(offsets) < 50.0 and abs(np.mean(offsets)) < 500.0:
        print(f"SCAN STAMP SOURCE:          micro-ROS synchronized epoch (HEALTHY)")
        print(f"MICRO-ROS CLOCK SYNC:       YES")
    else:
        print(f"SCAN STAMP SOURCE:          *** UNSYNCHRONIZED / DRIFTING ***")
        print(f"MICRO-ROS CLOCK SYNC:       NO - FLAG")

    print(f"HOST-SCAN TIME OFFSET:      mean={np.mean(offsets):+.2f} ms, std={np.std(offsets):.2f} ms")
    print(f"ODOM->BASE_LINK DYNAMIC:    {'YES' if odom_sr > 95 else 'PARTIAL/FAIL'}")
    print(f"MAP->LASER_FRAME DYNAMIC:   {'YES' if map_sr > 95 else f'PARTIAL/FAIL ({map_sr:.0f}%)'}")
    print(f"TF AVAILABLE AT SCAN STAMP: {'YES' if map_sr > 95 else 'NO'}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
