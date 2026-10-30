#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
import time
import math
import numpy as np
import threading

class PathTrackingAuditor(Node):
    def __init__(self):
        super().__init__('path_tracking_auditor')
        self.sub_err = self.create_subscription(
            Vector3Stamped, '/FollowPath/tracking_error', self.err_cb, 50)
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 50)
        self.sub_rpm_l = self.create_subscription(
            Float32, '/wheel_rpm_left', self.rpml_cb, 50)
        self.sub_rpm_r = self.create_subscription(
            Float32, '/wheel_rpm_right', self.rpmr_cb, 50)

        self.records = []
        self.lock = threading.Lock()
        self.last_rpml = 0.0
        self.last_rpmr = 0.0
        self.last_vx = 0.0
        self.last_wz = 0.0

    def err_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        e1 = msg.vector.x
        e2 = msg.vector.y
        e3 = msg.vector.z
        with self.lock:
            self.records.append({
                't': t,
                'e1': e1,
                'e2': e2,
                'e3_deg': math.degrees(e3),
                'vx': self.last_vx,
                'wz': self.last_wz,
                'rpm_l': self.last_rpml,
                'rpm_r': self.last_rpmr
            })

    def cmd_cb(self, msg):
        with self.lock:
            self.last_vx = msg.linear.x
            self.last_wz = msg.angular.z

    def rpml_cb(self, msg):
        with self.lock:
            self.last_rpml = msg.data

    def rpmr_cb(self, msg):
        with self.lock:
            self.last_rpmr = msg.data

def main():
    import sys
    route_name = sys.argv[1] if len(sys.argv) > 1 else "ROUTE"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    rclpy.init()
    node = PathTrackingAuditor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f"\n==========================================================================")
    print(f"PATH TRACKING LOGGER ({route_name}) — LISTENING FOR /cmd_vel & /tracking_error")
    print(f"==========================================================================")
    print(f"Time     | e1 (cm)  | e2 (cm)  | e3 (deg) | Cmd Vx   | Cmd Wz   | RPM L/R")
    print(f"--------------------------------------------------------------------------")

    t_start = time.time()
    last_count = 0
    while time.time() - t_start < duration:
        with node.lock:
            if len(node.records) > last_count:
                r = node.records[-1]
                last_count = len(node.records)
                print(f"{time.strftime('%H:%M:%S')} | {r['e1']*100:+6.2f}cm | {r['e2']*100:+6.2f}cm | {r['e3_deg']:+7.2f}° | {r['vx']:+5.2f}m/s | {r['wz']:+5.2f}r/s | L:{r['rpm_l']:+5.1f} R:{r['rpm_r']:+5.1f}")
        time.sleep(0.2)

    with node.lock:
        recs = list(node.records)

    node.destroy_node()
    rclpy.shutdown()

    if not recs:
        print(f"\n>> No tracking error received. Did the robot execute a goal?")
        return

    e2_arr = np.array([r['e2'] for r in recs])
    e3_arr = np.array([r['e3_deg'] for r in recs])
    wz_arr = np.array([r['wz'] for r in recs])

    # Count sign changes in e2
    signs = np.sign(e2_arr[np.abs(e2_arr) > 0.01])
    sign_changes = np.sum(np.diff(signs) != 0) if len(signs) > 1 else 0

    print(f"\n==========================================================================")
    print(f"PATH TRACKING SUMMARY FOR {route_name}:")
    print(f"  Samples recorded:       {len(recs)}")
    print(f"  Mean Cross-Track (|e2|): {np.mean(np.abs(e2_arr))*100:.2f} cm")
    print(f"  Max Cross-Track (|e2|):  {np.max(np.abs(e2_arr))*100:.2f} cm")
    print(f"  RMS Cross-Track (e2):   {np.sqrt(np.mean(e2_arr**2))*100:.2f} cm")
    print(f"  Mean Heading (|e3|):     {np.mean(np.abs(e3_arr)):.2f} deg")
    print(f"  Max Heading (|e3|):      {np.max(np.abs(e3_arr)):.2f} deg")
    print(f"  Peak Angular Vel (|wz|): {np.max(np.abs(wz_arr)):.2f} rad/s")
    print(f"  e2 Sign Crossings:       {sign_changes}")
    print(f"==========================================================================")

if __name__ == '__main__':
    main()
