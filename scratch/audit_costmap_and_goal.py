#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from nav_msgs.msg import Path, Odometry
from tf2_ros import Buffer, TransformListener
import time
import math
import numpy as np
import threading

class CostmapAndGoalAuditor(Node):
    def __init__(self):
        super().__init__('costmap_and_goal_auditor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 50)
        self.sub_err = self.create_subscription(Vector3Stamped, '/FollowPath/tracking_error', self.err_cb, 50)
        self.sub_ref = self.create_subscription(PoseStamped, '/FollowPath/reference_pose', self.ref_cb, 50)
        self.sub_plan = self.create_subscription(Path, '/plan', self.plan_cb, 10)

        self.last_cmd = Twist()
        self.last_err = Vector3Stamped()
        self.last_ref = PoseStamped()
        self.last_plan = Path()
        self.lock = threading.Lock()

    def cmd_cb(self, msg):
        with self.lock: self.last_cmd = msg
    def err_cb(self, msg):
        with self.lock: self.last_err = msg
    def ref_cb(self, msg):
        with self.lock: self.last_ref = msg
    def plan_cb(self, msg):
        with self.lock: self.last_plan = msg

def main():
    rclpy.init()
    node = CostmapAndGoalAuditor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("==========================================================================")
    print("TF & GOAL STOP AUDITOR RUNNING — LOGGING TF & FINAL 30CM METRICS")
    print("==========================================================================")

    # 1. Audit TF Rates & Jitter for 5 seconds
    print(">> Phase 1: Measuring TF map->odom and odom->base_link jitter (5s)...")
    map_odom_samples = []
    odom_base_samples = []
    
    t0 = time.time()
    while time.time() - t0 < 5.0:
        try:
            t_mo = node.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            map_odom_samples.append((
                time.time(),
                t_mo.transform.translation.x,
                t_mo.transform.translation.y,
                2.0 * math.atan2(t_mo.transform.rotation.z, t_mo.transform.rotation.w)
            ))
        except Exception:
            pass

        try:
            t_ob = node.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            odom_base_samples.append((
                time.time(),
                t_ob.transform.translation.x,
                t_ob.transform.translation.y,
                2.0 * math.atan2(t_ob.transform.rotation.z, t_ob.transform.rotation.w)
            ))
        except Exception:
            pass
        time.sleep(0.05)

    print(f"  map->odom TF samples:    {len(map_odom_samples)}")
    print(f"  odom->base_link TF samples: {len(odom_base_samples)}")

    if len(map_odom_samples) > 1:
        mo_arr = np.array(map_odom_samples)
        dx_mo = np.diff(mo_arr[:, 1])
        dy_mo = np.diff(mo_arr[:, 2])
        dyaw_mo = np.degrees(np.diff(mo_arr[:, 3]))
        print(f"  map->odom max trans jump: {np.max(np.hypot(dx_mo, dy_mo))*100:.3f} cm | max yaw jump: {np.max(np.abs(dyaw_mo)):.3f}°")

    if len(odom_base_samples) > 1:
        ob_arr = np.array(odom_base_samples)
        dx_ob = np.diff(ob_arr[:, 1])
        dy_ob = np.diff(ob_arr[:, 2])
        dyaw_ob = np.degrees(np.diff(ob_arr[:, 3]))
        print(f"  odom->base_link max trans jump: {np.max(np.hypot(dx_ob, dy_ob))*100:.3f} cm | max yaw jump: {np.max(np.abs(dyaw_ob)):.3f}°")

    print("\n>> Phase 2: Ready for Goal approach test. Waiting for /cmd_vel...")
    print("Time     | Dist to Goal | e1 (cm)  | e2 (cm)  | e3 (deg) | Cmd Vx   | Cmd Wz")
    print("--------------------------------------------------------------------------")

    try:
        while rclpy.ok():
            with node.lock:
                cmd = node.last_cmd
                err = node.last_err
                ref = node.last_ref
                plan = node.last_plan

            # Calculate distance to goal in odom
            dist_goal = -1.0
            if plan.poses:
                try:
                    t_ob = node.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                    t_mo = node.tf_buffer.lookup_transform('odom', plan.header.frame_id, rclpy.time.Time())
                    rx = t_ob.transform.translation.x
                    ry = t_ob.transform.translation.y
                    # Goal point transformed to odom
                    gx = plan.poses[-1].pose.position.x
                    gy = plan.poses[-1].pose.position.y
                    # simplified transform
                    dist_goal = math.hypot(gx - rx, gy - ry)
                except Exception:
                    pass

            if abs(cmd.linear.x) > 0.001 or abs(cmd.angular.z) > 0.001:
                e1 = err.vector.x * 100
                e2 = err.vector.y * 100
                e3 = math.degrees(err.vector.z)
                print(f"{time.strftime('%H:%M:%S')} | {dist_goal:6.2f}m     | {e1:+6.2f}cm | {e2:+6.2f}cm | {e3:+7.2f}° | {cmd.linear.x:+5.2f}m/s | {cmd.angular.z:+5.2f}r/s")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
