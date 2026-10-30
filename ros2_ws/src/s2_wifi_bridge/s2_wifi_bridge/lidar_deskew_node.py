#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

def euler_from_quaternion_yaw(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class LidarDeskewNode(Node):
    def __init__(self):
        super().__init__('lidar_deskew_node')

        # Parameters
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_deskewed')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('tf_timeout_sec', 0.08)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.tf_timeout_sec = self.get_parameter('tf_timeout_sec').value

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher & Subscriber
        self.deskewed_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.input_scan_topic, self.scan_callback, 10)

        # Performance & Diagnostic Metrics
        self.proc_times = []
        self.cnt_total = 0
        self.cnt_deskewed = 0
        self.cnt_passthrough = 0

        self.get_logger().info(
            f'LidarDeskewNode active: {self.input_scan_topic} -> {self.output_scan_topic} '
            f'(TF: {self.odom_frame} -> {self.laser_frame})'
        )

    def scan_callback(self, msg: LaserScan):
        t_start_proc = time.perf_counter()
        self.cnt_total += 1

        n_beams = len(msg.ranges)
        if n_beams == 0:
            return

        scan_dur = msg.scan_time if msg.scan_time > 0.0 else (n_beams * msg.time_increment)
        if scan_dur <= 0.0:
            scan_dur = 0.118

        t_ref = Time.from_msg(msg.header.stamp)
        t_end = t_ref + Duration(seconds=scan_dur)

        # Lookup TF at start and end of scan
        try:
            tf_ref = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id if msg.header.frame_id else self.laser_frame,
                t_ref,
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
            tf_end = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id if msg.header.frame_id else self.laser_frame,
                t_end,
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
        except Exception as ex:
            # Fallback: publish without deskewing if TF is momentarily unavailable during startup
            self.cnt_passthrough += 1
            self.deskewed_pub.publish(msg)
            return

        # Extract 2D poses: (x, y, yaw)
        q_ref = [tf_ref.transform.rotation.x, tf_ref.transform.rotation.y,
                 tf_ref.transform.rotation.z, tf_ref.transform.rotation.w]
        yaw_ref = euler_from_quaternion_yaw(q_ref)
        x_ref = tf_ref.transform.translation.x
        y_ref = tf_ref.transform.translation.y

        q_end = [tf_end.transform.rotation.x, tf_end.transform.rotation.y,
                 tf_end.transform.rotation.z, tf_end.transform.rotation.w]
        yaw_end = euler_from_quaternion_yaw(q_end)
        x_end = tf_end.transform.translation.x
        y_end = tf_end.transform.translation.y

        # Delta yaw with unwrapping
        delta_yaw = yaw_end - yaw_ref
        delta_yaw = (delta_yaw + math.pi) % (2.0 * math.pi) - math.pi
        delta_x = x_end - x_ref
        delta_y = y_end - y_ref

        # Fast vectorised deskewing via NumPy
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        if not np.any(valid_mask):
            self.deskewed_pub.publish(msg)
            return

        indices = np.arange(n_beams, dtype=np.float32)
        fractions = indices / float(n_beams - 1)

        # Beam angles in raw frame
        angles_raw = msg.angle_min + indices * msg.angle_increment

        # Interpolated pose delta for each beam relative to reference
        d_th_i = fractions * delta_yaw
        dx_odom_i = fractions * delta_x
        dy_odom_i = fractions * delta_y

        # Raw cartesian in sensor frame at t_i
        x_raw = ranges * np.cos(angles_raw)
        y_raw = ranges * np.sin(angles_raw)

        # Rotate sensor point by d_th_i, and rotate translation back into ref sensor frame
        cos_dth = np.cos(d_th_i)
        sin_dth = np.sin(d_th_i)
        x_rot = x_raw * cos_dth - y_raw * sin_dth
        y_rot = x_raw * sin_dth + y_raw * cos_dth

        cos_ref = math.cos(-yaw_ref)
        sin_ref = math.sin(-yaw_ref)
        dx_ref = dx_odom_i * cos_ref - dy_odom_i * sin_ref
        dy_ref = dx_odom_i * sin_ref + dy_odom_i * cos_ref

        x_deskewed = x_rot + dx_ref
        y_deskewed = y_rot + dy_ref

        ranges_deskewed = np.sqrt(x_deskewed**2 + y_deskewed**2)
        angles_deskewed = np.arctan2(y_deskewed, x_deskewed)

        # Normalize angles to [0, 2*pi)
        angles_deskewed = np.mod(angles_deskewed + 2.0 * math.pi, 2.0 * math.pi)

        # Re-bin into output scan
        out_ranges = np.full(n_beams, np.inf, dtype=np.float32)

        # Calculate bin indices for valid points
        valid_indices = np.where(valid_mask)[0]
        bin_indices = np.round((angles_deskewed[valid_indices] - msg.angle_min) / msg.angle_increment).astype(np.int32)
        bin_indices = np.mod(bin_indices, n_beams)

        v_ranges = ranges_deskewed[valid_indices]
        for b_idx, r_val in zip(bin_indices, v_ranges):
            if r_val < out_ranges[b_idx]:
                out_ranges[b_idx] = r_val

        # Create output LaserScan
        out_msg = LaserScan()
        out_msg.header = msg.header  # Preserves t_ref and frame_id
        out_msg.angle_min = msg.angle_min
        out_msg.angle_max = msg.angle_max
        out_msg.angle_increment = msg.angle_increment
        out_msg.time_increment = msg.time_increment
        out_msg.scan_time = msg.scan_time
        out_msg.range_min = msg.range_min
        out_msg.range_max = msg.range_max
        out_msg.ranges = out_ranges.tolist()
        if len(msg.intensities) > 0:
            out_msg.intensities = msg.intensities

        self.deskewed_pub.publish(out_msg)
        self.cnt_deskewed += 1

        t_proc = (time.perf_counter() - t_start_proc) * 1000.0  # ms
        self.proc_times.append(t_proc)
        if len(self.proc_times) > 100:
            self.proc_times.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDeskewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
