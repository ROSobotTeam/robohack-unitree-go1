#!/usr/bin/env python3
"""
Go1 odometry experiment: IMU yaw + robust LiDAR point-to-line translation matching.

Publishes /odom and odom->trunk TF at 20 Hz (sim time when enabled).
No pip installs required beyond existing NumPy/SciPy in the ROS environment.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial import cKDTree


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                     1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def R2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def make_scan(scan, stride=2):
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    ids = np.arange(0, len(ranges), stride)
    r = ranges[ids]
    a = scan.angle_min + ids * scan.angle_increment
    good = np.isfinite(r) & (r > max(scan.range_min, 0.15)) & (r < min(scan.range_max, 18.0))
    r, a, ids = r[good], a[good], ids[good]
    pts = np.column_stack((r*np.cos(a), r*np.sin(a)))
    # Normals from scan-order adjacent points, reject edges/discontinuities.
    normals = np.full_like(pts, np.nan)
    if len(pts) >= 3:
        # This is only a rough normal field; discontinuities are rejected by distance later.
        tangents = np.zeros_like(pts)
        tangents[1:-1] = pts[2:] - pts[:-2]
        tangents[0] = pts[1] - pts[0]
        tangents[-1] = pts[-1] - pts[-2]
        norm = np.linalg.norm(tangents, axis=1)
        valid = norm > 1e-5
        tangents[valid] /= norm[valid, None]
        normals[valid] = np.column_stack((-tangents[valid, 1], tangents[valid, 0]))
    return pts, normals


def point_line_translation(curr, prev_pts, prev_normals, yaw_delta,
                           iterations=6, max_pair_dist=0.35):
    """
    With yaw delta fixed from IMU, estimate translation t such that
    prev ~= R(yaw_delta) curr + t, using robust point-to-line residuals.
    """
    if len(curr) < 60 or len(prev_pts) < 60:
        return None
    valid_norm = np.isfinite(prev_normals[:, 0])
    target_pts = prev_pts[valid_norm]
    target_normals = prev_normals[valid_norm]
    if len(target_pts) < 50:
        return None
    tree = cKDTree(target_pts)
    rotated = curr @ R2(yaw_delta).T
    t = np.zeros(2, dtype=np.float64)
    for _ in range(iterations):
        moved = rotated + t
        d, ix = tree.query(moved, k=1)
        keep = (d < max_pair_dist)
        if keep.sum() < 50:
            return None
        A = moved[keep]
        B = target_pts[ix[keep]]
        N = target_normals[ix[keep]]
        # Point-to-line residual: n dot (A + dt - B) = 0.
        residual = np.sum(N * (B - A), axis=1)
        # Reject residual outliers robustly.
        med = np.median(residual)
        mad = np.median(np.abs(residual - med)) + 1e-6
        robust = np.abs(residual - med) < max(0.025, 2.5 * 1.4826 * mad)
        A_mat = N[robust]
        b_vec = residual[robust]
        if len(b_vec) < 40:
            return None
        # Weighted least squares; downweight residuals beyond Huber threshold.
        weights = np.ones_like(b_vec)
        huber = 0.04
        large = np.abs(b_vec) > huber
        weights[large] = huber / np.abs(b_vec[large])
        Aw = A_mat * weights[:, None]
        H = A_mat.T @ Aw
        g = A_mat.T @ (weights * b_vec)
        if np.linalg.cond(H) > 1e4:
            return None  # geometry doesn't constrain both translation axes
        dt = np.linalg.solve(H, g)
        t += dt
        if np.linalg.norm(dt) < 0.0015:
            break
    # Validate on final nearest-neighbour pairs and physical plausibility.
    moved = rotated + t
    d, ix = tree.query(moved, k=1)
    inliers = d < 0.22
    if inliers.sum() < 50:
        return None
    median_dist = float(np.median(d[inliers]))
    # A Go1 commanded at 0.2 m/s and scan period ~0.067 s should move
    # ~1.3 cm between scans; permit larger motion for dynamics but reject jumps.
    if median_dist > 0.10 or np.linalg.norm(t) > 0.10:
        return None
    # Check translation solution observability: both directions must be constrained.
    eig = np.linalg.eigvalsh(H)
    if eig[0] <= 1e-8 or eig[1] / eig[0] > 1e4:
        return None
    return t, median_dist, int(inliers.sum())


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('lidar_offset_x', 0.22)
        self.declare_parameter('max_translation_per_scan', 0.10)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.lidar_lever = np.array([float(self.get_parameter('lidar_offset_x').value), 0.0])
        self.max_step = float(self.get_parameter('max_translation_per_scan').value)

        self.x = self.y = self.yaw = 0.0
        self.imu_yaw0 = None
        self.last_imu_rel = None
        self.prev_scan_pts = None
        self.prev_scan_normals = None
        self.prev_scan_stamp = None
        self.prev_scan_yaw = None
        self.vx = self.vy = self.wz = 0.0
        self.rejects = 0
        self.last_err = float('nan')
        self.last_inliers = 0

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(1.0, float(self.get_parameter('publish_rate').value)),
                                       self.publish_timer)
        self.get_logger().info('Started point-to-line LiDAR translation + IMU yaw odometry')

    def imu_cb(self, msg):
        yaw_raw = quat_yaw(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw_raw
            self.last_imu_rel = 0.0
            return
        yaw_rel = wrap(yaw_raw - self.imu_yaw0)
        d_yaw = wrap(yaw_rel - self.last_imu_rel)
        self.yaw = wrap(self.yaw + d_yaw)
        self.last_imu_rel = yaw_rel
        self.wz = float(msg.angular_velocity.z)

    def scan_cb(self, msg):
        pts, normals = make_scan(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(pts) < 60:
            return
        if self.prev_scan_pts is None:
            self.prev_scan_pts, self.prev_scan_normals = pts, normals
            self.prev_scan_stamp, self.prev_scan_yaw = stamp, self.yaw
            return
        dt = stamp - self.prev_scan_stamp
        if dt <= 0.0 or dt > 0.25:
            self.prev_scan_pts, self.prev_scan_normals = pts, normals
            self.prev_scan_stamp, self.prev_scan_yaw = stamp, self.yaw
            self.vx = self.vy = 0.0
            return

        dyaw = wrap(self.yaw - self.prev_scan_yaw)
        result = point_line_translation(pts, self.prev_scan_pts,
                                       self.prev_scan_normals, dyaw)
        if result is not None:
            t_sensor, err, count = result
            # Convert LiDAR origin translation to trunk origin translation.
            t_trunk_prev = t_sensor - (R2(dyaw) @ self.lidar_lever - self.lidar_lever)
            if np.linalg.norm(t_trunk_prev) <= self.max_step:
                # Map increment from previous robot frame into odom/world.
                d_world = R2(self.yaw - dyaw) @ t_trunk_prev
                self.x += float(d_world[0])
                self.y += float(d_world[1])
                self.vx = float(t_trunk_prev[0] / dt)
                self.vy = float(t_trunk_prev[1] / dt)
                self.last_err, self.last_inliers = err, count
                self.rejects = 0
            else:
                result = None

        if result is None:
            self.rejects += 1
            self.vx = self.vy = 0.0
            self.get_logger().warn('LiDAR translation rejected (poor match/large step); holding XY',
                                  throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f'Pose x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}deg '
                f'pt-line err={self.last_err:.3f} inliers={self.last_inliers} rejects={self.rejects}',
                throttle_duration_sec=1.0
            )

        self.prev_scan_pts, self.prev_scan_normals = pts, normals
        self.prev_scan_stamp, self.prev_scan_yaw = stamp, self.yaw

    def publish_timer(self):
        self.publish(self.get_clock().now().to_msg())

    def publish(self, stamp):
        qz, qw = math.sin(self.yaw * 0.5), math.cos(self.yaw * 0.5)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'trunk'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz
        msg.pose.covariance[0] = 0.15
        msg.pose.covariance[7] = 0.15
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.25
        msg.twist.covariance[7] = 0.25
        msg.twist.covariance[35] = 0.05
        self.pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'trunk'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
