"""Localize PointCloud2 scans against a fixed map while rosbag replays."""
import argparse
from collections import deque
import csv
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header
from mapping_baseline_3d import pose_at, rotation, stamp, voxel, xyz, quat


def load_map(path):
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Invalid PLY header")
            if line.strip() == b"end_header":
                break
        points = np.fromfile(stream, dtype="<f4").reshape(-1, 3)
    if len(points) < 100:
        raise ValueError("Map has fewer than 100 points")
    return points


def align(points, tree, prior, radius):
    transform = prior.copy()
    for _ in range(8):
        moved = points @ transform[:3, :3].T + transform[:3, 3]
        distances, indices = tree.query(moved, distance_upper_bound=radius)
        matches = np.flatnonzero(np.isfinite(distances))
        if len(matches) < max(45, len(points) // 5):
            return None, len(matches) / len(points), float("nan")
        matches = matches[np.argsort(distances[matches])[:max(35, int(len(matches)*0.75))]]
        source, target = moved[matches], tree.data[indices[matches]]
        a, b = source.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((source-a).T @ (target-b))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0:
            vt[-1] *= -1
            rot = vt.T @ u.T
        shift = b - rot @ a
        transform[:3, :3] = rot @ transform[:3, :3]
        transform[:3, 3] = rot @ transform[:3, 3] + shift
        if np.linalg.norm(shift) < 0.005 and Rotation.from_matrix(rot).magnitude() < 0.005:
            break
    final = points @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = tree.query(final, distance_upper_bound=radius)
    valid = np.isfinite(distances)
    overlap = float(valid.mean())
    residual = float(np.median(distances[valid])) if valid.any() else float("nan")
    position_delta = np.linalg.norm(transform[:3, 3] - prior[:3, 3])
    angle_delta = Rotation.from_matrix(transform[:3, :3] @ prior[:3, :3].T).magnitude()
    if overlap < 0.25 or residual > radius * 0.65 or position_delta > 0.6 or angle_delta > 0.35:
        return None, overlap, residual
    return transform, overlap, residual


class Localizer(Node):
    def __init__(self, args):
        super().__init__("go2_scan_to_map")
        self.args = args
        points = load_map(args.map)
        self.tree = cKDTree(points)
        self.poses, self.pending = deque(maxlen=256), deque()
        tx, ty, tz, yaw = args.initial
        self.correction = np.eye(4)
        self.correction[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix()
        self.correction[:3, 3] = [tx, ty, tz]
        self.map_pub = self.create_publisher(PointCloud2, "/solution/map", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_pub.publish(create_cloud_xyz32(Header(frame_id="map"), points))
        self.pose_pub = self.create_publisher(PoseStamped, "/solution/pose", 20)
        self.create_subscription(Odometry, args.odom, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.cloud, self.on_cloud, qos_profile_sensor_data)
        self.log = args.metrics.open("w", newline="", encoding="utf8")
        self.csv = csv.writer(self.log)
        self.csv.writerow(["stamp_ns", "status", "compute_ms", "callback_to_publish_ms",
                           "overlap", "median_residual_m"])
        self.create_timer(1.0, self.log.flush)

    def on_odom(self, msg):
        t = stamp(msg)
        pose = np.asarray(xyz(msg.pose.pose.position) + quat(msg.pose.pose.orientation))
        if msg.child_frame_id != self.args.base_frame or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-9:
            return
        if self.poses and t <= self.poses[-1][0]:
            return
        self.poses.append((t, pose))
        while self.pending and stamp(self.pending[0][0]) <= t:
            self.process(*self.pending.popleft())

    def on_cloud(self, msg):
        start = perf_counter()
        if msg.header.frame_id != self.args.base_frame:
            self.get_logger().error("Expected cloud frame " + self.args.base_frame, throttle_duration_sec=3.0)
            return
        if self.poses and stamp(msg) <= self.poses[-1][0]:
            self.process(msg, start)
        else:
            if len(self.pending) >= 4:
                old, _ = self.pending.popleft()
                self.csv.writerow([stamp(old), "queue_drop", "", "", "", ""])
            self.pending.append((msg, start))

    def process(self, msg, received):
        start = perf_counter()
        t = stamp(msg)
        times, poses = zip(*self.poses)
        state = pose_at(np.asarray(times), np.asarray(poses), t, 150_000_000)
        if state is None:
            self.csv.writerow([t, "no_sync", "", "", "", ""])
            return
        odom = np.eye(4)
        odom[:3, :3], odom[:3, 3] = rotation(state[3:]), state[:3]
        raw = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        cloud = np.column_stack([raw[name] for name in ("x", "y", "z")])
        cloud = cloud[::max(1, len(cloud)//4500)]
        cloud = voxel(cloud[np.isfinite(cloud).all(axis=1)], 0.18)[:3500]
        prior = self.correction @ odom
        estimate, overlap, residual = align(cloud, self.tree, prior, self.args.radius) if len(cloud) >= 100 else (None, 0.0, float("nan"))
        if estimate is not None:
            self.correction = estimate @ np.linalg.inv(odom)
        result = prior if estimate is None else estimate
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "map", msg.header.stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, result[:3, 3])
        q = Rotation.from_matrix(result[:3, :3]).as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
        self.pose_pub.publish(pose)
        end = perf_counter()
        self.csv.writerow([t, "icp" if estimate is not None else "odom_prediction",
                           1000*(end-start), 1000*(end-received), overlap, residual])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path)
    parser.add_argument("--cloud", default="/utlidar/cloud_base")
    parser.add_argument("--odom", default="/utlidar/robot_odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--initial", type=float, nargs=4, required=True, metavar=("X", "Y", "Z", "YAW_DEG"))
    parser.add_argument("--radius", type=float, default=0.7)
    parser.add_argument("--metrics", type=Path, default=Path("localization.csv"))
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Localizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

