import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomTfNode(Node):
    def __init__(self):
        super().__init__('odom_tf_node')
        self.path_pub = self.create_publisher(Path, '/base_trajectory', 10)
        self.ekf_path_pub = self.create_publisher(Path, '/ekf_trajectory', 10)

        self.base_path = Path()
        self.base_path.header.frame_id = 'odom'
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'

        self.last_odom_t = None
        self.last_ekf_t = None
        self.latest_odom = None
        self.latest_ekf = None

        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 50)
        self.create_timer(0.25, self.publish_paths)

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_t is not None and t < self.last_odom_t - 1.0:
            self.base_path.poses.clear()
        self.last_odom_t = t
        self.latest_odom = msg

    def ekf_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_ekf_t is not None and t < self.last_ekf_t - 1.0:
            self.ekf_path.poses.clear()
        self.last_ekf_t = t

        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self.latest_ekf = msg

    def publish_paths(self):
        if self.latest_odom is not None:
            p = PoseStamped()
            p.header = self.latest_odom.header
            p.pose = self.latest_odom.pose.pose
            if not self.base_path.poses or math.hypot(
                p.pose.position.x - self.base_path.poses[-1].pose.position.x,
                p.pose.position.y - self.base_path.poses[-1].pose.position.y
            ) > 0.03:
                self.base_path.poses.append(p)
            self.base_path.header.stamp = self.latest_odom.header.stamp
            self.path_pub.publish(self.base_path)

        if self.latest_ekf is not None:
            p = PoseStamped()
            p.header = self.latest_ekf.header
            p.pose = self.latest_ekf.pose.pose
            if self.ekf_path.poses:
                prev = self.ekf_path.poses[-1].pose.position
                dist = math.hypot(p.pose.position.x - prev.x, p.pose.position.y - prev.y)
                if dist > 1.5:
                    self.ekf_path.poses.clear()
                elif dist < 0.03:
                    self.ekf_path.header.stamp = self.latest_ekf.header.stamp
                    self.ekf_path_pub.publish(self.ekf_path)
                    return
            self.ekf_path.poses.append(p)
            self.ekf_path.header.stamp = self.latest_ekf.header.stamp
            self.ekf_path_pub.publish(self.ekf_path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
