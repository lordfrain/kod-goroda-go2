import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def get_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DriftDetectorNode(Node):
    def __init__(self):
        super().__init__('drift_detector_node')
        self.global_path_pub = self.create_publisher(Path, '/global_trajectory', 10)
        self.global_path = Path()
        self.global_path.header.frame_id = 'odom'

        self.latest_odom = None
        self.last_pose_time = None
        self.start_time = None
        self.max_drift = 0.0
        self.max_yaw_diff_deg = 0.0

        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, 50)
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_cb, 20)
        self.get_logger().info('Детектор расхождений запущен. Ожидание данных...')

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now_sec

        # 1. Проверка пропуска кадров глобальной локализации (> 0.12 с при норме ~0.053 с)
        if self.last_pose_time is not None:
            dt_pose = now_sec - self.last_pose_time
            if dt_pose > 0.12:
                rel_t = now_sec - self.start_time
                self.get_logger().warn(
                    f'[ПРОГНОЗ ПО ТАЙМАУТУ] t_отн={rel_t:.2f} с (stamp={now_sec:.2f}) | '
                    f'пауза global_pose={dt_pose:.3f} с',
                    throttle_duration_sec=0.5
                )

    def pose_cb(self, msg: PoseStamped):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now_sec
        rel_t = now_sec - self.start_time
        self.last_pose_time = now_sec

        self.global_path.header.stamp = msg.header.stamp
        self.global_path.poses.append(msg)
        self.global_path_pub.publish(self.global_path)

        if self.latest_odom is None:
            return

        # 2. Сравнение текущих координат и угла Yaw
        gx, gy = msg.pose.position.x, msg.pose.position.y
        ox = self.latest_odom.pose.pose.position.x
        oy = self.latest_odom.pose.pose.position.y
        drift_dist = math.hypot(gx - ox, gy - oy)

        g_yaw = get_yaw(msg.pose.orientation)
        o_yaw = get_yaw(self.latest_odom.pose.pose.orientation)
        yaw_diff_deg = abs(math.degrees(normalize_angle(g_yaw - o_yaw)))

        if drift_dist > self.max_drift:
            self.max_drift = drift_dist
        if yaw_diff_deg > self.max_yaw_diff_deg:
            self.max_yaw_diff_deg = yaw_diff_deg

        # Выводим предупреждение, если расхождение > 15 см или > 5 градусов
        if drift_dist > 0.15 or yaw_diff_deg > 5.0:
            self.get_logger().error(
                f'[ДРЕЙФ / РАСХОЖДЕНИЕ] t_отн={rel_t:.2f} с (stamp={now_sec:.2f}) | '
                f'смещение={drift_dist:.3f} м (max={self.max_drift:.3f} м) | '
                f'угол={yaw_diff_deg:.2f}° (max={self.max_yaw_diff_deg:.2f}°)',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info(
                f'[OK] t_отн={rel_t:.2f} с | смещение={drift_dist:.3f} м | угол={yaw_diff_deg:.2f}°',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = DriftDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
