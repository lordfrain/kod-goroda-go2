import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


class PipelineBenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_node')
        self.raw_imu_times = {}
        self.raw_odom_times = {}
        self.raw_cloud_times = {}

        self.imu_latencies = []
        self.ekf_latencies = []
        self.cloud_latencies = []
        self.ekf_timestamps = []

        self.create_subscription(Imu, '/utlidar/imu', self.raw_imu_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/calibrated', self.cal_imu_cb, 50)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.raw_odom_cb, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_odom_cb, 50)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.raw_cloud_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_clean', self.clean_cloud_cb, 10)

        self.create_timer(3.0, self.print_report)

    def _stamp_key(self, stamp):
        return (stamp.sec, stamp.nanosec)

    def raw_imu_cb(self, msg: Imu):
        self.raw_imu_times[self._stamp_key(msg.header.stamp)] = time.perf_counter()

    def cal_imu_cb(self, msg: Imu):
        t0 = self.raw_imu_times.pop(self._stamp_key(msg.header.stamp), None)
        if t0 is not None:
            self.imu_latencies.append((time.perf_counter() - t0) * 1000.0)

    def raw_cloud_cb(self, msg: PointCloud2):
        self.raw_cloud_times[self._stamp_key(msg.header.stamp)] = time.perf_counter()

    def clean_cloud_cb(self, msg: PointCloud2):
        t0 = self.raw_cloud_times.pop(self._stamp_key(msg.header.stamp), None)
        if t0 is not None:
            self.cloud_latencies.append((time.perf_counter() - t0) * 1000.0)

    def raw_odom_cb(self, msg: Odometry):
        self.raw_odom_times[self._stamp_key(msg.header.stamp)] = time.perf_counter()

    def ekf_odom_cb(self, msg: Odometry):
        now = time.perf_counter()
        self.ekf_timestamps.append(now)
        t0 = self.raw_odom_times.pop(self._stamp_key(msg.header.stamp), None)
        if t0 is not None:
            self.ekf_latencies.append((now - t0) * 1000.0)

    def print_report(self):
        if len(self.ekf_timestamps) < 10:
            return
        dt = self.ekf_timestamps[-1] - self.ekf_timestamps[0]
        hz = (len(self.ekf_timestamps) - 1) / dt if dt > 0 else 0.0

        imu_avg = sum(self.imu_latencies[-200:]) / max(1, len(self.imu_latencies[-200:]))
        ekf_avg = sum(self.ekf_latencies[-200:]) / max(1, len(self.ekf_latencies[-200:]))
        ekf_max = max(self.ekf_latencies[-200:]) if self.ekf_latencies else 0.0
        cld_avg = sum(self.cloud_latencies[-50:]) / max(1, len(self.cloud_latencies[-50:]))

        self.get_logger().info(
            f'\n================ ЗАМЕРЫ ПАЙПЛАЙНА ================\n'
            f'  1. Препроцессинг IMU (bias + cov):   {imu_avg:.3f} мс\n'
            f'  2. Фильтрация лидара (Outlier Rem):  {cld_avg:.3f} мс\n'
            f'  3. Фильтрация EKF (sensor fusion):   {ekf_avg:.3f} мс (макс: {ekf_max:.3f} мс)\n'
            f'  4. Итоговая частота позы (TF/EKF):   {hz:.1f} Гц\n'
            f'=================================================='
        )


def main(args=None):
    rclpy.init(args=args)
    node = PipelineBenchmarkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
