import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class SensorCalibratorNode(Node):
    def __init__(self):
        super().__init__('imu_calibrator_node')
        self.imu_pub = self.create_publisher(Imu, '/imu/calibrated', 50)
        self.odom_pub = self.create_publisher(Odometry, '/odom/calibrated', 50)

        self.create_subscription(Imu, '/utlidar/imu', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, 50)

        self.bias_wx = 0.007616
        self.bias_wy = -0.013660
        self.bias_wz = -0.003561
        self.calib_samples = []
        self.calibrated = False

    def odom_cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_link'
        out.pose.pose = msg.pose.pose
        # Заполняем ненулевую диагональную ковариацию позы (x, y, z, roll, pitch, yaw)
        out.pose.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.02, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.03, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.05, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.05, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]
        out.twist.twist = msg.twist.twist
        # Заполняем ковариацию скоростей (Vx, Vy, Vz, Wx, Wy, Wz)
        out.twist.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.05, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.05, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.05, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.08
        ]
        self.odom_pub.publish(out)

    def imu_cb(self, msg: Imu):
        if not self.calibrated:
            if abs(msg.angular_velocity.z) < 0.05 and abs(msg.angular_velocity.x) < 0.05:
                self.calib_samples.append((
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ))
            if len(self.calib_samples) >= 300:
                n = len(self.calib_samples)
                self.bias_wx = sum(s[0] for s in self.calib_samples) / n
                self.bias_wy = sum(s[1] for s in self.calib_samples) / n
                self.bias_wz = sum(s[2] for s in self.calib_samples) / n
                self.calibrated = True
                self.get_logger().info(
                    f'Сенсоры откалиброваны! Gyro bias: ({self.bias_wx:.5f}, {self.bias_wy:.5f}, {self.bias_wz:.5f})'
                )

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.05
        ]
        out.angular_velocity.x = msg.angular_velocity.x - self.bias_wx
        out.angular_velocity.y = msg.angular_velocity.y - self.bias_wy
        out.angular_velocity.z = msg.angular_velocity.z - self.bias_wz
        # Доверяем гироскопу IMU (низкая дисперсия 0.0005) для сглаживания резких поворотов
        out.angular_velocity_covariance = [
            0.0005, 0.0,    0.0,
            0.0,    0.0005, 0.0,
            0.0,    0.0,    0.0005
        ]
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorCalibratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
