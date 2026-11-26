from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudFilterNode(Node):
    def __init__(self):
        super().__init__('cloud_filter_node')
        self.pub = self.create_publisher(PointCloud2, '/utlidar/cloud_clean', 10)
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cloud_cb, qos_profile_sensor_data
        )
        # Оптимальная ячейка 0.22 м и минимум 2 точки: отсекает одиночные блики, но сохраняет пол и дальние стены
        self.grid_res = 0.22
        self.min_neighbors = 2

    def cloud_cb(self, msg: PointCloud2):
        raw_pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
        if not raw_pts:
            return

        valid_pts = []
        voxel_counts = Counter()
        inv_res = 1.0 / self.grid_res

        for x, y, z, intensity in raw_pts:
            dist_sq = x * x + y * y + z * z
            # Отсекаем только блики самого корпуса (< 0.50 м) и дальний шум (> 12.0 м)
            if dist_sq < 0.25 or dist_sq > 144.0:
                continue
            # Сохраняем пол и стены до 1.5 м (отсекаем только потолочный мусор)
            if z < -0.8 or z > 1.5:
                continue
            key = (int(x * inv_res), int(y * inv_res), int(z * inv_res))
            voxel_counts[key] += 1
            valid_pts.append((x, y, z, intensity, key))

        clean_pts = [
            (x, y, z, intensity)
            for (x, y, z, intensity, key) in valid_pts
            if voxel_counts[key] >= self.min_neighbors
        ]

        if not clean_pts:
            return

        out_msg = point_cloud2.create_cloud(msg.header, msg.fields[:4], clean_pts)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
