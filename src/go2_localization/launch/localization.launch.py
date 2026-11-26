import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_pub',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0.289', '0.0', '-0.046', '0.0', '2.878', '0.0', 'base_link', 'utlidar_lidar'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub',
            arguments=['0.289', '0.0', '-0.046', '0.0', '0.13', '-2.95', 'base_link', 'utlidar_imu'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='go2_localization',
            executable='imu_calibrator',
            name='imu_calibrator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='go2_localization',
            executable='cloud_filter',
            name='cloud_filter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        Node(
            package='go2_localization',
            executable='odom_tf_node',
            name='odom_tf_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.08,
                'frame_id': 'map',
                'sensor_model.max_range': 12.0,
                'sensor_model.hit': 0.72,
                'sensor_model.miss': 0.42,
                'filter_ground': False,
                'occupancy_min_z': -0.6,
                'occupancy_max_z': 1.5,
                'latch': True,
            }],
            remappings=[
                ('cloud_in', '/utlidar/cloud_clean')
            ]
        ),
        Node(
            package='go2_localization',
            executable='benchmark_node',
            name='benchmark_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
