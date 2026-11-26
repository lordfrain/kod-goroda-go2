import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shelltime',
    maintainer_email='user@example.com',
    description='3D Mapping and EKF Localization pipeline for Unitree Go2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_node = go2_localization.odom_tf_node:main',
            'drift_detector = go2_localization.drift_detector_node:main',
            'imu_calibrator = go2_localization.imu_calibrator_node:main',
            'cloud_filter = go2_localization.cloud_filter_node:main',
            'benchmark_node = go2_localization.benchmark_node:main',
        ],
    },
)
