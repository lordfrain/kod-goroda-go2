# Unitree Go2: 3D-картографирование и EKF-локализация (ROS 2 Humble)

Программное решение для построения воксельной 3D-карты помещения и устойчивой оценки положения робота Unitree Go2 в реальном времени по данным полусферического лидара Unitree L1 и встроенного IMU.

## Структура репозитория
- `ros2_ws/src/go2_localization/` — основной пакет ROS 2:
  - `imu_calibrator_node.py` — калибровка смещения нуля гироскопа (gyro bias) и инъекция матриц ковариаций в `/imu/calibrated` и `/odom/calibrated`.
  - `cloud_filter_node.py` — геометрическая фильтрация и 3D-воксельное удаление шумовых выбросов (`/utlidar/cloud_clean`).
  - `odom_tf_node.py` — публикация и фильтрация траекторий `/base_trajectory` и `/ekf_trajectory`.
  - `benchmark_node.py` — профилировщик задержек (мс) и частоты работы пайплайна (Гц) в реальном времени.
  - `config/ekf.yaml` — конфигурация расширенного фильтра Калмана (`robot_localization`).
  - `launch/localization.launch.py` — единый launch-файл всего пайплайна.
- `room_map_clean.bt` — построенная воксельная 3D-карта помещения (OctoMap, разрешение 0.08 м).
- `go2_ekf.rviz` — настроенная конфигурация визуализации RViz2.

## Быстрый запуск
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch go2_localization localization.launch.py
