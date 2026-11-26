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

# Unitree Go2: 3D-картографирование и EKF-локализация (ROS 2 Humble)

Программное решение для построения воксельной 3D-карты помещения и устойчивой оценки положения робота Unitree Go2 в реальном времени по данным полусферического лидара Unitree L1 и встроенного IMU.

## Структура репозитория
- `src/go2_localization/` — основной пакет ROS 2:
  - `imu_calibrator_node.py` — калибровка смещения нуля гироскопа (gyro bias) и инъекция матриц ковариаций в `/imu/calibrated` и `/odom/calibrated`.
  - `cloud_filter_node.py` — геометрическая фильтрация и 3D-воксельное удаление шумовых выбросов (`/utlidar/cloud_clean`).
  - `odom_tf_node.py` — публикация и фильтрация траекторий `/base_trajectory` и `/ekf_trajectory`.
  - `benchmark_node.py` — профилировщик задержек (мс) и частоты работы пайплайна (Гц) в реальном времени.
  - `config/ekf.yaml` — конфигурация расширенного фильтра Калмана (`robot_localization`).
  - `launch/localization.launch.py` — единый launch-файл всего пайплайна.
- `room_map_clean.bt` — построенная воксельная 3D-карта помещения (OctoMap, разрешение 0.08 м).
- `go2_ekf.rviz` — настроенная конфигурация визуализации RViz2.

## Быстрый запуск

# Unitree Go2: 3D-картографирование и EKF-локализация (ROS 2 Humble)

Программное решение команды для построения воксельной 3D-карты помещения и устойчивой оценки положения робота **Unitree Go2** в реальном времени по данным полусферического лидара Unitree L1 (`utlidar`) и встроенного IMU.

## Структура репозитория
- `src/go2_localization/` — основной пакет ROS 2 Humble:
  - `imu_calibrator_node.py` — калибровка смещения нуля гироскопа (gyro bias) и инъекция матриц ковариаций в `/imu/calibrated` и `/odom/calibrated`.
  - `cloud_filter_node.py` — геометрическая фильтрация бликов/потолка и 3D-воксельное удаление шумовых выбросов (`/utlidar/cloud_clean`).
  - `odom_tf_node.py` — публикация и фильтрация траекторий `/base_trajectory` (исходная одометрия) и `/ekf_trajectory` (EKF-фильтрация).
  - `benchmark_node.py` — профилировщик задержек обработки (мс) и частоты обновления позы (Гц) в реальном времени.
  - `config/ekf.yaml` — конфигурация расширенного фильтра Калмана (`robot_localization`).
  - `launch/localization.launch.py` — единый launch-файл всего пайплайна локализации и 3D-маппинга.
- `room_map_clean.bt` — построенная воксельная 3D-карта помещения (OctoMap, разрешение 0.08 м).
- `go2_ekf.rviz` — конфигурация сцены RViz2 для отображения 3D-карты и траекторий.
- `mapping_baseline_3d.py` / `localize.py` / `solution.rviz` — вспомогательные скрипты и базовая конфигурация.

## Требования
- Ubuntu 22.04 + **ROS 2 Humble**
- Установленные пакеты:
  ```bash
  sudo apt update && sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-octomap-server \
    ros-humble-octomap-rviz-plugins

Быстрый запуск

    Клонирование и сборка рабочей области ROS 2:
    Bash

    git clone [https://github.com/lordfrain/kod-goroda-go2.git](https://github.com/lordfrain/kod-goroda-go2.git)
    cd kod-goroda-go2
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

    Запуск пайплайна локализации, 3D-маппинга и замера времени (Терминал 1):
    Bash

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch go2_localization localization.launch.py

    (В консоли каждые 3 секунды выводятся замеры времени работы каждого этапа пайплайна в мс и итоговая частота позы в Гц).

    Запуск визуализации в RViz2 (Терминал 2):
    Bash

    source /opt/ros/humble/setup.bash
    rviz2 -d go2_ekf.rviz --ros-args -p use_sim_time:=true

    Воспроизведение датасета (Терминал 3):
    Bash

    source /opt/ros/humble/setup.bash
    ros2 bag play <ПУТЬ_К_BAG_ФАЙЛУ> --clock 100 --rate 1.0 \
      --topics /utlidar/robot_odom /utlidar/imu /utlidar/cloud /utlidar/robot_pose
    source /opt/ros/humble/setup.bash
    ros2 bag play <ПУТЬ_К_BAG_ФАЙЛУ> --clock 100 --rate 1.0 --topics /utlidar/robot_odom /utlidar/imu /utlidar/cloud /utlidar/robot_pose
