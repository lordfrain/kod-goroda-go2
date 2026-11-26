# Unitree Go2 — карта и локализация

В ROS 2 Humble с numpy, scipy и плагином rosbag2 MCAP. Используются две разные записи. Первый проход строит карту, второй сопоставляет сканы с фиксированной картой. Для обработки достаточно облаков в системе корпуса и кинематической одометрии.

```bash
source /opt/ros/humble/setup.bash
ros2 bag info "$MAPPING_BAG"
ros2 bag info "$LOCALIZATION_BAG"
```

Переменные указывают на папки с metadata.yaml. Проверьте фактические типы сообщений, frame_id у облака и child_frame_id у одометрии. Если оба равны base_link, используйте команды ниже. Глобальный cloud_deskewed не подавайте как локальное облако.

```bash
python3 mapping_baseline_3d.py "$MAPPING_BAG" --storage mcap \
  --odom /utlidar/robot_odom --cloud /utlidar/cloud_base \
  --base-frame base_link --cloud-frame base_link \
  --extrinsic 0 0 0 0 0 0 1 --voxel 0.18 --output mapping
```

В отдельных терминалах, каждый с подключённым Humble, запустите локализатор, RViz и запись результатов. В примере начальное преобразование map ← odom второго прохода равно единичному. Введите измеренные X Y Z и угол yaw в градусах, если старты различаются. Локализатор рассчитан на приблизительно известный старт.

```bash
python3 localize.py mapping/map.ply --initial 0 0 0 0 --metrics localization.csv
```

```bash
rviz2 -d solution.rviz
```

```bash
ros2 bag record -s mcap -o result.bag /map /solution/aligned_cloud /solution/pose
```

Дождитесь подписок и воспроизведите второй проход в масштабе времени 1:1:

```bash
ros2 bag play "$LOCALIZATION_BAG" --clock --rate 1.0 \
  --topics /utlidar/cloud_base /utlidar/robot_odom
```

В RViz Fixed 3D map отображает /map (PointCloud2, готовая 3D-карта), а Current scan aligned to map отображает /solution/aligned_cloud (обновляемый скан в координатах карты). Стрелка получает /solution/pose. Для слоя сканов используйте Decay Time 0, чтобы видеть только текущий скан.

Остановите запись через Ctrl+C. result.bag — папка rosbag2 с metadata.yaml и MCAP. Заархивируйте её целиком. Проверьте ros2 bag info result.bag, наличие трёх топиков и воспроизводимость. Запишите экран RViz с видом сверху и затем с наклонным ракурсом.

В localization.csv compute_ms измеряет обработку от начала сопоставления по времени до публикации, callback_to_publish_ms включает ожидание одометрии после приёма облака. Для слайдов посчитайте p50/p95, долю ICP, предсказаний odom и пропусков и сравните время с фактическим периодом лидара. Остаток ICP оценивает согласование с картой, абсолютная точность требует независимого эталона. При плохом перекрытии или слишком большой поправке используется прогноз кинематической одометрии.

## Ускоренный режим
Добавьте `--fast` к команде запуска и задайте другое имя CSV и выходного rosbag. Режим ограничивает вход облака 600 точками и ICP пятью итерациями. На двух прогонах второго конкурсного bag при 1× задержка p95 составила 85,9 мс в исходном режиме и 64,2 мс в ускоренном. Прогнозов одометрии было соответственно 12 и 30. Эти цифры зависят от нагрузки виртуальной машины.
