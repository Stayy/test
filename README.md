# scan_feature_matcher

这是一个 ROS 2 Python 示例包，用于单线激光雷达在底盘运动过程中的相邻帧特征匹配。

节点订阅 `sensor_msgs/msg/LaserScan` 类型的 `/scan`，将雷达点转换到底盘坐标系，提取几何角点和距离断点特征，在相邻帧之间做互最近邻匹配，并通过 RANSAC + 2D 刚体最小二乘估计底盘的相对运动。

## 输出

- `scan_feature_matcher/odom` (`nav_msgs/msg/Odometry`): 基于相邻帧匹配累积得到的里程计。
- `scan_feature_matcher/markers` (`visualization_msgs/msg/MarkerArray`): RViz 调试标记，包含当前帧特征点和匹配线。
- `odom -> base_link` TF: 默认开启，可通过 `publish_tf` 关闭。

## 构建

```bash
colcon build --packages-select scan_feature_matcher
source install/setup.bash
```

## 运行

```bash
ros2 launch scan_feature_matcher feature_matcher.launch.py
```

如果你的雷达话题不是 `/scan`，可以修改 `config/feature_matcher.yaml` 或启动时传入自己的参数文件：

```bash
ros2 launch scan_feature_matcher feature_matcher.launch.py \
  config_file:=/absolute/path/to/feature_matcher.yaml
```

## 底盘安装参数

`config/feature_matcher.yaml` 中的以下参数描述雷达坐标系相对底盘 `base_link` 的固定安装位姿：

```yaml
laser_x: 0.0      # 雷达原点相对 base_link 的 x，单位 m
laser_y: 0.0      # 雷达原点相对 base_link 的 y，单位 m
laser_yaw: 0.0    # 雷达相对 base_link 的偏航角，单位 rad
```

如果雷达正向与底盘正向一致且安装在底盘中心，可以保持默认值。否则请实测安装偏移后填写，匹配估计会在底盘坐标系下进行。

## 调参建议

- 特征太少：降低 `corner_threshold` 或 `range_jump_threshold`，也可以增大 `max_features`。
- 误匹配较多：减小 `max_correspondence_distance`、`max_bearing_shift` 或 `ransac_inlier_threshold`。
- 底盘速度较快：适当增大 `max_correspondence_distance` 和 `max_motion_translation`。
- 环境几何单一：单线雷达只提供 2D 平面信息，长走廊、开阔空地等场景容易退化，需要轮速计/IMU 或地图约束辅助。

## 可视化

在 RViz 中添加：

1. `LaserScan` 显示 `/scan`
2. `MarkerArray` 显示 `/scan_feature_matcher/markers`
3. `Odometry` 显示 `/scan_feature_matcher/odom`

固定坐标系可以设为 `odom`。如果系统中已有其他里程计发布 `odom -> base_link`，请将本节点的 `publish_tf` 设为 `false`，避免 TF 冲突。
