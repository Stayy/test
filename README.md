# scan_feature_matcher

这是一个 ROS 2 Python 示例包，用于单线激光雷达在底盘运动过程中的相邻帧特征匹配。

节点订阅 `sensor_msgs/msg/LaserScan` 类型的 `/scan`，将雷达点转换到底盘坐标系，提取几何角点和距离断点特征，在相邻帧之间做互最近邻匹配，并通过 RANSAC + 2D 刚体最小二乘估计底盘的相对运动。

## 输出

- `scan_feature_matcher/odom` (`nav_msgs/msg/Odometry`): 基于相邻帧匹配累积得到的里程计。
- `scan_feature_matcher/markers` (`visualization_msgs/msg/MarkerArray`): RViz 调试标记，包含当前帧特征点和匹配线。
- `scan_feature_matcher/fence_poses` (`geometry_msgs/msg/PoseArray`): 识别到的自定义五柱栅栏地标位姿，坐标系由 `custom_pattern_frame` 决定。
- `scan_feature_matcher/line_feature_poses` (`geometry_msgs/msg/PoseArray`): 识别到的短直线点簇特征，适合先不考虑底盘时查看截图中黄色框里的目标。
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

## 自定义特征：五柱栅栏和短直线点簇

如果需要识别“由间隔 10cm 的五根柱子组成的栅栏”，节点会按以下步骤标识：

1. 将 `/scan` 中连续且距离接近的小点簇聚类成候选柱子。
2. 在候选柱子中寻找 5 个共线目标。
3. 检查相邻柱子的投影间距是否接近 `0.10m`。
4. 如果间距误差、共线误差都在阈值内，就输出一个 `FenceDetection`。

默认参数如下：

```yaml
custom_pattern_enabled: true
custom_pattern_frame: scan       # 先不考虑底盘时，直接在 /scan 的 frame 下识别

# 单根柱子的 LaserScan 点簇约束
pole_cluster_jump_threshold: 0.06  # 相邻激光点距离超过该值则切分点簇
pole_min_points: 1                 # 远距离时一根柱子可能只有一个激光点
pole_max_points: 12
pole_min_width: 0.0
pole_max_width: 0.08               # 单根柱子在扫描平面中的最大可见宽度
pole_max_range: 6.0

# 五柱栅栏模式约束
fence_pole_count: 5
fence_spacing: 0.10
fence_spacing_tolerance: 0.03
fence_collinearity_tolerance: 0.025
fence_max_pattern_error: 0.035
fence_max_candidate_poles: 30
fence_max_detections: 3
```

截图中黄色框里的目标也可以看成“短直线点簇”。如果五根柱子在 LaserScan 中已经连成一小段，短直线检测比单独分割每根柱子更稳：

```yaml
visualization_frame: scan        # Marker 直接发布在 /scan 的 header.frame_id 下
line_cluster_jump_threshold: 0.16
line_min_points: 5
line_min_length: 0.25
line_max_length: 0.75
line_max_width: 0.08
line_max_range: 6.0
line_anchor_cluster_jump_threshold: 0.06
line_min_anchor_count: 3
line_min_anchor_spacing: 0.10
line_group_max_anchor_gap: 0.18
line_hypothesis_lateral_tolerance: 0.05
line_hypothesis_endpoint_margin: 0.06
line_isolation_enabled: true
line_isolation_lateral_tolerance: 0.08
line_isolation_extension: 0.20
line_max_detections: 5
```

先不考虑底盘时，RViz 的 `Fixed Frame` 建议直接设置成 `/scan` 消息里的 `header.frame_id`，例如 `laser_frame`。这样不需要 `base_link` TF 也能看到自定义特征 Marker。

实际使用时建议先在 RViz 中看候选柱子是否稳定：

- 候选柱子太少：增大 `pole_cluster_jump_threshold` 或 `pole_max_width`，降低 `pole_min_points`。
- 误把其他物体识别成柱子：减小 `pole_max_width`、`pole_max_range`，或增大 `pole_min_points`。
- 五根柱子已识别但栅栏不输出：增大 `fence_spacing_tolerance` 或 `fence_collinearity_tolerance`。
- 误识别栅栏：减小 `fence_spacing_tolerance`、`fence_collinearity_tolerance` 或 `fence_max_pattern_error`。
- 连成一段的黄色短线识别不到：增大 `line_cluster_jump_threshold`、放宽 `line_max_width`，或降低 `line_min_points`。
- 蓝色框这类分离小簇没有合成短线：增大 `line_group_max_anchor_gap`，或降低 `line_min_anchor_count`。
- 小于 10cm 的密集点被误识别：保持或增大 `line_min_anchor_spacing`。
- 左右两类目标旁边有离群点导致漏检：增大 `line_hypothesis_lateral_tolerance` 或 `line_hypothesis_endpoint_margin`。
- 黄色框墙面被误识别为目标：保持 `line_isolation_enabled: true`，增大 `line_isolation_extension` 或 `line_isolation_lateral_tolerance`。
- 把底盘弧线也误识别成黄色特征：减小 `line_max_width` 或收紧 `line_min_length`/`line_max_length`。

## 可视化

在 RViz 中添加：

1. `LaserScan` 显示 `/scan`
2. `MarkerArray` 显示 `/scan_feature_matcher/markers`
3. `PoseArray` 显示 `/scan_feature_matcher/line_feature_poses`
4. `PoseArray` 显示 `/scan_feature_matcher/fence_poses`
5. `Odometry` 显示 `/scan_feature_matcher/odom`

`/scan_feature_matcher/markers` 中的颜色含义：

- 绿色点：普通角点特征
- 橙色点：普通距离断点特征
- 蓝色线：相邻帧普通特征匹配
- 紫色点：候选柱子
- 红色线/红色点：识别到的五柱栅栏和栅栏中心
- 黄色线/黄色点：截图中黄色框这类短直线自定义特征和中心

固定坐标系可以设为 `odom`。如果系统中已有其他里程计发布 `odom -> base_link`，请将本节点的 `publish_tf` 设为 `false`，避免 TF 冲突。
