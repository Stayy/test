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

## 自定义目标：10cm 间距柱子特征

当前主要目标是识别一组人工特征：若干根小柱子基本共线，柱子中心之间的名义间距约为 `10cm`。实际在 RViz 中，一根柱子可能表现为：

- 1 个红点
- 多个红点组成的小点堆
- 端部偶尔出现上下重叠点或重复回波
- 缩小雷达检测范围后，只稳定看到 4 根左右的柱子

因此节点对截图中的目标采用“短直线地标”识别方式，而不是要求每根柱子都必须是完美单点。

### 识别流程

1. 从 `/scan` 中提取有效点。
2. 将同一根柱子产生的多个点聚成一个锚点。
3. 用锚点中心拟合短直线。
4. 检查锚点中心之间的间距是否接近 `0.10m`。
5. 检查目标周围是否为空，避免把墙面的一小段误识别成目标。
6. 连续多帧确认并平滑输出，减少 RViz 中黄线闪烁。

### 推荐默认参数

```yaml
custom_pattern_enabled: true
custom_pattern_frame: scan       # 先不考虑底盘时，直接在 /scan 的 frame 下识别
visualization_frame: scan        # Marker 直接发布在 /scan 的 header.frame_id 下

# 短直线目标整体范围
line_min_points: 4
line_min_length: 0.25
line_max_length: 0.75
line_max_width: 0.08
line_max_range: 6.0

# 单根柱子的点堆 -> 锚点
line_anchor_cluster_jump_threshold: 0.06
line_anchor_max_diameter: 0.09

# 锚点数量和间距模板
line_min_anchor_count: 4
line_min_anchor_spacing: 0.10
line_anchor_spacing_tolerance: 0.02
line_group_max_anchor_gap: 0.18

# 直线假设和离群点容忍
line_hypothesis_lateral_tolerance: 0.05
line_hypothesis_endpoint_margin: 0.06

# 目标周围为空，用于过滤墙面
line_isolation_enabled: true
line_isolation_lateral_tolerance: 0.08
line_isolation_extension: 0.20

# 跨帧稳定
line_tracking_enabled: true
line_tracking_confirmations_required: 2
line_tracking_hold_frames: 3
line_tracking_max_match_distance: 0.18
line_tracking_max_match_yaw: 0.35
line_tracking_max_match_length_delta: 0.20
line_tracking_smoothing_alpha: 0.55
```

### 关键参数说明

| 参数 | 作用 |
| --- | --- |
| `line_anchor_cluster_jump_threshold` | 点堆内部相邻点的合并阈值。 |
| `line_anchor_max_diameter` | 一根柱子的点堆允许的最大直径。 |
| `line_min_anchor_count` | 至少需要几个柱子锚点才能输出目标，默认允许 4 个可见锚点。 |
| `line_min_anchor_spacing` | 柱子中心名义最小间距，目标为 `0.10m`。 |
| `line_anchor_spacing_tolerance` | 间距测量容差；设为 `0.0` 表示严格小于 10cm 一律拒绝。 |
| `line_isolation_extension` | 沿目标两端继续检查是否还有点，用于过滤墙。 |
| `line_tracking_confirmations_required` | 连续命中多少帧后才发布。 |
| `line_tracking_hold_frames` | 短暂漏检时保持上一结果的帧数。 |
| `line_tracking_smoothing_alpha` | 输出平滑系数，越小越稳，越大越跟手。 |

### 常见调参

- 一根柱子显示成多个点但没有合并：增大 `line_anchor_cluster_jump_threshold` 或 `line_anchor_max_diameter`。
- 大块墙面/物体被合成柱子点堆：减小 `line_anchor_max_diameter`。
- 只看到 4 根柱子时不输出：确认 `line_min_anchor_count: 4`、`line_min_points: 4`。
- 真实 10cm 目标因为误差被过滤：增大 `line_anchor_spacing_tolerance`，例如 `0.03`。
- 必须严格小于 10cm 一律拒绝：设置 `line_anchor_spacing_tolerance: 0.0`。
- 分离小簇没有合成短线：增大 `line_group_max_anchor_gap`。
- 端部出现上下重叠点：保持 `line_min_anchor_spacing: 0.10`，算法会尝试剔除一个重叠点后匹配主体。
- 墙面被误识别：保持 `line_isolation_enabled: true`，增大 `line_isolation_extension` 或 `line_isolation_lateral_tolerance`。
- 黄线显示不稳定/闪烁：增大 `line_tracking_confirmations_required` 或 `line_tracking_hold_frames`，减小 `line_tracking_smoothing_alpha`。
- 黄线响应太慢：减小 `line_tracking_confirmations_required` 或 `line_tracking_hold_frames`，增大 `line_tracking_smoothing_alpha`。

先不考虑底盘时，RViz 的 `Fixed Frame` 建议直接设置成 `/scan` 消息里的 `header.frame_id`，例如 `laser_frame`。这样不需要 `base_link` TF 也能看到自定义特征 Marker。

## 兼容的五柱栅栏输出

除了短直线地标，节点仍保留“五柱栅栏”检测输出。它使用候选柱子组合搜索，输出到 `/scan_feature_matcher/fence_poses`，主要参数如下：

```yaml
fence_pole_count: 5
fence_spacing: 0.10
fence_spacing_tolerance: 0.03
fence_collinearity_tolerance: 0.025
fence_max_pattern_error: 0.035
fence_max_candidate_poles: 30
fence_max_detections: 3
```

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
