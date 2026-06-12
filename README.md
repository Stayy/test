# Nav2 终点朝向摇摆调参说明

本仓库提供一份 Nav2 参数配置，适用于使用 RTAB-Map 定位、Regulated Pure
Pursuit 跟踪路径的低速底盘。

## 现象

执行单次 `NavigateToPose` 时，机器人到达目标 XY 位置后开始原地旋转，但还没转过
目标角度就提前反向，随后反复左右摇摆。若拆成两次导航，先运行到目标位置，再单独
原地旋转，则不会出现该问题。

## 可能原因

原配置中，控制器在接近目标点时仍保持较大的最小前视距离和较高的最低接近速度，
同时终点原地旋转角速度也偏高。到达目标附近后，这会让控制器仍然被路径跟踪和定位
延迟牵引，而不是稳定进入最终朝向对齐阶段。

最相关的参数位于 `controller_server.FollowPath`：

- `min_lookahead_dist`
- `lookahead_time`
- `min_approach_linear_velocity`
- `regulated_linear_scaling_min_speed`
- `rotate_to_heading_angular_vel`
- `max_angular_accel`
- `allow_reversing`

## 推荐配置

建议以 `nav2_params_rpp_stable.yaml` 作为起点。相比问题配置，关键变化如下：

- 将控制频率从 `10 Hz` 提高到 `20 Hz`。
- 将最小前视距离从 `0.50 m` 降到 `0.15 m`。
- 将接近目标时的最低线速度从 `0.05 m/s` 降到 `0.02 m/s`。
- 将曲率/代价调速后的最低速度从 `0.10 m/s` 降到 `0.03 m/s`。
- 将终点原地旋转角速度从 `0.80 rad/s` 降到 `0.35 rad/s`。
- 增加 `max_angular_accel: 0.80`，限制角加速度。
- 设置 `allow_reversing: false`，避免路径跟踪阶段倒车。
- 保持 `SimpleGoalChecker.stateful: true`。

如果最终朝向不重要，可以改用只检查位置的 goal checker，或把 yaw 容差放得更宽。
如果最终朝向重要，则需要让 `yaw_goal_tolerance` 与实际定位噪声匹配；当前推荐配置
使用 `0.35 rad`。
