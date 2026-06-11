# Nav2 RTAB-Map Navigation Parameters

This repository contains a Nav2 parameter file for a robot that uses RTAB-Map
for localization instead of AMCL. Nav2 is responsible for map loading, global
planning, path smoothing, local control, recovery behaviors, and lifecycle
management.

Use:

```bash
ros2 launch <your_nav2_launch_package> <your_nav2_launch_file>.py \
  params_file:=/workspace/config/nav2_rtabmap_params.yaml \
  map:=/path/to/map.yaml
```

## Goal-rotation oscillation fix

The tuned file is:

- `config/nav2_rtabmap_params.yaml`

It addresses the case where the robot reaches the target XY position, starts
rotating toward the goal yaw, then reverses direction before ever crossing the
target yaw and oscillates repeatedly.

The relevant changes are in `controller_server`:

- `FollowPath.stateful: true` keeps Regulated Pure Pursuit in the final heading
  adjustment state after the XY tolerance is reached, instead of letting the
  lookahead carrot jump around the robot and flip the angular command.
- `rotate_to_heading_angular_vel`, `max_angular_vel`, and `max_angular_accel`
  are limited to gentler values that better match a 0.35 m/s-class chassis.
- `goal_checker.yaw_goal_tolerance` is set to `0.52` rad, which is about 30
  degrees. The previous value `3.14` rad is about 180 degrees and effectively
  disables final-yaw accuracy.

If the robot only needs to reach the target position and the final yaw should be
ignored, keep a large yaw tolerance or switch to a position-only goal checker,
and consider disabling `FollowPath.use_rotate_to_heading`.
