# Nav2 terminal yaw oscillation tuning

This repository contains a Nav2 parameter profile for a low-speed chassis using
RTAB-Map for localization and Regulated Pure Pursuit for path following.

## Symptom

During a single `NavigateToPose` goal, the robot reaches the target XY position,
starts rotating in place, then reverses angular direction before passing the
target yaw. It repeats this left/right oscillation. Running the task as two
separate commands, first drive to the position and then rotate in place, does
not show the same issue.

## Likely cause

The original configuration asks the controller to keep a large minimum
lookahead distance and a relatively high minimum approach speed while also
using a fast terminal rotation command. Near the goal, this can keep the
controller coupled to path tracking and localization delay instead of settling
into a clean final yaw alignment.

The most relevant parameters are in `controller_server.FollowPath`:

- `min_lookahead_dist`
- `lookahead_time`
- `min_approach_linear_velocity`
- `regulated_linear_scaling_min_speed`
- `rotate_to_heading_angular_vel`
- `max_angular_accel`
- `allow_reversing`

## Recommended profile

Use `nav2_params_rpp_stable.yaml` as the starting point. The important changes
from the reported configuration are:

- Increase controller frequency from `10 Hz` to `20 Hz`.
- Reduce minimum lookahead from `0.50 m` to `0.15 m`.
- Reduce approach minimum linear speed from `0.05 m/s` to `0.02 m/s`.
- Reduce regulated minimum speed from `0.10 m/s` to `0.03 m/s`.
- Reduce terminal rotation speed from `0.80 rad/s` to `0.35 rad/s`.
- Add `max_angular_accel: 0.80`.
- Set `allow_reversing: false`.
- Keep `SimpleGoalChecker.stateful: true`.

If the final heading is not important, replace the yaw goal check with a
position-only goal checker or set a very loose yaw tolerance. If the final
heading is important, keep `yaw_goal_tolerance` realistic for the localization
noise; the included profile uses `0.35 rad`.
