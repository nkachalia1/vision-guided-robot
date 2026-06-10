# Obstacle Avoidance

This project now has a minimal lidar safety layer.

The vision controller publishes desired velocity:

```text
/cmd_vel_raw
```

The safety layer subscribes to `/cmd_vel_raw` and `/scan`, then publishes the final command:

```text
/cmd_vel
```

This means the robot can still see and approach the ball, but forward motion is slowed or blocked when the lidar sees an obstacle in front.

## Test With Occlusion

Launch a case where the wall partially blocks the direct path:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 ball_y:=1.5 \
  target_model:=dark_red_ball \
  detector_real_diameter_m:=0.20 \
  detector_min_area_px:=40.0 \
  detector_min_circularity:=0.20 \
  spawn_occluder:=true \
  occluder_x:=1.6 occluder_y:=1.3 occluder_z:=0.3
```

Watch the safety state:

```bash
ros2 topic echo /safety/state
```

Expected states:

- `CLEAR`: no obstacle close in front
- `SLOW`: obstacle ahead, forward speed reduced
- `BLOCKED`: obstacle too close, forward motion stopped
- `AVOID`: latched maneuver after a block; keep one turn direction and creep forward if the center path allows
- `STALE_SCAN`: no recent lidar data, robot stopped

## Tuned Defaults

These values are the current baseline after simulation tuning:

```text
safety_filter.avoid_hold_time_s: 1.2
safety_filter.avoid_forward_speed_mps: 0.50
safety_filter.avoid_turn_speed_radps: 0.55
visual_servo.post_avoid_recover_time_s: 0.1
visual_servo.recover_timeout_s: 0.1
visual_servo.recover_angular_speed_radps: 0.35
visual_servo.search_angular_speed_radps: 0.85
```

## Debug Commands

Check lidar is publishing:

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan --field ranges
```

Compare desired versus final velocity:

```bash
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
```

If `/cmd_vel_raw` commands forward motion but `/cmd_vel` has reduced or zero `linear.x`, the safety filter is actively slowing or blocking the robot.

## What This Is Not

This is not full navigation or path planning. The robot does not build a map or plan a route around the wall. It is a reactive safety layer that prevents blindly driving into obstacles while preserving the vision-guided behavior.

## AVOID Behavior

When the robot becomes blocked, the safety layer commits to an avoidance maneuver:

```text
BLOCKED -> AVOID -> CLEAR/SLOW/BLOCKED
```

During `AVOID`, the robot keeps turning in the direction that looked clearer. It uses a narrower center cone to decide whether it is safe to creep forward, so a wall beside the robot does not freeze motion forever after the direct center path opens. The default AVOID command is a short, wider arc rather than a long spin.

If the target disappears behind the obstacle, `visual_servo` enters `RECOVER` before normal `SEARCH`. While safety is still in `AVOID`, recovery is held back because safety owns the wheel command. When `/safety/state` leaves `AVOID`, the controller starts a fresh short `RECOVER` turn toward the last side where the target was seen.

If `RECOVER` still does not reacquire the target, `SEARCH` keeps rotating in one direction for a full scan. It does not bounce left and right in a small window, because after obstacle avoidance the target may be far outside the camera's current field of view.

This helps the robot escape the repeated `SLOW/CLEAR/BLOCKED` loop without pretending to be a full planner.
