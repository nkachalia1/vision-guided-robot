# Custom Planner vs Nav2 Comparison

This compares the validated custom planned-navigation stack against the first validated Nav2 run.

The runs are not identical experiments, but they are close enough to teach the engineering tradeoff:

- both use the same Gazebo robot
- both use `/odom`, `/scan`, `/tf`, and `/cmd_vel`
- both drive toward approximately `(2.0, 0.8)` in the `odom` frame
- both reach the goal region successfully

## Bags Compared

Custom planner:

```bash
python3 tools/analyze_bag.py bags/final_recovery_behavior_run2
```

Nav2:

```bash
python3 tools/analyze_bag.py bags/final_nav2_first_goal_run2
```

## Results Table

| Metric | Custom Planner + Recovery | Nav2 |
| --- | ---: | ---: |
| Bag | `final_recovery_behavior_run2` | `final_nav2_first_goal_run2` |
| Duration | 121.01 s | 71.82 s |
| Final odom | `(1.961, 0.717)` | `(1.864, 0.688)` |
| Goal error to `(2.0, 0.8)` | ~0.092 m | 0.176 m |
| Odom displacement | 2.052 m | 1.987 m |
| Max linear speed | 0.900 m/s | 0.450 m/s |
| Max angular speed | 1.800 rad/s | 0.316 rad/s |
| Custom planned path samples | 39 | 0 |
| Nav2 plan samples | 0 | 54 |
| Recovery behavior | 2 custom recovery cycles | Nav2 behavior tree available, not visibly needed |
| Success signal | `success: True`, `planner_success: True` | `nav2_odom_success: True`, action result `SUCCEEDED` |

## What The Numbers Mean

The custom planner reached closer to the requested goal:

```text
custom final error: ~0.092 m
Nav2 final error:   ~0.176 m
```

Nav2 finished the recorded run faster even though its maximum commanded speed was lower:

```text
custom max linear speed: 0.900 m/s
Nav2 max linear speed:   0.450 m/s
```

That happens because speed limit is not the whole story. The custom run spent a lot of time in recovery:

```text
RECOVERING: 63.05 s
FOLLOW_PATH: 53.28 s
```

Nav2 produced many more path updates:

```text
custom planned_path_samples: 39
Nav2 nav2_plan_samples:     54
```

This reflects a major architectural difference. The custom planner is easier to understand and inspect. Nav2 is a larger stack that continuously coordinates planner, controller, costmaps, behavior tree logic, and lifecycle nodes.

## Engineering Interpretation

The custom planner is valuable because it taught the building blocks:

- occupancy grids
- obstacle inflation
- scan projection
- path planning
- path following
- recovery behavior
- validation with rosbag

Nav2 is valuable because it packages those ideas into a standard production-style ROS 2 architecture:

- lifecycle-managed navigation nodes
- global and local costmaps
- planner server
- controller server
- behavior server
- behavior-tree navigator
- action interface through `/navigate_to_pose`

The right lesson is not "custom planner bad, Nav2 good." The right lesson is:

```text
Building the custom planner teaches what Nav2 is doing.
Running Nav2 teaches how production ROS navigation is organized.
```

## Interview-Style Summary

I built a custom A* and pure-pursuit-style navigation stack first so I could understand the mechanics of planning, costmaps, path following, and recovery. Then I integrated Nav2 against the same Gazebo robot and sensor topics. In the first comparison, the custom stack reached closer to the goal but spent significant time in recovery, while Nav2 reached the goal faster with smoother, lower-speed commands and a standard action-based interface.

## Next Comparison Experiments

1. Run `docs/forced_recovery.md` to validate the blocked-goal recovery stress test.
2. Review `docs/navigation_final_report.md` for the final custom-vs-Nav2 navigation summary.
3. Continue from the validated saved-map AMCL milestone into harder map-frame wall-passing or SLAM Toolbox mapping.

## Faster Nav2 Run

After increasing Nav2 forward speed from `0.45` to `0.60`, the first fast bag still succeeded:

| Metric | Baseline Nav2 | Faster Nav2 |
| --- | ---: | ---: |
| Bag | `final_nav2_first_goal_run2` | `final_nav2_fast_goal` |
| Duration | 71.82 s | 84.17 s |
| Final odom | `(1.864, 0.688)` | `(1.863, 0.692)` |
| Goal error | 0.176 m | 0.175 m |
| Odom displacement | 1.987 m | 1.987 m |
| Max linear speed | 0.450 m/s | 0.600 m/s |
| Max angular speed | 0.316 rad/s | 0.442 rad/s |
| Average linear command | 0.282 m/s | 0.309 m/s |
| Motion command span | 55.25 s | 45.50 s |
| Nav2 plan samples | 54 | 45 |
| Success | True | True |

The total bag duration was longer for the fast run because recording started earlier before motion:

```text
baseline first_motion_time_s: 4.62
fast first_motion_time_s:     18.88
```

The fairer motion comparison is `motion_command_span_s`:

```text
baseline motion_command_span_s: 55.25
fast motion_command_span_s:     45.50
```

So the faster Nav2 config did improve movement time by about `9.75 s`, while keeping nearly the same final goal error:

```text
baseline goal error: 0.176 m
fast goal error:     0.175 m
```

## Tighter Nav2 Run

After keeping the faster `0.60 m/s` speed and tightening Nav2 goal tolerance from `0.18 m` to `0.12 m`, the tight-goal run improved both accuracy and motion span:

| Metric | Faster Nav2 | Tight Nav2 |
| --- | ---: | ---: |
| Bag | `final_nav2_fast_goal` | `final_nav2_tight_goal` |
| Duration | 84.17 s | 76.71 s |
| Final odom | `(1.863, 0.692)` | `(1.913, 0.720)` |
| Goal error | 0.175 m | 0.118 m |
| Goal tolerance | 0.180 m | 0.120 m |
| Odom displacement | 1.987 m | 2.044 m |
| Max linear speed | 0.600 m/s | 0.600 m/s |
| Max angular speed | 0.442 rad/s | 0.442 rad/s |
| Average linear command | 0.309 m/s | 0.318 m/s |
| Motion command span | 45.50 s | 42.60 s |
| Nav2 plan samples | 45 | 42 |
| Action status captured | Not in bag | `EXECUTING -> SUCCEEDED` |
| Success | True | True |

The tight configuration is the best Nav2 result so far. It reached the same `(2.0, 0.8)` goal with `0.118 m` final error, inside the tighter `0.120 m` threshold, while also shortening the actual command-motion span by another `2.90 s` compared with the fast run.

## Two-Obstacle Run

The two-obstacle scenario used the same Gazebo walls for both systems:

```text
wall 1: x=1.2, y=0.4
wall 2: x=1.6, y=-0.45
goal:   x=2.0, y=0.8
```

Both stacks succeeded:

| Metric | Custom Planner | Nav2 |
| --- | ---: | ---: |
| Bag | `final_two_obstacle_custom` | `final_two_obstacle_nav2` |
| Duration | 51.38 s | 70.45 s |
| Final odom | `(1.925, 0.848)` | `(1.912, 0.718)` |
| Goal error | ~0.089 m | 0.120 m |
| Odom displacement | 1.928 m | 2.043 m |
| Max linear speed | 0.900 m/s | 0.600 m/s |
| Max angular speed | 1.542 rad/s | 0.442 rad/s |
| Average linear command | 0.332 m/s | 0.313 m/s |
| Average angular command | 0.313 rad/s | 0.117 rad/s |
| Motion command span | 31.11 s | 42.10 s |
| Plan samples | 17 custom `/planned_path` | 41 Nav2 `/plan` |
| Success signal | `FOLLOW_PATH -> DONE` | `EXECUTING -> SUCCEEDED` |
| Recovery behavior | Not triggered | Not triggered |

The custom stack was faster and ended closer to the goal in this specific layout. Nav2 was smoother and used lower angular velocity. Since neither system triggered recovery, this is best classified as a validated two-obstacle navigation comparison, not a completed recovery-stress comparison.

## Forced Recovery Stress Test

The next scenario deliberately places the goal inside a large blocker:

```text
large blocker: x=1.2, y=0.4, size=1.20 m x 1.60 m
blocked goal:  x=1.2, y=0.4
```

This is not a normal success test. The useful evidence is:

```text
custom_recovery_detected: True
```

or:

```text
nav2_action_state_counts:
  ABORTED: ...
```

The custom stack exposed recovery cycles directly. Nav2 approached the blocked/tight goal and aborted, while manual `/backup` and `/spin` behavior actions succeeded in open space and aborted near the blocker because the recovery motions were collision-constrained.

The full summary is in `docs/navigation_final_report.md`.
