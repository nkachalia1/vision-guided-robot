# Final Navigation Report

This report summarizes the navigation stack evolution:

1. custom A* planner and path follower
2. custom recovery behavior
3. Nav2 baseline integration
4. Nav2 tuning
5. two-obstacle comparison
6. forced recovery and abort behavior
7. saved-map AMCL localization
8. AMCL/Nav2 wall-passing route

The important robotics lesson is that "navigation works" is not one result. A robot can succeed, fail safely, recover, or abort, and each outcome tells you something different about the system.

## Summary Table

| Run | Stack | Scenario | Result | Final Pose | Goal Error | Motion Span | Recovery Evidence |
| --- | --- | --- | --- | ---: | ---: | ---: | --- |
| `final_recovery_behavior_run2` | Custom | planner + recovery | Pass | `(1.961, 0.717)` | `0.092 m` | n/a | `RECOVERING: 127`, `COSTMAP_CLEARED: 2` |
| `final_nav2_first_goal_run2` | Nav2 | baseline goal | Pass | `(1.864, 0.688)` | `0.176 m` | `55.25 s` | not needed |
| `final_nav2_fast_goal` | Nav2 | faster controller | Pass | `(1.863, 0.692)` | `0.175 m` | `45.50 s` | not needed |
| `final_nav2_tight_goal` | Nav2 | tighter tolerance | Pass | `(1.913, 0.720)` | `0.118 m` | `42.60 s` | action `SUCCEEDED` |
| `final_two_obstacle_custom` | Custom | two obstacles | Pass | `(1.925, 0.848)` | `0.089 m` | `31.11 s` | not triggered |
| `final_two_obstacle_nav2` | Nav2 | two obstacles | Pass | `(1.912, 0.718)` | `0.120 m` | `42.10 s` | not triggered |
| `final_forced_recovery_custom` | Custom | blocked goal | Expected fail | `(0.293, 1.121)` | `1.159 m` | `162.50 s` | `custom_recovery_detected: True` |
| `final_forced_recovery_nav2_run2` | Nav2 | blocked goal | Expected abort | `(1.165, 0.371)` | `0.046 m` | `76.55 s` | action `ABORTED` |
| `final_nav2_amcl_clear_goal` | Nav2 + AMCL | saved map clear goal | Pass | `(0.714, -0.515)` | `0.120 m` | `48.22 s` | `/amcl_pose: 37 samples` |
| `final_nav2_amcl_wall_pass_success` | Nav2 + AMCL | saved map wall pass | Pass with endpoint caveat | `(2.372, 0.787)` | `0.372 m` | `33.65 s` | `/navigate_through_poses` action `SUCCEEDED` |

## Command And Speed Table

| Run | Max Linear | Max Angular | Avg Linear | Avg Angular | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| `final_recovery_behavior_run2` | `0.900 m/s` | `1.800 rad/s` | n/a | n/a | Custom recovery reached the normal goal after backup/rotate/clear cycles. |
| `final_nav2_first_goal_run2` | `0.450 m/s` | `0.316 rad/s` | `0.282 m/s` | `0.114 rad/s` | First Nav2 validation. |
| `final_nav2_fast_goal` | `0.600 m/s` | `0.442 rad/s` | `0.309 m/s` | `0.116 rad/s` | Faster movement without worse final error. |
| `final_nav2_tight_goal` | `0.600 m/s` | `0.442 rad/s` | `0.318 m/s` | `0.118 rad/s` | Best Nav2 goal-reaching run so far. |
| `final_two_obstacle_custom` | `0.900 m/s` | `1.542 rad/s` | `0.332 m/s` | `0.313 rad/s` | Faster but more aggressive turning. |
| `final_two_obstacle_nav2` | `0.600 m/s` | `0.442 rad/s` | `0.313 m/s` | `0.117 rad/s` | Slower but smoother. |
| `final_forced_recovery_custom` | `0.900 m/s` | `1.800 rad/s` | `0.251 m/s` | `0.777 rad/s` | Repeated recovery attempts; goal intentionally unreachable. |
| `final_forced_recovery_nav2_run2` | `0.600 m/s` | `0.442 rad/s` | `0.134 m/s` | `0.131 rad/s` | Nav2 approached the blocked goal, then aborted under strict tolerance. |
| `final_nav2_amcl_wall_pass_success` | `1.800 m/s` | `2.200 rad/s` | `1.070 m/s` | `0.789 rad/s` | Fastest validated map-localized wall-pass run. Endpoint accuracy is loose by design. |

## Manual Nav2 Behavior Validation

The Nav2 behavior server was validated separately in open space:

| Action | Open-Space Result | Blocked-Goal Result | Interpretation |
| --- | --- | --- | --- |
| `/backup` | `SUCCEEDED` | `ABORTED` | Backup server works; blocked scene made reverse motion unsafe. |
| `/spin` | `SUCCEEDED` | `ABORTED` | Spin server works; blocked scene made spin unsafe. |

This separates two things that are easy to mix up:

```text
Behavior server broken?      No.
Recovery action available?   Yes.
Recovery motion always safe? No.
```

## What We Learned

The custom stack is easier to inspect. It exposes states like `FOLLOW_PATH`, `RECOVERING`, and `COSTMAP_CLEARED`, so it is good for learning the mechanics of planning and recovery.

Nav2 is more production-like. It uses lifecycle nodes, action servers, planner/controller/behavior servers, costmaps, and behavior-tree navigation. It is harder to inspect at first, but it is the standard ROS 2 navigation architecture.

The two-obstacle comparison showed both systems can plan around obstacles. The forced blocker test showed a more subtle robotics behavior: sometimes the correct result is not reaching the goal, but detecting that the goal is invalid or unsafe and aborting.

## Best Current Results

| Category | Best Run | Why |
| --- | --- | --- |
| Best custom normal navigation | `final_two_obstacle_custom` | Fastest successful two-obstacle run, `31.11 s` motion span. |
| Best Nav2 normal navigation | `final_nav2_tight_goal` | Tightest validated Nav2 goal error, `0.118 m`. |
| Best custom recovery evidence | `final_forced_recovery_custom` | `custom_recovery_detected: True` with repeated recovery cycles. |
| Best Nav2 recovery evidence | manual `/backup` and `/spin` tests | Behavior actions succeeded in open space and correctly aborted when unsafe. |
| Best map-localized Nav2 run | `final_nav2_amcl_clear_goal` | Saved map published, AMCL localized, and Nav2 completed a `map`-frame goal. |
| Best map-localized wall pass | `final_nav2_amcl_wall_pass_success` | AMCL stayed active, Nav2 planned a staged route around the wall layout, and `/navigate_through_poses` finished `SUCCEEDED`. |

## Map Localization Result

The saved-map AMCL milestone is validated by:

```text
bags/final_nav2_amcl_clear_goal
```

Key evidence:

| Metric | Result |
| --- | ---: |
| Map samples | `1` |
| Map size | `120 x 120 @ 0.050 m/px` |
| AMCL pose samples | `37` |
| Nav2 plan samples | `47` |
| Action status | `EXECUTING -> SUCCEEDED` |
| Final odom pose | `(0.714, -0.515)` |
| Final AMCL pose | `(0.759, -0.529)` |
| Goal | `(0.8, -0.6)` in `map` |
| Goal error | `0.120 m` |
| Result | `success: True` |

This proves the standard localization chain:

```text
map_server -> /map
AMCL -> map -> odom
Gazebo odometry -> odom -> base_link
Nav2 -> map-frame NavigateToPose goal
```

## Map Wall-Passing Result

The harder wall-passing milestone is validated by:

```text
bags/final_nav2_amcl_wall_pass_success
```

Key evidence:

| Metric | Result |
| --- | ---: |
| AMCL pose samples | `70` |
| Nav2 plan samples | `12` |
| Action status | `EXECUTING -> SUCCEEDED` |
| Final odom pose | `(2.372, 0.787)` |
| Final AMCL pose | `(2.257, 0.710)` |
| Nominal goal | `(2.0, 0.8)` in `map` |
| Goal error to nominal endpoint | `0.372 m` |
| Max linear speed | `1.800 m/s` |
| Max angular speed | `2.200 rad/s` |
| Motion command span | `33.65 s` |
| Result | `success: True` |

This proves the robot can use the saved map, AMCL localization, and Nav2 to get around the wall layout. It is not a tight final-position proof. The route succeeds because the wall-pass profile gives the planner enough tolerance to choose a feasible staged path through a constrained space.

## Next Engineering Step

The navigation, saved-map localization, and AMCL/Nav2 wall-passing milestones are strong enough to close. The next major robotics step should be either:

1. tighten final endpoint accuracy after the wall-pass route, or
2. start SLAM Toolbox mapping.

Choose endpoint tuning if the goal is to make map-based Nav2 stop closer to the exact `(2.0, 0.8)` target. Choose SLAM Toolbox if the goal is to learn how robots build maps from sensor data.
