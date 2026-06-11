# Autonomous Exploration

This milestone adds a small frontier explorer for SLAM practice.

Instead of manually clicking map goals, the robot watches the live occupancy map and chooses goals near frontiers: known free cells that touch unknown cells.

## Concept

SLAM produces an occupancy grid:

```text
-1  unknown
 0  free
100 occupied
```

A frontier is useful because driving there should reveal more unknown space with the lidar.

The first-pass explorer:

1. subscribes to `/map`
2. looks up the robot pose with TF: `map -> base_link`
3. finds free cells adjacent to unknown cells
4. clusters nearby frontier cells
5. scores each cluster by information gain and distance
6. sends the best candidate to Nav2 on `/navigate_to_pose`
7. repeats until `max_goals` is reached

This is intentionally educational. It is not a full production exploration stack.

## Build

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run The Explorer Demo

```bash
ros2 launch vision_guided_robot demo_slam_explore.launch.py rviz:=true max_goals:=4
```

Watch these topics:

```bash
ros2 topic echo /explorer/state
ros2 topic echo /explorer/goal
```

Useful states:

```text
WAITING_FOR_MAP
WAITING_FOR_TF
WAITING_FOR_NAV2
SENDING_GOAL
NAVIGATING
GOAL_SUCCEEDED
GOAL_FAILED
NO_FRONTIER
DONE
```

## RViz

The explorer publishes:

```text
/explorer/goal
/explorer/frontiers
```

In RViz, add:

- `Pose` for `/explorer/goal`
- `MarkerArray` for `/explorer/frontiers`

The markers show candidate frontier goals. The selected goal is the best-scoring candidate.

## Record A Validation Bag

Start the demo in one terminal, then record in another:

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record -o bags/final_frontier_exploration \
  /cmd_vel \
  /odom \
  /scan \
  /map \
  /tf \
  /explorer/state \
  /explorer/goal \
  /navigate_to_pose/_action/status
```

After the explorer reaches `DONE`, stop the bag with `Ctrl+C`.

Analyze:

```bash
python3 tools/analyze_bag.py bags/final_frontier_exploration
```

Validated result:

```text
bag: bags/final_frontier_exploration
duration_s: 245.86
explorer_goal_samples: 3
map_samples: 251
map_size: 269x242 @ 0.050 m/px
max_linear_mps: 1.500
max_angular_radps: 2.800
explorer_state_counts:
  COOLDOWN: 1
  GOAL_SUCCEEDED: 2
  NAVIGATING: 32
  SENDING_GOAL: 3
explorer_success: True
success: True
```

## Tuning

If the robot chooses goals too close to obstacles:

```bash
ros2 launch vision_guided_robot demo_slam_explore.launch.py \
  obstacle_clearance_m:=0.35
```

If goals are too far away:

```bash
ros2 launch vision_guided_robot demo_slam_explore.launch.py \
  max_goal_distance_m:=2.0
```

If it stops too soon:

```bash
ros2 launch vision_guided_robot demo_slam_explore.launch.py \
  max_goals:=8
```

## Robotics Concepts

- Occupancy grids discretize the world into cells.
- Frontiers are the boundary between known and unknown space.
- Exploration is goal generation, not low-level control.
- Nav2 still owns path planning, local control, obstacle avoidance, and recovery.
- TF is required because the explorer must score goals relative to the robot pose in the map frame.

## Limitations

- It does not blacklist failed frontiers yet.
- It does not reason about loop closure quality.
- It scores frontier clusters with a simple heuristic.
- It sends one goal at a time to Nav2 instead of building a longer exploration route.
