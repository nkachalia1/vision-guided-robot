# Experiments

Use launch arguments to test different ball starting positions without editing SDF files.

## Run A Position

```bash
cd ~/vision_guided_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_guided_robot sim.launch.py ball_x:=1.5 ball_y:=0.6
```

`ball_x` is forward distance from the world origin. `ball_y` is lateral offset. Positive `ball_y` starts the ball to one side of the robot, and negative `ball_y` starts it on the other side.

## Test Matrix

| Run | `ball_x` | `ball_y` | Saw Ball Initially | `x` Moved Toward `0` | `z` Decreased | Stopped Near `0.45 m` | Notes |
| --- | ---: | ---: | --- | --- | --- | --- | --- |
| 1 | 1.5 | 0.6 |  |  |  |  |  |
| 2 | 1.5 | -0.6 |  |  |  |  |  |
| 3 | 2.0 | 0.8 |  |  |  |  |  |
| 4 | 2.0 | -0.8 |  |  |  |  |  |

The default ground plane is large enough for distance experiments out to several meters. If the robot falls before reaching the ball, that is a world-size failure rather than a detector or controller failure.

## Observe The Loop

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/vision_guided_robot_ws/install/setup.bash
ros2 topic echo /ball/relative_position
```

Success looks like:

- `x` starts away from zero, then moves toward `0.0`.
- `z` decreases as the robot approaches.
- `/cmd_vel` becomes zero when the robot stops near the ball.

## Record And Analyze State

Record the target estimate, command, and behavior state:

```bash
ros2 bag record -o bags/state_test_1p5_y_0p6 \
  /ball/relative_position \
  /cmd_vel \
  /visual_servo/state
```

Analyze the bag:

```bash
python3 tools/analyze_bag.py bags/state_test_1p5_y_0p6
```

The analyzer reports target metrics, velocity metrics, stop confirmation, state counts, and estimated time spent in each state.

## Controller Tuning

The launch file exposes controller gains and speed limits:

```bash
ros2 launch vision_guided_robot sim.launch.py \
  ball_x:=3.0 \
  ball_y:=2.0 \
  linear_kp:=0.8 \
  angular_kp:=1.4 \
  max_linear_speed_mps:=0.8 \
  max_angular_speed_radps:=1.2
```

See `docs/controller_tuning.md` for the full tuning protocol.

## Interview Question

What changed architecturally when the ball moved from the world file into the launch file?

Answer: the world became the static environment, while launch arguments became the experiment setup. That separation makes repeated trials faster and easier to reproduce.
