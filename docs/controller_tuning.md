# Controller Tuning

This experiment compares controller settings on the same ball pose.

Use the scenario:

```bash
ball_x:=3.0 ball_y:=2.0
```

The controller publishes `/cmd_vel`, and the finite-state behavior publishes `/visual_servo/state`.

## Metrics

Use `tools/analyze_bag.py` to compare:

- `success`
- `first_stop_time_s`
- `max_linear_mps`
- `max_angular_radps`
- `state_time_s`
- notes about overshoot, oscillation, or target loss

## Presets

| Preset | `linear_kp` | `angular_kp` | `max_linear_speed_mps` | `max_angular_speed_radps` | Expected Behavior |
| --- | ---: | ---: | ---: | ---: | --- |
| conservative | 0.45 | 1.6 | 0.35 | 1.2 | Slow and stable |
| fast | 0.8 | 1.4 | 0.8 | 1.2 | Faster approach, still moderate turning |
| default/aggressive | 1.0 | 2.2 | 1.2 | 1.8 | Fastest, higher risk of overshoot or oscillation |

## Run Pattern

Terminal 1 launches the sim. Terminal 2 records the bag.

Stop the bag only after the robot reaches `STOP`.

## Results Table

| Preset | Bag | Success | First Stop Time | Max Linear | Max Angular | State Timing Notes | Behavior Notes |
| --- | --- | --- | ---: | ---: | ---: | --- | --- |
| conservative | `bags/tune_conservative_3p0_y_2p0` |  |  |  |  |  |  |
| fast | `bags/tune_fast_3p0_y_2p0` |  |  |  |  |  |  |
| default/aggressive | `bags/tune_default_aggressive_3p0_y_2p0` |  |  |  |  |  |  |

## Robotics Concept

Increasing speed is not free. Higher gains can reduce time-to-target, but they also increase the risk of oscillation, overshoot, actuator saturation, and target loss. A good controller is not just fast; it is fast enough while remaining stable.
