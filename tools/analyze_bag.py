#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict
import math
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPIC = "/ball/relative_position"
CMD_TOPIC = "/cmd_vel"
STATE_TOPIC = "/visual_servo/state"
SAFETY_STATE_TOPIC = "/safety/state"
WAYPOINT_STATE_TOPIC = "/waypoint/state"
WAYPOINT_PROGRESS_TOPIC = "/waypoint/progress"
MISSION_STATE_TOPIC = "/mission/state"
PLANNER_STATE_TOPIC = "/planner/state"
PLANNED_PATH_TOPIC = "/planned_path"
NAV2_PLAN_TOPIC = "/plan"
EXPLORER_STATE_TOPIC = "/explorer/state"
EXPLORER_GOAL_TOPIC = "/explorer/goal"
NAV2_STATUS_TOPICS = {
    "/navigate_to_pose/_action/status",
    "/navigate_through_poses/_action/status",
}
ODOM_TOPIC = "/odom"
GOAL_TOPIC = "/goal_pose"
AMCL_POSE_TOPIC = "/amcl_pose"
MAP_TOPIC = "/map"
TF_TOPICS = {"/tf", "/tf_static"}
DEFAULT_NAV2_GOAL_X_M = 2.0
DEFAULT_NAV2_GOAL_Y_M = 0.8
DEFAULT_NAV2_GOAL_TOLERANCE_M = 0.18


def read_topic_types(db_path: Path) -> dict[int, tuple[str, str]]:
    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT id, name, type FROM topics").fetchall()
    conn.close()
    return {topic_id: (name, msg_type) for topic_id, name, msg_type in rows}


def main() -> None:
    args = parse_args()

    bag_dir = Path(args.bag_dir)
    db_files = sorted(bag_dir.glob("*.db3"))
    if not db_files:
        raise SystemExit(f"no .db3 files found in {bag_dir}")

    db_path = db_files[0]
    topics = read_topic_types(db_path)

    target_type = get_message("geometry_msgs/msg/PointStamped")
    twist_type = get_message("geometry_msgs/msg/Twist")
    state_type = get_message("std_msgs/msg/String")
    odom_type = get_message("nav_msgs/msg/Odometry")
    goal_type = get_message("geometry_msgs/msg/PoseStamped")
    amcl_pose_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")
    map_type = get_message("nav_msgs/msg/OccupancyGrid")
    path_type = get_message("nav_msgs/msg/Path")
    action_status_type = get_message("action_msgs/msg/GoalStatusArray")
    tf_type = get_message("tf2_msgs/msg/TFMessage")

    target_samples: list[tuple[int, float, float]] = []
    cmd_samples: list[tuple[int, float, float]] = []
    state_samples: list[tuple[int, str]] = []
    safety_state_samples: list[tuple[int, str]] = []
    waypoint_state_samples: list[tuple[int, str]] = []
    waypoint_progress_samples: list[tuple[int, str]] = []
    mission_state_samples: list[tuple[int, str]] = []
    planner_state_samples: list[tuple[int, str]] = []
    explorer_state_samples: list[tuple[int, str]] = []
    planned_path_samples: list[tuple[int, int]] = []
    nav2_plan_samples: list[tuple[int, int]] = []
    nav2_status_samples: list[tuple[int, str]] = []
    action_status_samples_by_topic: dict[str, list[tuple[int, str]]] = defaultdict(list)
    odom_samples: list[tuple[int, float, float]] = []
    goal_samples: list[tuple[int, float, float]] = []
    explorer_goal_samples: list[tuple[int, float, float]] = []
    amcl_pose_samples: list[tuple[int, float, float]] = []
    map_samples: list[tuple[int, int, int, float]] = []
    tf_samples_by_pair: dict[tuple[str, str], list[tuple[int, float, float, float]]] = defaultdict(list)
    all_timestamps: list[int] = []

    conn = sqlite3.connect(db_path)
    for topic_id, timestamp, data in conn.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
    ):
        topic_name, topic_type = topics[topic_id]
        all_timestamps.append(timestamp)

        if (
            topic_type == "action_msgs/msg/GoalStatusArray"
            and topic_name.endswith("/_action/status")
        ):
            msg = deserialize_message(data, action_status_type)
            for status in msg.status_list:
                status_name = goal_status_name(status.status)
                action_status_samples_by_topic[topic_name].append((timestamp, status_name))
                if topic_name in NAV2_STATUS_TOPICS:
                    nav2_status_samples.append((timestamp, status_name))

        if topic_name == TARGET_TOPIC:
            msg = deserialize_message(data, target_type)
            target_samples.append((timestamp, msg.point.x, msg.point.z))

        elif topic_name == CMD_TOPIC:
            msg = deserialize_message(data, twist_type)
            cmd_samples.append((timestamp, msg.linear.x, msg.angular.z))

        elif topic_name == STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            state_samples.append((timestamp, msg.data))

        elif topic_name == SAFETY_STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            safety_state_samples.append((timestamp, msg.data))

        elif topic_name == WAYPOINT_STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            waypoint_state_samples.append((timestamp, msg.data))

        elif topic_name == WAYPOINT_PROGRESS_TOPIC:
            msg = deserialize_message(data, state_type)
            waypoint_progress_samples.append((timestamp, msg.data))

        elif topic_name == MISSION_STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            mission_state_samples.append((timestamp, msg.data))

        elif topic_name == PLANNER_STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            planner_state_samples.append((timestamp, msg.data))

        elif topic_name == EXPLORER_STATE_TOPIC:
            msg = deserialize_message(data, state_type)
            explorer_state_samples.append((timestamp, msg.data))

        elif topic_name == PLANNED_PATH_TOPIC:
            msg = deserialize_message(data, path_type)
            planned_path_samples.append((timestamp, len(msg.poses)))

        elif topic_name == NAV2_PLAN_TOPIC:
            msg = deserialize_message(data, path_type)
            nav2_plan_samples.append((timestamp, len(msg.poses)))

        elif topic_name == ODOM_TOPIC:
            msg = deserialize_message(data, odom_type)
            odom_samples.append((timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y))

        elif topic_name == GOAL_TOPIC:
            msg = deserialize_message(data, goal_type)
            goal_samples.append((timestamp, msg.pose.position.x, msg.pose.position.y))

        elif topic_name == EXPLORER_GOAL_TOPIC:
            msg = deserialize_message(data, goal_type)
            explorer_goal_samples.append(
                (timestamp, msg.pose.position.x, msg.pose.position.y)
            )

        elif topic_name == AMCL_POSE_TOPIC:
            msg = deserialize_message(data, amcl_pose_type)
            amcl_pose_samples.append(
                (timestamp, msg.pose.pose.position.x, msg.pose.pose.position.y)
            )

        elif topic_name == MAP_TOPIC:
            msg = deserialize_message(data, map_type)
            map_samples.append(
                (
                    timestamp,
                    msg.info.width,
                    msg.info.height,
                    msg.info.resolution,
                )
            )

        elif topic_name in TF_TOPICS:
            msg = deserialize_message(data, tf_type)
            for transform in msg.transforms:
                parent = normalize_frame_id(transform.header.frame_id)
                child = normalize_frame_id(transform.child_frame_id)
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                tf_samples_by_pair[(parent, child)].append(
                    (
                        timestamp,
                        translation.x,
                        translation.y,
                        yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                    )
                )

    conn.close()

    if not all_timestamps:
        raise SystemExit("bag has no messages")

    bag_start_t = all_timestamps[0]
    bag_end_t = all_timestamps[-1]
    bag_duration_s = ns_to_s(bag_end_t - bag_start_t)

    max_linear = max((abs(sample[1]) for sample in cmd_samples), default=0.0)
    max_angular = max((abs(sample[2]) for sample in cmd_samples), default=0.0)
    moving_samples = [
        sample for sample in cmd_samples if abs(sample[1]) >= 0.01 or abs(sample[2]) >= 0.01
    ]
    average_abs_linear = average_abs([sample[1] for sample in cmd_samples])
    average_abs_angular = average_abs([sample[2] for sample in cmd_samples])

    stopped_samples = [
        sample for sample in cmd_samples if abs(sample[1]) < 0.01 and abs(sample[2]) < 0.01
    ]
    first_stop_time_s = None
    if stopped_samples:
        first_stop_time_s = ns_to_s(stopped_samples[0][0] - bag_start_t)

    state_counts = count_states(state_samples)
    state_durations_s = estimate_state_durations_s(state_samples, bag_end_t)
    safety_state_counts = count_states(safety_state_samples)
    safety_state_durations_s = estimate_state_durations_s(safety_state_samples, bag_end_t)
    waypoint_state_counts = count_states(waypoint_state_samples)
    waypoint_state_durations_s = estimate_state_durations_s(waypoint_state_samples, bag_end_t)
    mission_state_counts = count_states(mission_state_samples)
    mission_state_durations_s = estimate_state_durations_s(mission_state_samples, bag_end_t)
    planner_state_counts = count_states(planner_state_samples)
    planner_state_durations_s = estimate_state_durations_s(planner_state_samples, bag_end_t)
    explorer_state_counts = count_states(explorer_state_samples)
    explorer_state_durations_s = estimate_state_durations_s(explorer_state_samples, bag_end_t)
    nav2_status_counts = count_states(nav2_status_samples)
    nav2_status_durations_s = estimate_state_durations_s(nav2_status_samples, bag_end_t)
    waypoint_success = bool(waypoint_state_samples and waypoint_state_samples[-1][1] == "DONE")
    mission_success = bool(mission_state_samples and mission_state_samples[-1][1] == "DONE")
    planner_success = any(state == "PLANNED" for _, state in planner_state_samples)
    custom_recovery_detected = detect_custom_recovery(
        waypoint_state_samples,
        mission_state_samples,
        planner_state_samples,
    )
    nav2_behavior_recovery_detected = detect_nav2_behavior_recovery(
        action_status_samples_by_topic
    )
    nav2_success = any(status == "SUCCEEDED" for _, status in nav2_status_samples)
    explorer_success = any(
        state in {"GOAL_SUCCEEDED", "DONE"} for _, state in explorer_state_samples
    )
    map_base_samples = estimate_map_base_samples(tf_samples_by_pair)
    nav2_goal_pose_source = "map_tf" if map_base_samples else "odom"
    nav2_goal_error_m = estimate_nav2_goal_error_m(
        odom_samples,
        goal_samples,
        map_base_samples=map_base_samples,
        default_goal_x_m=args.nav2_goal_x_m,
        default_goal_y_m=args.nav2_goal_y_m,
    )
    nav2_odom_success = (
        (bool(nav2_plan_samples) or bool(nav2_status_samples))
        and bool(cmd_samples)
        and nav2_goal_error_m is not None
        and nav2_goal_error_m <= args.nav2_goal_tolerance_m
        and len(stopped_samples) > 0
    )
    rerouting_detected = any(state == "REROUTING" for _, state in mission_state_samples)
    blocked_detected = any(state == "BLOCKED" for _, state in mission_state_samples)

    print(f"bag: {bag_dir}")
    print(f"duration_s: {bag_duration_s:.2f}")
    print(f"target_samples: {len(target_samples)}")
    print(f"cmd_samples: {len(cmd_samples)}")
    print(f"state_samples: {len(state_samples)}")
    print(f"waypoint_state_samples: {len(waypoint_state_samples)}")
    print(f"waypoint_progress_samples: {len(waypoint_progress_samples)}")
    print(f"mission_state_samples: {len(mission_state_samples)}")
    print(f"planner_state_samples: {len(planner_state_samples)}")
    print(f"explorer_state_samples: {len(explorer_state_samples)}")
    print(f"planned_path_samples: {len(planned_path_samples)}")
    print_path_summary("planned_path", planned_path_samples)
    print(f"nav2_plan_samples: {len(nav2_plan_samples)}")
    print_path_summary("nav2_plan", nav2_plan_samples)
    print(f"nav2_status_samples: {len(nav2_status_samples)}")
    print(f"odom_samples: {len(odom_samples)}")
    print(f"goal_samples: {len(goal_samples)}")
    print(f"explorer_goal_samples: {len(explorer_goal_samples)}")
    print(f"amcl_pose_samples: {len(amcl_pose_samples)}")
    print_map_summary(map_samples)
    if not target_samples:
        print("initial_x_m: n/a")
        print("final_x_m: n/a")
        print("initial_z_m: n/a")
        print("final_z_m: n/a")
        print("min_z_m: n/a")
        print_odom_summary(odom_samples)
        print_goal_summary(goal_samples)
        print_xy_summary("explorer_goal", explorer_goal_samples)
        print_amcl_summary(amcl_pose_samples)
        print_map_base_summary(map_base_samples)
        print(f"max_linear_mps: {max_linear:.3f}")
        print(f"max_angular_radps: {max_angular:.3f}")
        print(f"avg_abs_linear_mps: {average_abs_linear:.3f}")
        print(f"avg_abs_angular_radps: {average_abs_angular:.3f}")
        print_motion_summary(moving_samples, bag_start_t)
        print(f"stopped_samples: {len(stopped_samples)}")
        print_state_summary("visual_servo", state_counts, state_durations_s)
        print_state_summary("waypoint", waypoint_state_counts, waypoint_state_durations_s)
        print_progress_summary(waypoint_progress_samples)
        print_state_summary("mission", mission_state_counts, mission_state_durations_s)
        print_state_summary("planner", planner_state_counts, planner_state_durations_s)
        print_state_summary("explorer", explorer_state_counts, explorer_state_durations_s)
        print_state_summary("nav2_action", nav2_status_counts, nav2_status_durations_s)
        print_action_status_topic_summary(action_status_samples_by_topic, bag_end_t)
        print(f"custom_recovery_detected: {custom_recovery_detected}")
        print(f"nav2_behavior_recovery_detected: {nav2_behavior_recovery_detected}")
        print_state_summary("safety", safety_state_counts, safety_state_durations_s)
        print_nav2_goal_summary(
            nav2_goal_error_m,
            nav2_odom_success,
            nav2_goal_tolerance_m=args.nav2_goal_tolerance_m,
            pose_source=nav2_goal_pose_source,
        )
        print(f"rerouting_detected: {rerouting_detected}")
        print(f"blocked_detected: {blocked_detected}")
        print(f"planner_success: {planner_success}")
        print(f"nav2_success: {nav2_success or nav2_odom_success}")
        print(f"explorer_success: {explorer_success}")
        print(
            "success: "
            f"{waypoint_success or mission_success or nav2_success or nav2_odom_success or explorer_success}"
        )
        return

    initial_x = target_samples[0][1]
    final_x = target_samples[-1][1]
    initial_z = target_samples[0][2]
    final_z = target_samples[-1][2]
    min_z = min(sample[2] for sample in target_samples)

    success = (
        abs(final_x) < 0.08
        and 0.40 <= final_z <= 0.55
        and len(stopped_samples) > 0
    )

    print(f"initial_x_m: {initial_x:.3f}")
    print(f"final_x_m: {final_x:.3f}")
    print(f"initial_z_m: {initial_z:.3f}")
    print(f"final_z_m: {final_z:.3f}")
    print(f"min_z_m: {min_z:.3f}")
    print_odom_summary(odom_samples)
    print_goal_summary(goal_samples)
    print_xy_summary("explorer_goal", explorer_goal_samples)
    print_amcl_summary(amcl_pose_samples)
    print_map_base_summary(map_base_samples)
    print(f"max_linear_mps: {max_linear:.3f}")
    print(f"max_angular_radps: {max_angular:.3f}")
    print(f"avg_abs_linear_mps: {average_abs_linear:.3f}")
    print(f"avg_abs_angular_radps: {average_abs_angular:.3f}")
    print_motion_summary(moving_samples, bag_start_t)
    print(f"stopped_samples: {len(stopped_samples)}")
    if first_stop_time_s is not None:
        print(f"first_stop_time_s: {first_stop_time_s:.2f}")
    print_state_summary("visual_servo", state_counts, state_durations_s)
    print_state_summary("waypoint", waypoint_state_counts, waypoint_state_durations_s)
    print_progress_summary(waypoint_progress_samples)
    print_state_summary("mission", mission_state_counts, mission_state_durations_s)
    print_state_summary("planner", planner_state_counts, planner_state_durations_s)
    print_state_summary("explorer", explorer_state_counts, explorer_state_durations_s)
    print_state_summary("nav2_action", nav2_status_counts, nav2_status_durations_s)
    print_action_status_topic_summary(action_status_samples_by_topic, bag_end_t)
    print(f"custom_recovery_detected: {custom_recovery_detected}")
    print(f"nav2_behavior_recovery_detected: {nav2_behavior_recovery_detected}")
    print_state_summary("safety", safety_state_counts, safety_state_durations_s)
    print_nav2_goal_summary(
        nav2_goal_error_m,
        nav2_odom_success,
        nav2_goal_tolerance_m=args.nav2_goal_tolerance_m,
        pose_source=nav2_goal_pose_source,
    )
    print(f"rerouting_detected: {rerouting_detected}")
    print(f"blocked_detected: {blocked_detected}")
    print(f"planner_success: {planner_success}")
    print(f"nav2_success: {nav2_success or nav2_odom_success}")
    print(f"explorer_success: {explorer_success}")
    print(
        "success: "
        f"{success or waypoint_success or mission_success or nav2_success or nav2_odom_success or explorer_success}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze a ROS 2 bag from this project.")
    parser.add_argument("bag_dir", help="Path to the rosbag directory.")
    parser.add_argument(
        "--nav2-goal-x-m",
        type=float,
        default=DEFAULT_NAV2_GOAL_X_M,
        help="Default Nav2 goal x used when /goal_pose was not recorded.",
    )
    parser.add_argument(
        "--nav2-goal-y-m",
        type=float,
        default=DEFAULT_NAV2_GOAL_Y_M,
        help="Default Nav2 goal y used when /goal_pose was not recorded.",
    )
    parser.add_argument(
        "--nav2-goal-tolerance-m",
        type=float,
        default=DEFAULT_NAV2_GOAL_TOLERANCE_M,
        help="Goal tolerance used for Nav2 odometry-based success checks.",
    )
    return parser.parse_args()


def count_states(state_samples: list[tuple[int, str]]) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)
    for _, state in state_samples:
        counts[state] += 1
    return dict(sorted(counts.items()))


def estimate_state_durations_s(
    state_samples: list[tuple[int, str]],
    bag_end_t: int,
) -> dict[str, float]:
    durations: dict[str, float] = defaultdict(float)
    if not state_samples:
        return {}

    for index, (timestamp, state) in enumerate(state_samples):
        if index + 1 < len(state_samples):
            next_timestamp = state_samples[index + 1][0]
        else:
            next_timestamp = bag_end_t
        durations[state] += max(0.0, ns_to_s(next_timestamp - timestamp))

    return dict(sorted(durations.items()))


def print_state_summary(
    label: str,
    state_counts: dict[str, int],
    state_durations_s: dict[str, float],
) -> None:
    if not state_counts:
        print(f"{label}_states: none recorded")
        return

    print(f"{label}_state_counts:")
    for state, count in state_counts.items():
        print(f"  {state}: {count}")

    print(f"{label}_state_time_s:")
    for state, duration_s in state_durations_s.items():
        print(f"  {state}: {duration_s:.2f}")


def print_action_status_topic_summary(
    samples_by_topic: dict[str, list[tuple[int, str]]],
    bag_end_t: int,
) -> None:
    behavior_topics = {
        topic: samples
        for topic, samples in sorted(samples_by_topic.items())
        if topic not in NAV2_STATUS_TOPICS
    }
    if not behavior_topics:
        print("action_status_topics: none recorded")
        return

    print("action_status_topics:")
    for topic, samples in behavior_topics.items():
        counts = count_states(samples)
        durations = estimate_state_durations_s(samples, bag_end_t)
        print(f"  {topic}:")
        for state, count in counts.items():
            print(f"    {state}: {count}")
        print(f"  {topic}_time_s:")
        for state, duration_s in durations.items():
            print(f"    {state}: {duration_s:.2f}")


def detect_custom_recovery(
    waypoint_state_samples: list[tuple[int, str]],
    mission_state_samples: list[tuple[int, str]],
    planner_state_samples: list[tuple[int, str]],
) -> bool:
    return (
        any(state == "RECOVERING" for _, state in waypoint_state_samples)
        or any(state == "RECOVERING" for _, state in mission_state_samples)
        or any(state == "COSTMAP_CLEARED" for _, state in planner_state_samples)
    )


def detect_nav2_behavior_recovery(
    samples_by_topic: dict[str, list[tuple[int, str]]],
) -> bool:
    return any(
        topic not in NAV2_STATUS_TOPICS and bool(samples)
        for topic, samples in samples_by_topic.items()
    )


def print_odom_summary(odom_samples: list[tuple[int, float, float]]) -> None:
    if not odom_samples:
        print("initial_odom_xy_m: n/a")
        print("final_odom_xy_m: n/a")
        print("odom_displacement_m: n/a")
        return

    _, initial_x, initial_y = odom_samples[0]
    _, final_x, final_y = odom_samples[-1]
    displacement_m = ((final_x - initial_x) ** 2 + (final_y - initial_y) ** 2) ** 0.5
    print(f"initial_odom_xy_m: ({initial_x:.3f}, {initial_y:.3f})")
    print(f"final_odom_xy_m: ({final_x:.3f}, {final_y:.3f})")
    print(f"odom_displacement_m: {displacement_m:.3f}")


def average_abs(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(abs(value) for value in values) / len(values)


def print_motion_summary(
    moving_samples: list[tuple[int, float, float]],
    bag_start_t: int,
) -> None:
    print(f"moving_cmd_samples: {len(moving_samples)}")
    if not moving_samples:
        print("first_motion_time_s: n/a")
        print("last_motion_time_s: n/a")
        print("motion_command_span_s: n/a")
        return

    first_motion_t = moving_samples[0][0]
    last_motion_t = moving_samples[-1][0]
    print(f"first_motion_time_s: {ns_to_s(first_motion_t - bag_start_t):.2f}")
    print(f"last_motion_time_s: {ns_to_s(last_motion_t - bag_start_t):.2f}")
    print(f"motion_command_span_s: {ns_to_s(last_motion_t - first_motion_t):.2f}")


def print_goal_summary(goal_samples: list[tuple[int, float, float]]) -> None:
    if not goal_samples:
        print("goal_samples: none")
        return

    _, goal_x, goal_y = goal_samples[-1]
    print(f"last_goal_xy_m: ({goal_x:.3f}, {goal_y:.3f})")


def print_xy_summary(label: str, samples: list[tuple[int, float, float]]) -> None:
    if not samples:
        print(f"{label}_samples: none")
        return

    _, x, y = samples[-1]
    print(f"last_{label}_xy_m: ({x:.3f}, {y:.3f})")


def print_amcl_summary(amcl_pose_samples: list[tuple[int, float, float]]) -> None:
    if not amcl_pose_samples:
        print("initial_amcl_xy_m: n/a")
        print("final_amcl_xy_m: n/a")
        return

    _, initial_x, initial_y = amcl_pose_samples[0]
    _, final_x, final_y = amcl_pose_samples[-1]
    print(f"initial_amcl_xy_m: ({initial_x:.3f}, {initial_y:.3f})")
    print(f"final_amcl_xy_m: ({final_x:.3f}, {final_y:.3f})")


def print_map_base_summary(map_base_samples: list[tuple[int, float, float]]) -> None:
    if not map_base_samples:
        print("final_map_base_xy_m: n/a")
        return

    _, final_x, final_y = map_base_samples[-1]
    print(f"final_map_base_xy_m: ({final_x:.3f}, {final_y:.3f})")


def print_map_summary(map_samples: list[tuple[int, int, int, float]]) -> None:
    print(f"map_samples: {len(map_samples)}")
    if not map_samples:
        print("map_size: n/a")
        return

    _, width, height, resolution = map_samples[-1]
    print(f"map_size: {width}x{height} @ {resolution:.3f} m/px")


def estimate_map_base_samples(
    tf_samples_by_pair: dict[tuple[str, str], list[tuple[int, float, float, float]]],
) -> list[tuple[int, float, float]]:
    map_to_odom = tf_samples_by_pair.get(("map", "odom"), [])
    odom_to_base = tf_samples_by_pair.get(("odom", "base_link"), [])
    if not map_to_odom or not odom_to_base:
        direct = tf_samples_by_pair.get(("map", "base_link"), [])
        return [(timestamp, x, y) for timestamp, x, y, _ in direct]

    map_base_samples: list[tuple[int, float, float]] = []
    odom_index = 0

    for map_timestamp, map_x, map_y, map_yaw in map_to_odom:
        while (
            odom_index + 1 < len(odom_to_base)
            and odom_to_base[odom_index + 1][0] <= map_timestamp
        ):
            odom_index += 1

        odom_timestamp, odom_x, odom_y, _ = odom_to_base[odom_index]
        if odom_timestamp > map_timestamp:
            continue

        base_x, base_y, _ = compose_2d(map_x, map_y, map_yaw, odom_x, odom_y, 0.0)
        map_base_samples.append((map_timestamp, base_x, base_y))

    if map_base_samples:
        return map_base_samples

    latest_map = map_to_odom[-1]
    latest_odom = odom_to_base[-1]
    base_x, base_y, _ = compose_2d(
        latest_map[1],
        latest_map[2],
        latest_map[3],
        latest_odom[1],
        latest_odom[2],
        0.0,
    )
    return [(max(latest_map[0], latest_odom[0]), base_x, base_y)]


def compose_2d(
    parent_x: float,
    parent_y: float,
    parent_yaw: float,
    child_x: float,
    child_y: float,
    child_yaw: float,
) -> tuple[float, float, float]:
    cos_yaw = math.cos(parent_yaw)
    sin_yaw = math.sin(parent_yaw)
    return (
        parent_x + cos_yaw * child_x - sin_yaw * child_y,
        parent_y + sin_yaw * child_x + cos_yaw * child_y,
        parent_yaw + child_yaw,
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_frame_id(frame_id: str) -> str:
    return frame_id.lstrip("/")


def estimate_nav2_goal_error_m(
    odom_samples: list[tuple[int, float, float]],
    goal_samples: list[tuple[int, float, float]],
    *,
    map_base_samples: list[tuple[int, float, float]],
    default_goal_x_m: float,
    default_goal_y_m: float,
) -> float | None:
    pose_samples = map_base_samples if map_base_samples else odom_samples
    if not pose_samples:
        return None

    _, final_x, final_y = pose_samples[-1]
    if goal_samples:
        _, goal_x, goal_y = goal_samples[-1]
    else:
        goal_x = default_goal_x_m
        goal_y = default_goal_y_m

    return ((final_x - goal_x) ** 2 + (final_y - goal_y) ** 2) ** 0.5


def print_nav2_goal_summary(
    nav2_goal_error_m: float | None,
    nav2_odom_success: bool,
    *,
    nav2_goal_tolerance_m: float,
    pose_source: str,
) -> None:
    if nav2_goal_error_m is None:
        print("nav2_goal_error_m: n/a")
        print(f"nav2_goal_pose_source: {pose_source}")
        print("nav2_odom_success: False")
        return

    print(f"nav2_goal_error_m: {nav2_goal_error_m:.3f}")
    print(f"nav2_goal_pose_source: {pose_source}")
    print(f"nav2_goal_tolerance_m: {nav2_goal_tolerance_m:.3f}")
    print(f"nav2_odom_success: {nav2_odom_success}")


def print_progress_summary(progress_samples: list[tuple[int, str]]) -> None:
    if not progress_samples:
        print("waypoint_progress: none recorded")
        return

    unique_progress = []
    for _, progress in progress_samples:
        if not unique_progress or unique_progress[-1] != progress:
            unique_progress.append(progress)

    print("waypoint_progress_transitions:")
    for progress in unique_progress[:20]:
        print(f"  {progress}")
    if len(unique_progress) > 20:
        print(f"  ... {len(unique_progress) - 20} more")


def print_path_summary(label: str, path_samples: list[tuple[int, int]]) -> None:
    if not path_samples:
        print(f"{label}_poses: none")
        return

    pose_counts = [count for _, count in path_samples]
    print(f"{label}_first_poses: {pose_counts[0]}")
    print(f"{label}_last_poses: {pose_counts[-1]}")


def goal_status_name(status: int) -> str:
    names = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    return names.get(status, f"STATUS_{status}")


def ns_to_s(duration_ns: int) -> float:
    return duration_ns * 1e-9


if __name__ == "__main__":
    main()
