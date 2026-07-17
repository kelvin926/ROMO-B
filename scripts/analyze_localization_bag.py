#!/usr/bin/env python3
import argparse
import json
import math
import pathlib
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Evaluate ROMO-B localization diagnostics recorded during replay."
    )
    parser.add_argument("bag", type=pathlib.Path)
    parser.add_argument("--output-json", type=pathlib.Path)
    parser.add_argument("--min-status", type=int, default=1000)
    parser.add_argument("--min-healthy-ratio", type=float, default=0.95)
    parser.add_argument("--max-p95-fitness", type=float, default=1.5)
    parser.add_argument("--max-consecutive-rejects", type=int, default=10)
    parser.add_argument("--min-trajectory-length", type=float, default=150.0)
    parser.add_argument("--max-closure-distance", type=float, default=2.0)
    parser.add_argument("--min-duration", type=float, default=300.0)
    return parser.parse_args()


def percentile(values, percentile_value):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile_value / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def main():
    args = parse_arguments()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag.resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required_topics = {
        "/clock",
        "/localization/pose_with_covariance",
        "/localization/alignment_status",
        "/localization/reinitialization_requested",
    }
    missing = required_topics - topic_types.keys()
    if missing:
        raise RuntimeError(f"Replay bag is missing topics: {sorted(missing)}")
    message_types = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }

    poses = []
    fitness_scores = []
    healthy_count = 0
    status_count = 0
    max_rejects = 0
    reinitialization_true_count = 0
    first_clock = None
    last_clock = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in required_topics:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == "/clock":
            clock_time = message.clock.sec + message.clock.nanosec * 1e-9
            first_clock = clock_time if first_clock is None else first_clock
            last_clock = clock_time
        elif topic == "/localization/pose_with_covariance":
            position = message.pose.pose.position
            if all(math.isfinite(value) for value in (position.x, position.y, position.z)):
                poses.append((position.x, position.y, position.z))
        elif topic == "/localization/reinitialization_requested":
            reinitialization_true_count += int(message.data)
        else:
            for status in message.status:
                if status.name != "lidar_localization_ros2/alignment":
                    continue
                values = {entry.key: entry.value for entry in status.values}
                status_count += 1
                healthy_count += int(values.get("failure_category") == "healthy")
                try:
                    fitness_scores.append(float(values["fitness_score"]))
                    max_rejects = max(
                        max_rejects,
                        int(values.get("consecutive_rejected_updates", "0")),
                    )
                except (KeyError, ValueError):
                    pass

    trajectory_length = sum(
        math.dist(previous, current) for previous, current in zip(poses, poses[1:])
    )
    closure_distance = math.dist(poses[0], poses[-1]) if len(poses) >= 2 else None
    replay_duration = (
        last_clock - first_clock
        if first_clock is not None and last_clock is not None
        else None
    )
    healthy_ratio = healthy_count / status_count if status_count else 0.0
    summary = {
        "replay_duration_sec": replay_duration,
        "status_count": status_count,
        "pose_count": len(poses),
        "healthy_count": healthy_count,
        "healthy_ratio": healthy_ratio,
        "fitness_median": statistics.median(fitness_scores) if fitness_scores else None,
        "fitness_p95": percentile(fitness_scores, 95.0) if fitness_scores else None,
        "fitness_max": max(fitness_scores) if fitness_scores else None,
        "max_consecutive_rejected_updates": max_rejects,
        "reinitialization_true_count": reinitialization_true_count,
        "trajectory_length_m": trajectory_length,
        "closure_distance_m": closure_distance,
        "initial_position_xyz_m": poses[0] if poses else None,
        "final_position_xyz_m": poses[-1] if poses else None,
    }
    checks = {
        "status_count": status_count >= args.min_status,
        "duration": replay_duration is not None
        and replay_duration >= args.min_duration,
        "healthy_ratio": healthy_ratio >= args.min_healthy_ratio,
        "fitness_p95": bool(fitness_scores)
        and summary["fitness_p95"] <= args.max_p95_fitness,
        "consecutive_rejects": max_rejects <= args.max_consecutive_rejects,
        "reinitialization": reinitialization_true_count == 0,
        "trajectory_length": trajectory_length >= args.min_trajectory_length,
        "closure_distance": closure_distance is not None
        and closure_distance <= args.max_closure_distance,
    }
    summary["checks"] = checks
    summary["result"] = "PASS" if all(checks.values()) else "FAIL"
    rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding="utf-8")
    sys.stdout.write(rendered)
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
