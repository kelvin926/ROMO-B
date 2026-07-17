#!/usr/bin/env python3
import argparse
import json
import math
import pathlib

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check a forward-only Nav2 plan between pose-graph vertices."
    )
    parser.add_argument("pose_graph", type=pathlib.Path)
    parser.add_argument("start_vertex", type=int)
    parser.add_argument("goal_vertex", type=int)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def load_vertices(path):
    vertices = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 9 and fields[0] == "VERTEX_SE3:QUAT":
            vertices[int(fields[1])] = tuple(float(value) for value in fields[2:9])
    return vertices


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def set_pose(stamped, values, stamp):
    stamped.header.frame_id = "map"
    stamped.header.stamp = stamp
    position = stamped.pose.position
    orientation = stamped.pose.orientation
    position.x, position.y, position.z = values[:3]
    orientation.x, orientation.y, orientation.z, orientation.w = values[3:]


def main():
    args = parse_args()
    vertices = load_vertices(args.pose_graph)
    for vertex in (args.start_vertex, args.goal_vertex):
        if vertex not in vertices:
            raise RuntimeError(f"Pose graph does not contain vertex {vertex}")

    rclpy.init()
    node = Node("romo_b_nav2_plan_check")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("ComputePathToPose action server is unavailable")
        goal = ComputePathToPose.Goal()
        stamp = node.get_clock().now().to_msg()
        set_pose(goal.start, vertices[args.start_vertex], stamp)
        set_pose(goal.goal, vertices[args.goal_vertex], stamp)
        goal.planner_id = "GridBased"
        goal.use_start = True
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("ComputePathToPose goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("ComputePathToPose timed out")

        poses = wrapped.result.path.poses
        path_length = 0.0
        reverse_segments = 0
        minimum_forward_projection = math.inf
        for previous, current in zip(poses, poses[1:]):
            dx = current.pose.position.x - previous.pose.position.x
            dy = current.pose.position.y - previous.pose.position.y
            distance = math.hypot(dx, dy)
            path_length += distance
            quaternion = previous.pose.orientation
            yaw = yaw_from_quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w
            )
            projection = dx * math.cos(yaw) + dy * math.sin(yaw)
            minimum_forward_projection = min(minimum_forward_projection, projection)
            reverse_segments += int(projection < -1e-4)

        summary = {
            "start_vertex": args.start_vertex,
            "goal_vertex": args.goal_vertex,
            "result_code": int(wrapped.status),
            "pose_count": len(poses),
            "path_length_m": path_length,
            "reverse_segment_count": reverse_segments,
            "minimum_forward_projection_m": (
                minimum_forward_projection if len(poses) >= 2 else None
            ),
            "planning_time_sec": (
                wrapped.result.planning_time.sec
                + wrapped.result.planning_time.nanosec * 1e-9
            ),
        }
        summary["result"] = (
            "PASS"
            if wrapped.status == 4 and len(poses) >= 2 and reverse_segments == 0
            else "FAIL"
        )
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary["result"] == "PASS" else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
