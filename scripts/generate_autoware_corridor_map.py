#!/usr/bin/env python3
"""Generate a local-coordinate Lanelet2 corridor map from a SLAM pose graph."""

import argparse
import heapq
import json
import math
import os
import pathlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class GraphPose:
    identifier: int
    x: float
    y: float
    z: float


def load_poses(path: pathlib.Path) -> list[GraphPose]:
    poses = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 9 and fields[0] == "VERTEX_SE3:QUAT":
            poses.append(
                GraphPose(
                    identifier=int(fields[1]),
                    x=float(fields[2]),
                    y=float(fields[3]),
                    z=float(fields[4]),
                )
            )
    if len(poses) < 3:
        raise RuntimeError(f"Not enough poses in {path}")
    return poses


def distance(lhs: GraphPose, rhs: GraphPose) -> float:
    return math.hypot(lhs.x - rhs.x, lhs.y - rhs.y)


def reversal_indices(poses: list[GraphPose]) -> list[int]:
    candidates = []
    for index in range(1, len(poses) - 1):
        previous = poses[index - 1]
        current = poses[index]
        following = poses[index + 1]
        first = (current.x - previous.x, current.y - previous.y)
        second = (following.x - current.x, following.y - current.y)
        first_norm = math.hypot(*first)
        second_norm = math.hypot(*second)
        if first_norm < 0.1 or second_norm < 0.1:
            continue
        cosine = (first[0] * second[0] + first[1] * second[1]) / (
            first_norm * second_norm
        )
        if cosine < -0.20:
            candidates.append((cosine, index))
    candidates.sort()
    selected = []
    for _, index in candidates:
        if all(abs(index - other) > 10 for other in selected):
            selected.append(index)
        if len(selected) == 2:
            break
    return sorted(selected)


def build_spatial_graph(
    poses: list[GraphPose], neighbor_radius: float
) -> list[list[tuple[int, float]]]:
    graph = [[] for _ in poses]
    for left in range(len(poses)):
        for right in range(left + 1, len(poses)):
            separation = distance(poses[left], poses[right])
            sequential = right == left + 1 and separation <= neighbor_radius * 1.8
            spatial = separation <= neighbor_radius
            if sequential or spatial:
                # A small per-edge cost discourages jumping repeatedly between
                # duplicate outbound/inbound passes while still repairing gaps.
                weight = separation + 0.03
                graph[left].append((right, weight))
                graph[right].append((left, weight))
    return graph


def shortest_path(
    graph: list[list[tuple[int, float]]], start: int, goal: int
) -> list[int]:
    costs = [math.inf] * len(graph)
    parents = [-1] * len(graph)
    costs[start] = 0.0
    queue = [(0.0, start)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal:
            break
        for following, weight in graph[current]:
            candidate = cost + weight
            if candidate < costs[following]:
                costs[following] = candidate
                parents[following] = current
                heapq.heappush(queue, (candidate, following))
    if not math.isfinite(costs[goal]):
        raise RuntimeError("SLAM graph endpoints are not connected")
    result = []
    current = goal
    while current >= 0:
        result.append(current)
        if current == start:
            break
        current = parents[current]
    result.reverse()
    return result


def resample(points: list[tuple[float, float]], interval: float) -> list[tuple[float, float]]:
    output = [points[0]]
    carry = 0.0
    previous = points[0]
    for endpoint in points[1:]:
        segment_start = previous
        segment_length = math.hypot(
            endpoint[0] - segment_start[0], endpoint[1] - segment_start[1]
        )
        if segment_length < 1.0e-6:
            previous = endpoint
            continue
        while carry + segment_length >= interval:
            needed = interval - carry
            ratio = needed / segment_length
            sample = (
                segment_start[0] + ratio * (endpoint[0] - segment_start[0]),
                segment_start[1] + ratio * (endpoint[1] - segment_start[1]),
            )
            output.append(sample)
            segment_start = sample
            segment_length = math.hypot(
                endpoint[0] - segment_start[0], endpoint[1] - segment_start[1]
            )
            carry = 0.0
        carry += segment_length
        previous = endpoint
    if math.hypot(output[-1][0] - points[-1][0], output[-1][1] - points[-1][1]) > 0.05:
        output.append(points[-1])
    return output


def smooth(points: list[tuple[float, float]], passes: int = 2) -> list[tuple[float, float]]:
    output = list(points)
    for _ in range(passes):
        updated = [output[0]]
        for index in range(1, len(output) - 1):
            updated.append(
                (
                    0.25 * output[index - 1][0]
                    + 0.50 * output[index][0]
                    + 0.25 * output[index + 1][0],
                    0.25 * output[index - 1][1]
                    + 0.50 * output[index][1]
                    + 0.25 * output[index + 1][1],
                )
            )
        updated.append(output[-1])
        output = updated
    return output


def boundaries(
    centerline: list[tuple[float, float]], half_width: float
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    left = []
    right = []
    for index, point in enumerate(centerline):
        before = centerline[max(0, index - 1)]
        after = centerline[min(len(centerline) - 1, index + 1)]
        dx = after[0] - before[0]
        dy = after[1] - before[1]
        norm = math.hypot(dx, dy)
        if norm < 1.0e-6:
            raise RuntimeError("Degenerate centerline tangent")
        normal = (-dy / norm * half_width, dx / norm * half_width)
        left.append((point[0] + normal[0], point[1] + normal[1]))
        right.append((point[0] - normal[0], point[1] - normal[1]))
    return left, right


def segment_ranges(
    centerline: list[tuple[float, float]], segment_length: float
) -> list[tuple[int, int]]:
    result = []
    start = 0
    accumulated = 0.0
    for index in range(1, len(centerline)):
        accumulated += math.hypot(
            centerline[index][0] - centerline[index - 1][0],
            centerline[index][1] - centerline[index - 1][1],
        )
        if accumulated >= segment_length and index - start >= 2:
            result.append((start, index))
            start = index
            accumulated = 0.0
    if start < len(centerline) - 1:
        result.append((start, len(centerline) - 1))
    return result


def write_lanelet_map(
    path: pathlib.Path,
    centerline: list[tuple[float, float]],
    lane_width: float,
    segment_length: float,
    speed_limit_kph: float,
) -> int:
    left, right = boundaries(centerline, lane_width * 0.5)
    ranges = segment_ranges(centerline, segment_length)
    root = ET.Element("osm", {"version": "0.6", "generator": "ROMO-B"})
    ET.SubElement(root, "MetaInfo", {"format_version": "1", "map_version": "1"})

    left_ids = []
    right_ids = []
    next_id = 1
    for side, identifiers in ((left, left_ids), (right, right_ids)):
        for x, y in side:
            identifier = next_id
            next_id += 1
            identifiers.append(identifier)
            node = ET.SubElement(
                root,
                "node",
                {"id": str(identifier), "lat": "0.0", "lon": "0.0"},
            )
            ET.SubElement(node, "tag", {"k": "local_x", "v": f"{x:.6f}"})
            ET.SubElement(node, "tag", {"k": "local_y", "v": f"{y:.6f}"})
            ET.SubElement(node, "tag", {"k": "ele", "v": "0.0"})

    for start, end in ranges:
        left_way_id = next_id
        next_id += 1
        right_way_id = next_id
        next_id += 1
        for way_id, identifiers in (
            (left_way_id, left_ids),
            (right_way_id, right_ids),
        ):
            way = ET.SubElement(root, "way", {"id": str(way_id)})
            for node_id in identifiers[start : end + 1]:
                ET.SubElement(way, "nd", {"ref": str(node_id)})
            ET.SubElement(way, "tag", {"k": "type", "v": "line_thin"})
            ET.SubElement(way, "tag", {"k": "subtype", "v": "solid"})

        relation = ET.SubElement(root, "relation", {"id": str(next_id)})
        next_id += 1
        ET.SubElement(
            relation, "member", {"type": "way", "role": "left", "ref": str(left_way_id)}
        )
        ET.SubElement(
            relation,
            "member",
            {"type": "way", "role": "right", "ref": str(right_way_id)},
        )
        for key, value in (
            ("type", "lanelet"),
            ("subtype", "road"),
            ("location", "urban"),
            ("one_way", "no"),
            ("participant:vehicle", "yes"),
            ("speed_limit", f"{speed_limit_kph:.3f}"),
        ):
            ET.SubElement(relation, "tag", {"k": key, "v": value})

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return len(ranges)


def replace_symlink(path: pathlib.Path, target: pathlib.Path) -> None:
    if path.is_symlink() or path.exists():
        path.unlink()
    path.symlink_to(os.path.relpath(target, path.parent))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-graph", required=True, type=pathlib.Path)
    parser.add_argument("--pointcloud-map", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--lane-width", type=float, default=2.40)
    parser.add_argument("--sample-interval", type=float, default=0.50)
    parser.add_argument("--segment-length", type=float, default=8.0)
    parser.add_argument("--neighbor-radius", type=float, default=2.20)
    parser.add_argument("--speed-limit-kph", type=float, default=0.72)
    args = parser.parse_args()
    if args.lane_width <= 0.8 or args.sample_interval <= 0.1:
        raise RuntimeError("Lane width or sampling interval is unsafe")
    for required in (args.pose_graph, args.pointcloud_map):
        if not required.exists():
            raise RuntimeError(f"Missing map input: {required}")

    poses = load_poses(args.pose_graph)
    reversals = reversal_indices(poses)
    if len(reversals) != 2:
        raise RuntimeError(f"Expected two corridor U-turns, found {reversals}")
    graph = build_spatial_graph(poses, args.neighbor_radius)
    indices = shortest_path(graph, reversals[1], reversals[0])
    raw_centerline = [(poses[index].x, poses[index].y) for index in indices]
    centerline = smooth(resample(raw_centerline, args.sample_interval))

    args.output.mkdir(parents=True, exist_ok=True)
    lanelet_path = args.output / "lanelet2_map.osm"
    lanelets = write_lanelet_map(
        lanelet_path,
        centerline,
        args.lane_width,
        args.segment_length,
        args.speed_limit_kph,
    )
    (args.output / "map_projector_info.yaml").write_text(
        "projector_type: Local\n", encoding="utf-8"
    )
    replace_symlink(args.output / "pointcloud_map.pcd", args.pointcloud_map.resolve())
    report = {
        # Keep generated metadata portable across clones.  These paths are
        # interpreted relative to generation_report.json's directory.
        "pose_graph": os.path.relpath(args.pose_graph.resolve(), args.output.resolve()),
        "pointcloud_map": os.path.relpath(
            args.pointcloud_map.resolve(), args.output.resolve()
        ),
        "reversal_indices": reversals,
        "source_pose_count": len(poses),
        "centerline_point_count": len(centerline),
        "lanelet_count": lanelets,
        "lane_width_m": args.lane_width,
        "speed_limit_kph": args.speed_limit_kph,
        "start": centerline[0],
        "end": centerline[-1],
    }
    (args.output / "generation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
