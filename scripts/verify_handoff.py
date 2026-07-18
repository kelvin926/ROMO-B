#!/usr/bin/env python3
"""Verify the tracked ROMO-B handoff without contacting any hardware."""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


class Reporter:
    def __init__(self) -> None:
        self.items: list[dict[str, str]] = []

    def add(self, level: str, name: str, message: str) -> None:
        self.items.append({"level": level, "name": name, "message": message})
        print(f"{level:<5} {name:<30} {message}")

    def require(self, condition: bool, name: str, message: str) -> None:
        self.add("PASS" if condition else "FAIL", name, message)

    def warn(self, name: str, message: str) -> None:
        self.add("WARN", name, message)

    @property
    def failures(self) -> int:
        return sum(item["level"] == "FAIL" for item in self.items)

    @property
    def warnings(self) -> int:
        return sum(item["level"] == "WARN" for item in self.items)


def load_yaml(path: pathlib.Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_json(path: pathlib.Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def close_sequence(lhs, rhs, tolerance: float = 1.0e-7) -> bool:
    return len(lhs) == len(rhs) and all(
        math.isclose(float(left), float(right), abs_tol=tolerance)
        for left, right in zip(lhs, rhs)
    )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def pcd_header(path: pathlib.Path) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("PCD DATA header is missing")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            result[fields[0].upper()] = fields[1:]
            if fields[0].upper() == "DATA":
                return result


def pgm_header(path: pathlib.Path) -> tuple[str, int, int, int]:
    tokens: list[str] = []
    with path.open("rb") as stream:
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError("Incomplete PGM header")
            text = line.decode("ascii").split("#", 1)[0]
            tokens.extend(text.split())
    return tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])


def tracked(repo: pathlib.Path, path: pathlib.Path) -> bool:
    try:
        relative = path.resolve().relative_to(repo.resolve())
    except ValueError:
        return False
    process = subprocess.run(
        ["git", "ls-files", "--error-unmatch", str(relative)],
        cwd=repo,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return process.returncode == 0


def verify_manifest_file(
    reporter: Reporter,
    repo: pathlib.Path,
    manifest_path: pathlib.Path,
) -> tuple[dict, pathlib.Path | None]:
    manifest = load_yaml(manifest_path) or {}
    uri = str(manifest.get("uri", "pending"))
    name = manifest_path.stem
    required = ("schema_version", "id", "type", "size_bytes", "sha256", "uri")
    reporter.require(
        all(key in manifest for key in required),
        f"manifest/{name}",
        "required metadata present",
    )
    if uri == "pending":
        reporter.warn(f"manifest/{name}", "external artifact URI is pending")
        return manifest, None
    if not uri.startswith("repo://"):
        reporter.require(True, f"manifest/{name}", f"external URI recorded: {uri}")
        return manifest, None
    artifact = repo / uri.removeprefix("repo://").lstrip("/")
    reporter.require(artifact.is_file(), f"artifact/{name}", f"exists: {artifact.relative_to(repo)}")
    if not artifact.is_file():
        return manifest, artifact
    reporter.require(tracked(repo, artifact), f"artifact/{name}", "artifact is tracked by Git")
    reporter.require(
        artifact.stat().st_size == int(manifest["size_bytes"]),
        f"artifact/{name}",
        f"size={artifact.stat().st_size}",
    )
    actual_hash = sha256(artifact)
    reporter.require(
        actual_hash == str(manifest["sha256"]),
        f"artifact/{name}",
        f"sha256={actual_hash}",
    )
    return manifest, artifact


def verify_hardware(reporter: Reporter, repo: pathlib.Path) -> None:
    hardware = load_yaml(repo / "config/local/hardware.yaml") or {}
    serial = hardware.get("serial", {})
    lidar = hardware.get("lidar", {})
    reporter.require(
        hardware.get("schema_version") == 1
        and serial.get("device") == "/dev/romo_b_pcu"
        and serial.get("baud") == 115200
        and serial.get("data_bits") == 8
        and serial.get("parity") == "none"
        and serial.get("stop_bits") == 1
        and serial.get("command_endian") == "little",
        "hardware/serial",
        "115200 8N1 little-endian PCU configuration",
    )
    reporter.require(
        str(serial.get("adapter_serial", "pending")) != "pending",
        "hardware/adapter",
        str(serial.get("adapter_serial", "pending")),
    )
    if serial.get("wiring") == "pending":
        reporter.warn("hardware/wiring", "straight/null-modem is still a field-verification item")

    try:
        host_ip = ipaddress.ip_address(str(lidar["host_ip"]))
        sensor_ip = ipaddress.ip_address(str(lidar["lidar_ip"]))
        same_subnet = ipaddress.ip_network(f"{host_ip}/24", strict=False) == ipaddress.ip_network(
            f"{sensor_ip}/24", strict=False
        )
    except (KeyError, ValueError):
        same_subnet = False
    reporter.require(
        same_subnet
        and lidar.get("interface") not in (None, "", "pending")
        and lidar.get("frame_id") == "livox_frame",
        "hardware/lidar-network",
        f"host={lidar.get('host_ip')} sensor={lidar.get('lidar_ip')} interface={lidar.get('interface')}",
    )
    reporter.require(
        lidar.get("calibrated") is True,
        "hardware/lidar-transform",
        "measured transform is marked calibrated",
    )
    transform = lidar.get("transform", {})
    expected_extrinsic = quaternion_from_rpy(
        float(transform.get("roll", 0.0)),
        float(transform.get("pitch", 0.0)),
        float(transform.get("yaw", 0.0)),
    ) + [
        float(transform.get("x", 0.0)),
        float(transform.get("y", 0.0)),
        float(transform.get("z", 0.0)),
    ]
    rko = load_yaml(repo / "config/local/rko_lio_mid360.yaml") or {}
    rko_params = rko.get("/**", {}).get("ros__parameters", {})
    reporter.require(
        close_sequence(
            rko_params.get("extrinsic_imu2base_quat_xyzw_xyz", []), expected_extrinsic
        )
        and close_sequence(
            rko_params.get("extrinsic_lidar2base_quat_xyzw_xyz", []), expected_extrinsic
        ),
        "hardware/rko-extrinsics",
        f"quaternion+xyz={expected_extrinsic}",
    )

    livox = load_json(repo / "config/local/MID360_config.json")
    host_info = livox.get("MID360", {}).get("host_net_info", {})
    lidar_info = livox.get("MID360", {}).get("lidar_net_info", {})
    lidar_configs = livox.get("lidar_configs", [])
    host_fields = ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")
    reporter.require(
        all(host_info.get(field) == lidar.get("host_ip") for field in host_fields)
        and len(lidar_configs) == 1
        and lidar_configs[0].get("ip") == lidar.get("lidar_ip")
        and lidar_info.get("point_data_port") == 56300
        and host_info.get("point_data_port") == 56301
        and lidar_info.get("imu_data_port") == 56400
        and host_info.get("imu_data_port") == 56401,
        "hardware/livox-config",
        "derived Livox IP addresses and UDP ports match hardware.yaml",
    )

    waypoints = load_yaml(repo / "config/local/waypoints.yaml") or {}
    points = waypoints.get("waypoints", [])
    speed = float(waypoints.get("default_speed_mps", 0.0))
    reporter.require(
        waypoints.get("schema_version") == 1
        and waypoints.get("frame_id") == "map"
        and waypoints.get("mode") == "continuous"
        and 0.0 < speed <= 0.2
        and len(points) >= 2
        and all("x" in point and "y" in point for point in points),
        "hardware/waypoints",
        f"{len(points)} points, speed={speed:.3f} m/s",
    )


def verify_maps(reporter: Reporter, repo: pathlib.Path) -> None:
    manifest_dir = repo / "data/manifests"
    manifests: dict[str, tuple[dict, pathlib.Path | None]] = {}
    for kind in ("pcd", "nav2", "autoware", "bag", "localization"):
        path = manifest_dir / f"mapping-20260717-195653-{kind}.yaml"
        manifests[kind] = verify_manifest_file(reporter, repo, path)

    map_root = repo / "data/local/maps/mapping-20260717-195653"
    pcd_manifest, pcd = manifests["pcd"]
    if pcd and pcd.is_file():
        header = pcd_header(pcd)
        reporter.require(
            header.get("FIELDS") == ["x", "y", "z", "intensity"]
            and header.get("DATA") == ["binary_compressed"]
            and int(header.get("POINTS", ["-1"])[0])
            == int(pcd_manifest.get("mapping", {}).get("point_count", -2)),
            "map/pcd-header",
            f"fields={' '.join(header.get('FIELDS', []))} points={header.get('POINTS', ['?'])[0]}",
        )

    pose_graph = map_root / "pose_graph.g2o"
    mapping = pcd_manifest.get("mapping", {})
    lines = pose_graph.read_text(encoding="utf-8").splitlines()
    vertices = sum(line.startswith("VERTEX_SE3:QUAT ") for line in lines)
    edges = sum(line.startswith("EDGE_SE3:QUAT ") for line in lines)
    reporter.require(
        sha256(pose_graph) == mapping.get("pose_graph_sha256")
        and vertices == int(mapping.get("submap_count", -1))
        and edges == int(mapping.get("constraint_count", -1)),
        "map/pose-graph",
        f"vertices={vertices} edges={edges}",
    )

    nav_manifest, nav_pgm = manifests["nav2"]
    if nav_pgm and nav_pgm.is_file():
        magic, width, height, max_value = pgm_header(nav_pgm)
        occupancy = nav_manifest.get("occupancy", {})
        nav_yaml_path = nav_pgm.with_name("map.yaml")
        nav_yaml = load_yaml(nav_yaml_path) or {}
        reporter.require(
            magic in ("P2", "P5")
            and max_value == 255
            and width == int(occupancy.get("width_cells", -1))
            and height == int(occupancy.get("height_cells", -1))
            and math.isclose(
                float(nav_yaml.get("resolution", -1.0)),
                float(occupancy.get("resolution_m", -2.0)),
                abs_tol=1.0e-9,
            )
            and close_sequence(nav_yaml.get("origin", [])[:2], occupancy.get("origin_xy_m", []))
            and nav_yaml.get("image") == "map.pgm"
            and nav_yaml_path.stat().st_size == int(occupancy.get("yaml_size_bytes", -1))
            and sha256(nav_yaml_path) == occupancy.get("yaml_sha256"),
            "map/nav2",
            f"{width}x{height} at {nav_yaml.get('resolution')} m/cell",
        )

    autoware_manifest, lanelet_path = manifests["autoware"]
    if lanelet_path and lanelet_path.is_file():
        root = ET.parse(lanelet_path).getroot()
        lanelets = 0
        for relation in root.findall("relation"):
            tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
            lanelets += tags.get("type") == "lanelet"
        generation = autoware_manifest.get("generation", {})
        pointcloud_link = lanelet_path.parent / str(generation.get("pointcloud_link"))
        report_path = lanelet_path.parent / "generation_report.json"
        report = load_json(report_path)
        report_pose_graph = (report_path.parent / report.get("pose_graph", "")).resolve()
        report_pointcloud = (report_path.parent / report.get("pointcloud_map", "")).resolve()
        reporter.require(
            lanelets == int(generation.get("lanelet_count", -1))
            and pointcloud_link.is_symlink()
            and pointcloud_link.resolve() == (map_root / "map.pcd").resolve()
            and report_pose_graph == pose_graph.resolve()
            and report_pointcloud == (map_root / "map.pcd").resolve()
            and not os.path.isabs(str(report.get("pose_graph", "")))
            and not os.path.isabs(str(report.get("pointcloud_map", "")))
            and report.get("lanelet_count") == lanelets
            and report.get("source_pose_count") == vertices,
            "map/autoware",
            f"lanelets={lanelets}, portable generation report and pointcloud link",
        )
        projector_path = lanelet_path.parent / "map_projector_info.yaml"
        projector = load_yaml(projector_path) or {}
        reporter.require(
            projector.get("projector_type") == generation.get("projector") == "Local"
            and sha256(projector_path) == generation.get("projector_info_sha256"),
            "map/projector",
            "Local projector metadata matches manifest",
        )

    bag_manifest, _ = manifests["bag"]
    metadata_path = repo / "data/local/bags/mapping-20260717-195653/metadata.yaml"
    metadata = load_yaml(metadata_path).get("rosbag2_bagfile_information", {})
    topic_counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in metadata.get("topics_with_message_count", [])
    }
    recording = bag_manifest.get("recording", {})
    reporter.require(
        metadata.get("storage_identifier") == recording.get("storage")
        and int(metadata.get("message_count", -1)) == int(recording.get("message_count", -2))
        and math.isclose(
            float(metadata.get("duration", {}).get("nanoseconds", -1)) / 1.0e9,
            float(recording.get("duration_sec", -2)),
            abs_tol=1.0e-9,
        )
        and topic_counts.get("/sensing/lidar/top/pointcloud_raw")
        == int(recording.get("pointcloud_count", -1))
        and topic_counts.get("/sensing/imu/imu_raw")
        == int(recording.get("normalized_imu_count", -1))
        and topic_counts.get("/wheel/odometry_raw")
        == int(recording.get("wheel_odometry_count", -1)),
        "artifact/bag-metadata",
        f"messages={metadata.get('message_count')} duration={recording.get('duration_sec')} s",
    )

    localization_manifest, _ = manifests["localization"]
    summary_path = repo / "data/local/validation/localization-20260717-205119/summary.json"
    summary = load_json(summary_path)
    validation = localization_manifest.get("validation", {})
    reporter.require(
        sha256(summary_path) == validation.get("summary_sha256")
        and summary.get("result") == validation.get("result") == "PASS"
        and all(summary.get("checks", {}).values())
        and all(validation.get("checks", {}).values()),
        "validation/localization-replay",
        f"healthy={summary.get('healthy_ratio')}, closure={summary.get('closure_distance_m'):.3f} m",
    )


def verify_validation_results(reporter: Reporter, repo: pathlib.Path) -> None:
    base = repo / "data/local/validation"
    canonical = {
        "localization-interface": base
        / "autoware-localization-interface-20260718-094138/result.json",
        "safety-gates": base / "autoware-safety-gates-20260718-094140/result.json",
        "perception": base / "autoware-perception-20260718-094144/result.json",
        "waypoint-sender": base
        / "autoware-waypoint-sender-20260718-094147/result.json",
        "obstacle-stop": base / "autoware-planning-20260718-094404/result.json",
        "obstacle-avoidance": base / "rviz-ui-20260718/planning-v3.json",
    }
    for name, path in canonical.items():
        if not path.is_file():
            reporter.require(False, f"validation/{name}", f"missing {path.relative_to(repo)}")
            continue
        result = load_json(path)
        checks = result.get("checks", {})
        reporter.require(
            result.get("result") == "PASS" and bool(checks) and all(checks.values()),
            f"validation/{name}",
            f"PASS with {len(checks)} asserted checks",
        )
    avoidance = load_json(canonical["obstacle-avoidance"])
    reporter.require(
        float(avoidance.get("maximum_clearance_m", -1.0))
        >= float(avoidance.get("required_clearance_m", math.inf))
        and float(avoidance.get("minimum_near_obstacle_speed_mps", -1.0)) > 0.0,
        "validation/avoidance-clearance",
        f"{avoidance.get('maximum_clearance_m'):.3f} m >= {avoidance.get('required_clearance_m'):.3f} m",
    )
    stop = load_json(canonical["obstacle-stop"])
    reporter.require(
        math.isclose(float(stop.get("minimum_near_obstacle_speed_mps", -1.0)), 0.0)
        and float(stop.get("baseline_speed_range_mps", [math.inf, math.inf])[1]) <= 0.200001,
        "validation/blocked-corridor-stop",
        "planned speed reaches 0.0 m/s and baseline stays <= 0.2 m/s",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=pathlib.Path,
        default=pathlib.Path(__file__).resolve().parent.parent,
    )
    parser.add_argument("--json-output", type=pathlib.Path)
    args = parser.parse_args()
    repo = args.repo_root.resolve()
    reporter = Reporter()
    try:
        verify_hardware(reporter, repo)
        verify_maps(reporter, repo)
        verify_validation_results(reporter, repo)
    except Exception as error:  # Keep CI output actionable instead of a traceback-only failure.
        reporter.add("FAIL", "internal", f"{type(error).__name__}: {error}")

    outcome = "FAIL" if reporter.failures else (
        "PASS_WITH_WARNINGS" if reporter.warnings else "PASS"
    )
    report = {
        "result": outcome,
        "failures": reporter.failures,
        "warnings": reporter.warnings,
        "checks": reporter.items,
    }
    print(
        f"RESULT: {outcome} ({len(reporter.items)} checks, "
        f"{reporter.failures} failures, {reporter.warnings} warnings)"
    )
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 1 if reporter.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
