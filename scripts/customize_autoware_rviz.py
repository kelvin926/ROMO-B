#!/usr/bin/env python3
"""Generate the ROMO-B field RViz layout from pinned Autoware 1.8.0."""

from __future__ import annotations

import argparse
import copy
import pathlib
import subprocess
import sys

import yaml


RELATIVE_CONFIG = pathlib.Path("autoware_launch/rviz/autoware.rviz")


def _walk(displays):
    for display in displays or []:
        yield display
        yield from _walk(display.get("Displays"))


def _find(displays, name):
    return next(display for display in _walk(displays) if display.get("Name") == name)


def _remove_non_field_displays(displays):
    removed_classes = {
        "rviz_default_plugins/Image",
        "rviz_default_plugins/Camera",
        "rviz_plugins/PolarGridDisplay",
    }
    kept = []
    for display in displays or []:
        if display.get("Class") in removed_classes:
            continue
        if "Displays" in display:
            display["Displays"] = _remove_non_field_displays(display["Displays"])
        kept.append(display)
    return kept


def customize(config):
    """Return a LiDAR-first, low-speed ROMO-B operator layout."""
    config = copy.deepcopy(config)
    config["Panels"] = [
        panel
        for panel in config.get("Panels", [])
        if panel.get("Name") in {"Displays", "AutowareStatePanel"}
    ]
    displays_panel = next(
        panel for panel in config["Panels"] if panel.get("Name") == "Displays"
    )
    displays_panel["Tree Height"] = 1100
    displays_panel["Property Tree Widget"]["Expanded"] = ["/ROMO-B Field1"]

    manager = config["Visualization Manager"]
    manager["Global Options"]["Background Color"] = "8; 13; 17"
    displays = _remove_non_field_displays(manager["Displays"])
    manager["Displays"] = displays

    pointcloud_map = _find(displays, "PointCloudMap")
    pointcloud_map.update(
        {
            "Alpha": 0.5,
            "Color": "196; 205; 214",
            "Color Transformer": "FlatColor",
            "Size (Pixels)": 2,
        }
    )

    lidar_group = _find(displays, "LiDAR")
    concatenated = _find(lidar_group["Displays"], "ConcatenatePointCloud")
    concatenated.update(
        {
            "Alpha": 0.72,
            "Name": "Simulation / Autoware LiDAR",
            "Size (Pixels)": 2,
        }
    )
    filtered = copy.deepcopy(concatenated)
    filtered.update(
        {
            "Alpha": 0.9,
            "Name": "ROMO-B Filtered LiDAR",
            "Size (Pixels)": 3,
        }
    )
    filtered["Topic"]["Value"] = "/sensing/lidar/top/pointcloud_filtered"
    lidar_group["Displays"].insert(0, filtered)

    predicted = _find(displays, "PredictedObjects")
    predicted["Display UUID"] = False
    predicted["UNKNOWN"] = {"Alpha": 0.999, "Color": "255; 92; 64"}
    predicted["Line Width"] = 0.06

    trajectory = _find(displays, "ScenarioTrajectory")
    trajectory["View Path"].update(
        {
            "Alpha": 0.999,
            "Color": "0; 255; 170",
            "Constant Color": True,
            "Width": 4,
        }
    )
    trajectory["View Velocity"]["Value"] = False

    lanelet_map = _find(displays, "Lanelet2VectorMap")
    route = _find(displays, "RouteArea")
    goal = _find(displays, "GoalPose")
    field_items = []
    for source, name in (
        (pointcloud_map, "PCD Map"),
        (lanelet_map, "Lanelet2 Road"),
        (filtered, "Live Mid-360 (filtered)"),
        (concatenated, "Simulation / Autoware LiDAR"),
        (predicted, "Recognized Obstacles"),
        (route, "Active Route"),
        (goal, "Final Goal"),
        (trajectory, "Planned Trajectory"),
    ):
        item = copy.deepcopy(source)
        item["Name"] = name
        item["Enabled"] = True
        item["Value"] = True
        field_items.append(item)

    for group in displays:
        if group.get("Name") in {"Map", "Sensing", "Perception", "Planning", "Control"}:
            group["Enabled"] = False
            group["Value"] = False
    displays.insert(
        1,
        {
            "Class": "rviz_common/Group",
            "Displays": field_items,
            "Enabled": True,
            "Name": "ROMO-B Field",
            "Value": True,
        },
    )

    current_view = manager["Views"]["Current"]
    current_view.update(
        {
            "Scale": 14,
            "Target Frame": "base_link",
            "X": 4,
            "Y": 0,
        }
    )

    geometry = config.setdefault("Window Geometry", {})
    geometry.pop("RecognitionResultOnImage", None)
    geometry.pop("Image", None)
    geometry.pop("AutowareDateTimePanel", None)
    geometry.pop("ControlModeDisplay", None)
    geometry.pop("Selection", None)
    geometry.pop("Tool Properties", None)
    geometry.pop("Views", None)
    geometry.update({"Height": 1536, "Width": 2490, "X": 0, "Y": 37})
    return config


def render(config):
    return yaml.safe_dump(config, sort_keys=False, width=8192)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--autoware-root", type=pathlib.Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    checkout = args.autoware_root / "src/launcher/autoware_launch"
    target = checkout / RELATIVE_CONFIG
    original = subprocess.check_output(
        ["git", "-C", str(checkout), "show", f"HEAD:{RELATIVE_CONFIG}"],
        text=True,
    )
    generated = render(customize(yaml.safe_load(original)))

    if args.check:
        if not target.exists() or target.read_text(encoding="utf-8") != generated:
            print("ROMO-B Autoware RViz layout is missing or stale", file=sys.stderr)
            return 1
        return 0

    target.write_text(generated, encoding="utf-8")
    print(f"Generated LiDAR-first ROMO-B RViz layout: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
