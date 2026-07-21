"""Allow-listed process runner for the ROMO-B browser operator console.

The web UI never accepts an arbitrary command or path.  Every operation below
maps to a repository-owned script, and map/bag selections are constrained to
the ignored ``data/local`` artifact roots.
"""

from __future__ import annotations

import copy
import os
import pathlib
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class OperationSpec:
    label: str
    group: str
    description: str
    selection: str = "none"
    primary: bool = False
    discover_markers: tuple[str, ...] = ()
    caution: str = ""


OPERATION_SPECS = {
    "live_mapping": OperationSpec(
        "Live mapping",
        "Mapping",
        "Start Mid-360 + IMU + wheel odometry mapping, rosbag recording, and RViz.",
        primary=True,
        discover_markers=("mapping_live.launch.py",),
        caution="RC Manual only; drive slowly after LIO odometry appears.",
    ),
    "save_live_mapping": OperationSpec(
        "Save current map",
        "Mapping",
        "Optimize the active graph, save PCD/pose graph, and generate the Nav2 map.",
        selection="map",
    ),
    "record_mapping_bag": OperationSpec(
        "Record mapping bag",
        "Mapping",
        "Record LiDAR, IMU, wheel odometry, TF, platform, and diagnostics topics.",
        discover_markers=("ros2 bag record",),
    ),
    "field_navigation": OperationSpec(
        "Nav2 field navigation",
        "Navigation",
        "Start the complete ROMO-B LiDAR localization, Nav2, obstacle avoidance, and RViz stack.",
        selection="map",
        primary=True,
        discover_markers=("field_navigation.launch.py",),
        caution="The bridge starts disarmed; arm explicitly from Main after checking PCU feedback.",
    ),
    "autoware_field": OperationSpec(
        "Autoware field",
        "Navigation",
        "Start the Autoware Universe field stack with the selected map.",
        selection="map",
        primary=True,
        discover_markers=("romo_b_autoware", "field.launch.py"),
        caution="Receive-only is the default and can be changed in the operation controls.",
    ),
    "autoware_planning_sim": OperationSpec(
        "Autoware planning simulation",
        "Navigation",
        "Start Autoware planning simulation and RViz without robot command hardware.",
        selection="map",
        primary=True,
        discover_markers=("planning_sim.launch.py",),
    ),
    "localization_replay": OperationSpec(
        "Localization replay",
        "Navigation",
        "Replay a selected bag against a selected PCD map in an isolated ROS domain.",
        selection="map_bag",
    ),
    "doctor_preflight": OperationSpec(
        "Host preflight",
        "Validation",
        "Check Ubuntu, ROS 2, tools, serial, network, and local hardware configuration.",
    ),
    "doctor_hardware": OperationSpec(
        "Hardware doctor",
        "Validation",
        "Run strict USB-RS232, Ethernet, LiDAR, and transform checks.",
    ),
    "doctor_autoware": OperationSpec(
        "Autoware doctor",
        "Validation",
        "Verify pinned Autoware, build markers, presets, dependencies, and map bundle.",
        selection="map",
    ),
    "hardware_discovery": OperationSpec(
        "Discover hardware",
        "Validation",
        "Read-only discovery of USB-RS232 and USB-Ethernet devices; transmits no serial data.",
    ),
    "serial_receive_only": OperationSpec(
        "PCU receive-only probe",
        "Validation",
        "Decode PCU feedback for five seconds without ever calling serial.write().",
        caution="Stop any stack already using /dev/romo_b_pcu first.",
    ),
    "mapping_calibration": OperationSpec(
        "Mapping calibration probe",
        "Validation",
        "Compare live LiDAR, IMU, wheel odometry, timestamps, extrinsics, and yaw motion.",
    ),
    "yaw_sign": OperationSpec(
        "Yaw-sign check",
        "Validation",
        "Compare wheel-odometry and Mid-360 IMU yaw direction during a short turn.",
    ),
    "command_pipeline": OperationSpec(
        "Command pipeline check",
        "Validation",
        "Inject synthetic commands/obstacles and verify mux, smoother, and collision monitor output.",
    ),
    "simulation_smoke": OperationSpec(
        "Simulation smoke test",
        "Validation",
        "Run the isolated ROMO-B PTY/simulation smoke test.",
    ),
    "nav2_preflight": OperationSpec(
        "Nav2 map preflight",
        "Validation",
        "Validate lifecycle, planning, command pipeline, and rotation shim on the selected map.",
        selection="map",
    ),
    "handoff": OperationSpec(
        "Repository handoff check",
        "Validation",
        "Verify repository layout, documentation, locks, scripts, and reproducibility markers.",
    ),
    "autoware_validation": OperationSpec(
        "Autoware validation suite",
        "Validation",
        "Run all isolated Autoware interface, perception, planning, safety, and waypoint checks.",
        selection="map",
    ),
    "autoware_localization_check": OperationSpec(
        "Autoware localization check",
        "Validation",
        "Run the isolated Autoware localization-interface validation.",
        selection="map",
    ),
    "autoware_perception_check": OperationSpec(
        "Autoware perception check",
        "Validation",
        "Run the isolated pointcloud-to-object perception validation.",
    ),
    "autoware_planning_check": OperationSpec(
        "Autoware planning check",
        "Validation",
        "Run the isolated Autoware planning validation.",
        selection="map",
    ),
    "autoware_safety_check": OperationSpec(
        "Autoware safety-gate check",
        "Validation",
        "Validate command gates and operation-mode behavior in an isolated domain.",
    ),
    "autoware_waypoint_check": OperationSpec(
        "Autoware waypoint check",
        "Validation",
        "Validate the waypoint-to-route sender in an isolated domain.",
    ),
    "build_project": OperationSpec(
        "Build ROMO-B workspace",
        "Build & data",
        "Build only repository ROS 2 packages with tests enabled.",
    ),
    "build_all": OperationSpec(
        "Build all dependencies",
        "Build & data",
        "Fetch and build Livox/vendor overlays plus the ROMO-B workspace.",
        caution="This can take a long time and uses substantial CPU and disk.",
    ),
    "fetch_dependencies": OperationSpec(
        "Fetch pinned dependencies",
        "Build & data",
        "Import or update repository-pinned vendor sources.",
    ),
    "prepare_autoware_map": OperationSpec(
        "Prepare Autoware map",
        "Build & data",
        "Generate the corridor Lanelet2/PCD/projector bundle from the selected map.",
        selection="map",
    ),
    "tracked_sizes": OperationSpec(
        "Check tracked file sizes",
        "Build & data",
        "Confirm no oversized generated artifact is about to be committed.",
    ),
}


class OperationManager:
    """Run and observe only the operations declared in ``OPERATION_SPECS``."""

    def __init__(self, repo_root: pathlib.Path):
        self.repo_root = pathlib.Path(repo_root).resolve()
        self.map_root = self.repo_root / "data" / "local" / "maps"
        self.bag_root = self.repo_root / "data" / "local" / "bags"
        self.log_root = self.repo_root / "data" / "local" / "logs"
        self._lock = threading.RLock()
        self._owned: dict[str, dict] = {}
        self._history: dict[str, dict] = {}
        self._discovery_deadline = 0.0
        self._discovered: dict[str, list[int]] = {}
        self._artifact_deadline = 0.0
        self._artifact_cache = {"maps": [], "bags": []}

    @staticmethod
    def _bool(value, default: bool = True) -> bool:
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _script(self, name: str) -> str:
        path = (self.repo_root / "scripts" / name).resolve()
        if path.parent != (self.repo_root / "scripts").resolve() or not path.is_file():
            raise ValueError(f"Repository script is missing: {name}")
        return str(path)

    @staticmethod
    def _contained_directory(root: pathlib.Path, artifact_id: str) -> pathlib.Path:
        if not artifact_id or pathlib.Path(artifact_id).name != artifact_id:
            raise ValueError("Select a valid local artifact")
        candidate = (root / artifact_id).resolve()
        if candidate.parent != root.resolve() or not candidate.is_dir():
            raise ValueError(f"Local artifact does not exist: {artifact_id}")
        return candidate

    def _map(self, payload: dict, required: bool = True) -> pathlib.Path | None:
        artifact_id = str(payload.get("map_id", ""))
        if artifact_id:
            return self._contained_directory(self.map_root, artifact_id)
        maps = self.artifacts()["maps"]
        if maps:
            return pathlib.Path(maps[0]["path"])
        if required:
            raise ValueError("No local map run is available")
        return None

    def _bag(self, payload: dict) -> pathlib.Path:
        artifact_id = str(payload.get("bag_id", ""))
        if artifact_id:
            return self._contained_directory(self.bag_root, artifact_id)
        bags = self.artifacts()["bags"]
        if not bags:
            raise ValueError("No local rosbag is available")
        return pathlib.Path(bags[0]["path"])

    @staticmethod
    def _stamp() -> str:
        return time.strftime("%Y%m%d-%H%M%S")

    def _command(self, operation_id: str, payload: dict) -> tuple[list[str], dict]:
        env = os.environ.copy()
        use_rviz = "true" if self._bool(payload.get("use_rviz"), True) else "false"

        if operation_id == "live_mapping":
            run_name = f"mapping-{self._stamp()}"
            map_dir = self.map_root / run_name
            bag_dir = self.bag_root / run_name
            env["USE_RVIZ"] = use_rviz
            return [self._script("run_live_mapping.sh"), str(map_dir), str(bag_dir)], env
        if operation_id == "save_live_mapping":
            return [self._script("save_live_mapping.sh"), str(self._map(payload))], env
        if operation_id == "record_mapping_bag":
            output = self.bag_root / f"mapping-{self._stamp()}"
            return [self._script("record_mapping_bag.sh"), str(output)], env
        if operation_id == "field_navigation":
            env.update(
                {
                    "MAP_RUN": str(self._map(payload)),
                    "USE_OPERATOR_UI": "false",
                    "OPEN_OPERATOR_UI_BROWSER": "false",
                    "USE_RVIZ": use_rviz,
                    "MAX_SPEED_MPS": str(payload.get("max_speed_mps", 0.5)),
                }
            )
            return [self._script("run_field_navigation.sh")], env
        if operation_id == "autoware_field":
            env.update(
                {
                    "MAP_RUN": str(self._map(payload)),
                    "RECEIVE_ONLY": "true"
                    if self._bool(payload.get("receive_only"), True)
                    else "false",
                    "USE_RVIZ": use_rviz,
                }
            )
            return [self._script("run_autoware_field.sh")], env
        if operation_id == "autoware_planning_sim":
            env.update({"MAP_RUN": str(self._map(payload)), "USE_RVIZ": use_rviz})
            return [self._script("run_autoware_planning_sim.sh")], env
        if operation_id == "localization_replay":
            rate = float(payload.get("rate", 1.0))
            if not 0.1 <= rate <= 4.0:
                raise ValueError("Replay rate must be from 0.1 to 4.0")
            return [
                self._script("run_localization_replay.sh"),
                str(self._bag(payload)),
                str(self._map(payload)),
                str(rate),
            ], env
        if operation_id.startswith("doctor_"):
            mode = "--" + operation_id.removeprefix("doctor_")
            if operation_id == "doctor_autoware":
                env["AUTOWARE_MAP"] = str(self._map(payload) / "autoware")
            return [self._script("doctor.sh"), mode], env
        if operation_id == "hardware_discovery":
            return [self._script("onboard_hardware.sh"), "--discover"], env
        if operation_id == "serial_receive_only":
            return [
                "python3",
                self._script("serial_receive_only.py"),
                "--duration",
                "5",
                "--min-frames",
                "50",
            ], env
        if operation_id == "mapping_calibration":
            return ["python3", self._script("check_mapping_calibration.py")], env
        if operation_id == "yaw_sign":
            return ["python3", self._script("check_yaw_sign.py")], env
        if operation_id == "command_pipeline":
            return ["python3", self._script("check_cmd_pipeline.py")], env
        if operation_id == "simulation_smoke":
            return [self._script("run_simulation_smoke.sh")], env
        if operation_id == "nav2_preflight":
            map_run = self._map(payload)
            return [
                self._script("run_nav2_preflight.sh"),
                str(map_run / "nav2-raycast" / "map.yaml"),
                str(map_run / "pose_graph.g2o"),
            ], env
        if operation_id == "handoff":
            return ["python3", self._script("verify_handoff.py")], env
        if operation_id == "autoware_validation":
            env["MAP_RUN"] = str(self._map(payload))
            return [self._script("run_autoware_validation.sh")], env
        autoware_checks = {
            "autoware_localization_check": "run_autoware_localization_interface_check.sh",
            "autoware_perception_check": "run_autoware_perception_check.sh",
            "autoware_planning_check": "run_autoware_planning_check.sh",
            "autoware_safety_check": "run_autoware_safety_gate_check.sh",
            "autoware_waypoint_check": "run_autoware_waypoint_sender_check.sh",
        }
        if operation_id in autoware_checks:
            if OPERATION_SPECS[operation_id].selection == "map":
                env["MAP_RUN"] = str(self._map(payload))
            return [self._script(autoware_checks[operation_id])], env
        if operation_id == "build_project":
            return [self._script("build_all.sh"), "--project-only"], env
        if operation_id == "build_all":
            return [self._script("build_all.sh")], env
        if operation_id == "fetch_dependencies":
            return [self._script("fetch_dependencies.sh")], env
        if operation_id == "prepare_autoware_map":
            return [self._script("prepare_autoware_map.sh"), str(self._map(payload))], env
        if operation_id == "tracked_sizes":
            return [self._script("check_tracked_file_sizes.sh")], env
        raise ValueError(f"Unknown operation: {operation_id}")

    def _scan_processes(self) -> dict[str, list[int]]:
        now = time.monotonic()
        with self._lock:
            if now < self._discovery_deadline:
                return copy.deepcopy(self._discovered)
        command_lines = []
        try:
            entries: Iterable[pathlib.Path] = pathlib.Path("/proc").iterdir()
        except OSError:
            entries = ()
        for proc in entries:
            if not proc.name.isdigit():
                continue
            try:
                command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    errors="replace"
                )
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            command_lines.append((int(proc.name), command))
        found = {}
        for operation_id, spec in OPERATION_SPECS.items():
            if not spec.discover_markers:
                found[operation_id] = []
                continue
            found[operation_id] = sorted(
                pid
                for pid, command in command_lines
                if all(marker in command for marker in spec.discover_markers)
            )
        with self._lock:
            self._discovered = found
            self._discovery_deadline = now + 0.75
        return copy.deepcopy(found)

    def _poll_owned(self) -> None:
        with self._lock:
            for operation_id, record in list(self._owned.items()):
                process = record["process"]
                return_code = process.poll()
                if return_code is None:
                    continue
                record["handle"].close()
                self._history[operation_id] = {
                    "pid": process.pid,
                    "started_at": record["started_at"],
                    "finished_at": time.time(),
                    "exit_code": return_code,
                    "log_path": record["log_path"],
                    "message": "Completed" if return_code == 0 else f"Exited with code {return_code}",
                }
                del self._owned[operation_id]
                self._discovery_deadline = 0.0

    def start(self, operation_id: str, payload: dict | None = None) -> dict:
        if operation_id not in OPERATION_SPECS:
            raise ValueError(f"Unknown operation: {operation_id}")
        payload = payload or {}
        self._poll_owned()
        discovered = self._scan_processes()
        if discovered.get(operation_id):
            return {
                "accepted": False,
                "message": f"{OPERATION_SPECS[operation_id].label} is already running (PID {discovered[operation_id][0]})",
            }
        if OPERATION_SPECS[operation_id].primary:
            conflict = next(
                (
                    other
                    for other, spec in OPERATION_SPECS.items()
                    if other != operation_id and spec.primary and discovered.get(other)
                ),
                None,
            )
            if conflict:
                return {
                    "accepted": False,
                    "message": f"Stop {OPERATION_SPECS[conflict].label} before starting this stack",
                }
        with self._lock:
            if operation_id in self._owned:
                return {
                    "accepted": False,
                    "message": f"{OPERATION_SPECS[operation_id].label} is already running",
                }
        command, env = self._command(operation_id, payload)
        self.log_root.mkdir(parents=True, exist_ok=True)
        log_path = self.log_root / f"operator-{operation_id}-{self._stamp()}.log"
        handle = log_path.open("ab", buffering=0)
        try:
            process = subprocess.Popen(
                command,
                cwd=str(self.repo_root),
                env=env,
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            handle.close()
            raise
        with self._lock:
            self._owned[operation_id] = {
                "process": process,
                "handle": handle,
                "started_at": time.time(),
                "log_path": str(log_path),
            }
            self._history[operation_id] = {
                "pid": process.pid,
                "started_at": time.time(),
                "finished_at": None,
                "exit_code": None,
                "log_path": str(log_path),
                "message": "Start requested",
            }
            self._discovery_deadline = 0.0
            self._artifact_deadline = 0.0
        return {
            "accepted": True,
            "message": f"{OPERATION_SPECS[operation_id].label} start requested (PID {process.pid})",
            "pid": process.pid,
            "log_path": str(log_path),
        }

    def stop(self, operation_id: str) -> dict:
        if operation_id not in OPERATION_SPECS:
            raise ValueError(f"Unknown operation: {operation_id}")
        self._poll_owned()
        stopped = []
        with self._lock:
            owned = self._owned.get(operation_id)
        if owned and owned["process"].poll() is None:
            try:
                os.killpg(owned["process"].pid, signal.SIGINT)
                stopped.append(owned["process"].pid)
            except (ProcessLookupError, PermissionError):
                pass
        else:
            for pid in self._scan_processes().get(operation_id, []):
                try:
                    os.kill(pid, signal.SIGINT)
                    stopped.append(pid)
                except (ProcessLookupError, PermissionError):
                    continue
        with self._lock:
            self._discovery_deadline = 0.0
        if not stopped:
            return {
                "accepted": True,
                "message": f"{OPERATION_SPECS[operation_id].label} is already stopped",
            }
        return {
            "accepted": True,
            "message": f"Stop requested for {OPERATION_SPECS[operation_id].label} PID(s): "
            + ", ".join(str(pid) for pid in stopped),
        }

    @staticmethod
    def _artifact_rows(root: pathlib.Path, kind: str) -> list[dict]:
        rows = []
        if not root.is_dir():
            return rows
        for path in root.iterdir():
            if not path.is_dir():
                continue
            try:
                stat = path.stat()
            except OSError:
                continue
            if kind == "map":
                row = {
                    "id": path.name,
                    "path": str(path),
                    "modified_at": stat.st_mtime,
                    "has_pcd": (path / "map.pcd").is_file(),
                    "has_pose_graph": (path / "pose_graph.g2o").is_file(),
                    "has_nav2": (path / "nav2-raycast" / "map.yaml").is_file(),
                    "has_autoware": (path / "autoware" / "lanelet2_map.osm").is_file(),
                }
                row["ready_nav2"] = row["has_pcd"] and row["has_nav2"]
            else:
                row = {
                    "id": path.name,
                    "path": str(path),
                    "modified_at": stat.st_mtime,
                    "has_metadata": (path / "metadata.yaml").is_file(),
                    "db3_count": len(list(path.glob("*.db3"))),
                }
            rows.append(row)
        return sorted(rows, key=lambda row: row["modified_at"], reverse=True)

    def artifacts(self) -> dict:
        now = time.monotonic()
        with self._lock:
            if now < self._artifact_deadline:
                return copy.deepcopy(self._artifact_cache)
        result = {
            "maps": self._artifact_rows(self.map_root, "map"),
            "bags": self._artifact_rows(self.bag_root, "bag"),
        }
        with self._lock:
            self._artifact_cache = result
            self._artifact_deadline = now + 1.5
        return copy.deepcopy(result)

    def snapshot(self) -> dict:
        self._poll_owned()
        discovered = self._scan_processes()
        now = time.time()
        tasks = []
        with self._lock:
            owned_records = dict(self._owned)
            history = copy.deepcopy(self._history)
        for operation_id, spec in OPERATION_SPECS.items():
            owned = owned_records.get(operation_id)
            item_history = history.get(operation_id, {})
            pids = discovered.get(operation_id, [])
            running = bool(pids) or bool(owned and owned["process"].poll() is None)
            pid = owned["process"].pid if owned else (pids[0] if pids else item_history.get("pid"))
            started_at = owned["started_at"] if owned else item_history.get("started_at")
            tasks.append(
                {
                    "id": operation_id,
                    "label": spec.label,
                    "group": spec.group,
                    "description": spec.description,
                    "selection": spec.selection,
                    "primary": spec.primary,
                    "caution": spec.caution,
                    "running": running,
                    "owned_by_ui": bool(owned),
                    "pids": pids or ([owned["process"].pid] if owned else []),
                    "pid": pid,
                    "started_at": started_at,
                    "elapsed_sec": round(now - started_at, 1) if running and started_at else None,
                    "exit_code": None if running else item_history.get("exit_code"),
                    "log_path": owned["log_path"] if owned else item_history.get("log_path", ""),
                    "message": "Running" if running else item_history.get("message", "Ready"),
                }
            )
        return {
            "tasks": tasks,
            "artifacts": self.artifacts(),
            "terminal_only": [
                "Host package installation (setup_host.sh)",
                "Hardware udev/network apply (onboard_hardware.sh --apply)",
                "Autoware source installation (setup_autoware.sh)",
                "acados source installation (setup_acados.sh)",
            ],
        }

    def field_runtime_status(self) -> dict:
        field = next(
            task for task in self.snapshot()["tasks"] if task["id"] == "field_navigation"
        )
        return {
            "field_running": field["running"],
            "field_pids": field["pids"],
            "owned_by_ui": field["owned_by_ui"],
            "log_path": field["log_path"],
        }

    def log_tail(self, operation_id: str, max_bytes: int = 24000) -> dict:
        if operation_id not in OPERATION_SPECS:
            raise ValueError(f"Unknown operation: {operation_id}")
        task = next(task for task in self.snapshot()["tasks"] if task["id"] == operation_id)
        path_text = task.get("log_path", "")
        if not path_text:
            return {"id": operation_id, "log_path": "", "tail": "No run log yet."}
        path = pathlib.Path(path_text)
        try:
            with path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - max_bytes))
                data = stream.read().decode(errors="replace")
        except OSError as error:
            data = f"Could not read log: {error}"
        return {"id": operation_id, "log_path": str(path), "tail": data}
