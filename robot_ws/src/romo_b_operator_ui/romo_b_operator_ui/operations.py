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
    "robot_control": OperationSpec(
        "로봇만 연결",
        "자율주행",
        "PCU 브리지와 웹 직접제어 명령 경로만 실행합니다. LiDAR·Nav2·위치추정·RViz는 실행하지 않습니다.",
        primary=True,
        discover_markers=("robot_control.launch.py",),
        caution="메인 제어에서 PCU 상태 확인 후 Arm 하면 바로 전진·후진·조향할 수 있습니다.",
    ),
    "live_mapping": OperationSpec(
        "실시간 매핑",
        "매핑",
        "Mid-360, IMU, 휠 오도메트리를 사용해 지도를 만들고 rosbag과 RViz를 함께 실행합니다.",
        primary=True,
        discover_markers=("mapping_live.launch.py",),
        caution="RC 수동 모드에서만 사용하고, LIO 오도메트리가 나타난 뒤 천천히 주행하세요.",
    ),
    "save_live_mapping": OperationSpec(
        "현재 지도 저장",
        "매핑",
        "현재 그래프를 최적화하고 PCD·포즈 그래프·Nav2 점유 지도를 생성합니다.",
        selection="map",
    ),
    "record_mapping_bag": OperationSpec(
        "매핑 bag 기록",
        "매핑",
        "LiDAR, IMU, 휠 오도메트리, TF, 플랫폼 상태와 진단 토픽을 기록합니다.",
        discover_markers=("ros2 bag record",),
    ),
    "field_navigation": OperationSpec(
        "Nav2 실주행",
        "자율주행",
        "ROMO-B LiDAR 위치추정, Nav2, 장애물 회피와 RViz 전체 스택을 실행합니다.",
        selection="map",
        primary=True,
        discover_markers=("field_navigation.launch.py",),
        caution="브리지는 비활성 상태로 시작합니다. PCU 피드백 확인 후 메인 화면에서 Arm 하세요.",
    ),
    "autoware_field": OperationSpec(
        "Autoware 실주행",
        "자율주행",
        "선택한 지도로 Autoware Universe 실주행 스택을 실행합니다.",
        selection="map",
        primary=True,
        discover_markers=("romo_b_autoware", "field.launch.py"),
        caution="기본값은 수신 전용입니다. 위 실행 설정에서 변경할 수 있습니다.",
    ),
    "autoware_planning_sim": OperationSpec(
        "Autoware 계획 시뮬레이션",
        "자율주행",
        "로봇 구동 명령 없이 Autoware 경로 계획과 RViz를 실행합니다.",
        selection="map",
        primary=True,
        discover_markers=("planning_sim.launch.py",),
    ),
    "localization_replay": OperationSpec(
        "위치추정 재생",
        "자율주행",
        "격리된 ROS 도메인에서 선택한 bag과 PCD 지도로 위치추정을 재생합니다.",
        selection="map_bag",
    ),
    "doctor_preflight": OperationSpec(
        "노트북 사전 점검",
        "점검",
        "Ubuntu, ROS 2, 개발 도구, 시리얼, 네트워크와 하드웨어 설정을 확인합니다.",
    ),
    "doctor_hardware": OperationSpec(
        "하드웨어 정밀 점검",
        "점검",
        "USB-RS232, 이더넷, LiDAR와 센서 좌표변환을 엄격하게 확인합니다.",
    ),
    "doctor_autoware": OperationSpec(
        "Autoware 환경 점검",
        "점검",
        "고정된 Autoware 버전, 빌드, 프리셋, 의존성과 지도 묶음을 확인합니다.",
        selection="map",
    ),
    "hardware_discovery": OperationSpec(
        "하드웨어 검색",
        "점검",
        "USB-RS232와 USB 이더넷을 읽기 전용으로 검색하며 시리얼 데이터는 전송하지 않습니다.",
    ),
    "serial_receive_only": OperationSpec(
        "PCU 수신 전용 확인",
        "점검",
        "시리얼 쓰기 없이 5초 동안 PCU 피드백 프레임을 해석합니다.",
        caution="먼저 /dev/romo_b_pcu를 사용 중인 다른 스택을 종료하세요.",
    ),
    "mapping_calibration": OperationSpec(
        "매핑 보정 확인",
        "점검",
        "LiDAR, IMU, 휠 오도메트리, 시간 동기, 외부 파라미터와 회전 방향을 비교합니다.",
    ),
    "yaw_sign": OperationSpec(
        "회전 방향 확인",
        "점검",
        "짧게 회전하며 휠 오도메트리와 Mid-360 IMU의 yaw 부호를 비교합니다.",
    ),
    "command_pipeline": OperationSpec(
        "속도 명령 경로 확인",
        "점검",
        "합성 명령과 장애물로 mux, 속도 평활기와 충돌 감시기 출력을 확인합니다.",
    ),
    "simulation_smoke": OperationSpec(
        "시뮬레이션 기본 시험",
        "점검",
        "격리된 환경에서 ROMO-B PTY 시뮬레이션 기본 시험을 실행합니다.",
    ),
    "nav2_preflight": OperationSpec(
        "Nav2 지도 사전 점검",
        "점검",
        "선택한 지도에서 lifecycle, 경로 계획, 명령 경로와 회전 동작을 확인합니다.",
        selection="map",
    ),
    "handoff": OperationSpec(
        "저장소 인수인계 점검",
        "점검",
        "저장소 구조, 문서, 버전 잠금, 스크립트와 재현성 표시를 확인합니다.",
    ),
    "autoware_validation": OperationSpec(
        "Autoware 전체 점검",
        "점검",
        "Autoware 인터페이스, 인지, 계획, 안전 게이트와 웨이포인트 시험을 실행합니다.",
        selection="map",
    ),
    "autoware_localization_check": OperationSpec(
        "Autoware 위치추정 점검",
        "점검",
        "격리된 환경에서 Autoware 위치추정 인터페이스를 확인합니다.",
        selection="map",
    ),
    "autoware_perception_check": OperationSpec(
        "Autoware 인지 점검",
        "점검",
        "격리된 환경에서 포인트클라우드 기반 객체 인지를 확인합니다.",
    ),
    "autoware_planning_check": OperationSpec(
        "Autoware 경로 계획 점검",
        "점검",
        "격리된 환경에서 Autoware 경로 계획을 확인합니다.",
        selection="map",
    ),
    "autoware_safety_check": OperationSpec(
        "Autoware 안전 게이트 점검",
        "점검",
        "격리된 도메인에서 명령 게이트와 운전 모드 동작을 확인합니다.",
    ),
    "autoware_waypoint_check": OperationSpec(
        "Autoware 웨이포인트 점검",
        "점검",
        "격리된 도메인에서 웨이포인트 경로 전송기를 확인합니다.",
    ),
    "build_project": OperationSpec(
        "ROMO-B 워크스페이스 빌드",
        "빌드 및 데이터",
        "저장소의 ROS 2 패키지만 빌드합니다.",
    ),
    "build_all": OperationSpec(
        "전체 의존성 빌드",
        "빌드 및 데이터",
        "Livox·외부 의존성 overlay와 ROMO-B 워크스페이스를 모두 빌드합니다.",
        caution="시간이 오래 걸릴 수 있으며 CPU와 디스크를 많이 사용합니다.",
    ),
    "fetch_dependencies": OperationSpec(
        "고정 버전 의존성 받기",
        "빌드 및 데이터",
        "저장소에서 버전을 고정한 외부 소스를 가져오거나 갱신합니다.",
    ),
    "prepare_autoware_map": OperationSpec(
        "Autoware 지도 준비",
        "빌드 및 데이터",
        "선택한 지도에서 복도용 Lanelet2·PCD·투영 설정 묶음을 생성합니다.",
        selection="map",
    ),
    "tracked_sizes": OperationSpec(
        "Git 파일 용량 확인",
        "빌드 및 데이터",
        "커밋하려는 파일에 대용량 생성물이 포함되지 않았는지 확인합니다.",
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
            raise ValueError(f"저장소 스크립트가 없습니다: {name}")
        return str(path)

    @staticmethod
    def _contained_directory(root: pathlib.Path, artifact_id: str) -> pathlib.Path:
        if not artifact_id or pathlib.Path(artifact_id).name != artifact_id:
            raise ValueError("올바른 로컬 데이터를 선택하세요")
        candidate = (root / artifact_id).resolve()
        if candidate.parent != root.resolve() or not candidate.is_dir():
            raise ValueError(f"로컬 데이터가 없습니다: {artifact_id}")
        return candidate

    def _map(self, payload: dict, required: bool = True) -> pathlib.Path | None:
        artifact_id = str(payload.get("map_id", ""))
        if artifact_id:
            return self._contained_directory(self.map_root, artifact_id)
        maps = self.artifacts()["maps"]
        if maps:
            return pathlib.Path(maps[0]["path"])
        if required:
            raise ValueError("사용할 수 있는 로컬 지도가 없습니다")
        return None

    def _bag(self, payload: dict) -> pathlib.Path:
        artifact_id = str(payload.get("bag_id", ""))
        if artifact_id:
            return self._contained_directory(self.bag_root, artifact_id)
        bags = self.artifacts()["bags"]
        if not bags:
            raise ValueError("사용할 수 있는 로컬 rosbag이 없습니다")
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
        if operation_id == "robot_control":
            env["MAX_SPEED_MPS"] = str(payload.get("max_speed_mps", 0.5))
            return [self._script("run_robot_control.sh")], env
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
                raise ValueError("재생 배속은 0.1~4.0이어야 합니다")
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
        raise ValueError(f"알 수 없는 작업입니다: {operation_id}")

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
                    "message": "완료" if return_code == 0 else f"종료 코드 {return_code}",
                }
                del self._owned[operation_id]
                self._discovery_deadline = 0.0

    def start(self, operation_id: str, payload: dict | None = None) -> dict:
        if operation_id not in OPERATION_SPECS:
            raise ValueError(f"알 수 없는 작업입니다: {operation_id}")
        payload = payload or {}
        self._poll_owned()
        discovered = self._scan_processes()
        if discovered.get(operation_id):
            return {
                "accepted": False,
                "message": f"{OPERATION_SPECS[operation_id].label} 작업이 이미 실행 중입니다 (PID {discovered[operation_id][0]})",
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
                    "message": f"먼저 {OPERATION_SPECS[conflict].label} 작업을 종료하세요",
                }
        with self._lock:
            if operation_id in self._owned:
                return {
                    "accepted": False,
                    "message": f"{OPERATION_SPECS[operation_id].label} 작업이 이미 실행 중입니다",
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
                "message": "실행 요청됨",
            }
            self._discovery_deadline = 0.0
            self._artifact_deadline = 0.0
        return {
            "accepted": True,
            "message": f"{OPERATION_SPECS[operation_id].label} 실행을 요청했습니다 (PID {process.pid})",
            "pid": process.pid,
            "log_path": str(log_path),
        }

    def stop(self, operation_id: str) -> dict:
        if operation_id not in OPERATION_SPECS:
            raise ValueError(f"알 수 없는 작업입니다: {operation_id}")
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
                "message": f"{OPERATION_SPECS[operation_id].label} 작업은 이미 종료되어 있습니다",
            }
        return {
            "accepted": True,
            "message": f"{OPERATION_SPECS[operation_id].label} 종료를 요청했습니다. PID: "
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
                    "message": "실행 중" if running else item_history.get("message", "실행 대기"),
                }
            )
        return {
            "tasks": tasks,
            "artifacts": self.artifacts(),
            "terminal_only": [
                "노트북 패키지 설치 (setup_host.sh)",
                "하드웨어 udev·네트워크 적용 (onboard_hardware.sh --apply)",
                "Autoware 소스 설치 (setup_autoware.sh)",
                "acados 소스 설치 (setup_acados.sh)",
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
            raise ValueError(f"알 수 없는 작업입니다: {operation_id}")
        task = next(task for task in self.snapshot()["tasks"] if task["id"] == operation_id)
        path_text = task.get("log_path", "")
        if not path_text:
            return {"id": operation_id, "log_path": "", "tail": "아직 실행 로그가 없습니다."}
        path = pathlib.Path(path_text)
        try:
            with path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - max_bytes))
                data = stream.read().decode(errors="replace")
        except OSError as error:
            data = f"로그를 읽을 수 없습니다: {error}"
        return {"id": operation_id, "log_path": str(path), "tail": data}
