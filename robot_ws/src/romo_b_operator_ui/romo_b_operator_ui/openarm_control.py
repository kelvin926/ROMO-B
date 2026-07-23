"""OpenArm-v1 bimanual SocketCAN control for the browser operator console.

The controller is deliberately independent from ExoArm, ROS topics, cameras,
and learned policies.  Opening the CAN sockets does not enable a motor.  Motor
commands become possible only after explicit feedback, enable, and target
requests from the operator page.
"""

from __future__ import annotations

import copy
import math
import os
import pathlib
import re
import select
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import yaml


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_SFF_MASK = 0x7FF
CAN_INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,15}$")
SIDES = ("left", "right")

MOTOR_LIMITS = {
    "DM4310": (12.5, 30.0, 10.0),
    "DM4340": (12.5, 8.0, 28.0),
    "DM8009": (12.5, 45.0, 54.0),
}


@dataclass(frozen=True)
class JointSpec:
    key: str
    label: str
    motor_type: str
    send_id: int
    receive_id: int
    lower_rad: float
    upper_rad: float
    velocity_limit_rad_s: float
    kp: float
    kd: float

    @property
    def lower_deg(self) -> float:
        return math.degrees(self.lower_rad)

    @property
    def upper_deg(self) -> float:
        return math.degrees(self.upper_rad)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _double_to_uint(value: float, lower: float, upper: float, bits: int) -> int:
    clipped = _clamp(float(value), lower, upper)
    normalized = (clipped - lower) / (upper - lower)
    return int(normalized * ((1 << bits) - 1))


def _uint_to_double(value: int, lower: float, upper: float, bits: int) -> float:
    return (float(value) / ((1 << bits) - 1)) * (upper - lower) + lower


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    if not 0 <= int(can_id) <= CAN_SFF_MASK:
        raise ValueError("Classic CAN ID must be in 0x000..0x7ff")
    if len(data) > 8:
        raise ValueError("Classic CAN payload cannot exceed 8 bytes")
    return CAN_FRAME.pack(int(can_id), len(data), data.ljust(8, b"\0"))


def unpack_can_frame(frame: bytes) -> tuple[int, bytes]:
    if len(frame) < CAN_FRAME.size:
        raise ValueError("short SocketCAN frame")
    can_id, length, data = CAN_FRAME.unpack(frame[: CAN_FRAME.size])
    return can_id & CAN_SFF_MASK, data[: min(int(length), 8)]


def pack_mit_data(
    motor_type: str,
    position_rad: float,
    kp: float,
    kd: float,
    velocity_rad_s: float = 0.0,
    torque_nm: float = 0.0,
) -> bytes:
    position_max, velocity_max, torque_max = MOTOR_LIMITS[motor_type]
    q_uint = _double_to_uint(position_rad, -position_max, position_max, 16)
    dq_uint = _double_to_uint(velocity_rad_s, -velocity_max, velocity_max, 12)
    kp_uint = _double_to_uint(kp, 0.0, 500.0, 12)
    kd_uint = _double_to_uint(kd, 0.0, 5.0, 12)
    tau_uint = _double_to_uint(torque_nm, -torque_max, torque_max, 12)
    return bytes(
        (
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            dq_uint >> 4,
            ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF),
            kp_uint & 0xFF,
            kd_uint >> 4,
            ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF),
            tau_uint & 0xFF,
        )
    )


def decode_motor_feedback(spec: JointSpec, data: bytes) -> dict:
    if len(data) < 8:
        raise ValueError("motor feedback must contain 8 bytes")
    _, velocity_max, torque_max = MOTOR_LIMITS[spec.motor_type]
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0xF) << 8) | data[5]
    state_code = int((data[0] >> 4) & 0xF)
    state_names = {
        0x0: "DISABLED",
        0x1: "ENABLED",
        0x8: "OVER_VOLTAGE",
        0x9: "UNDER_VOLTAGE",
        0xA: "OVER_CURRENT",
        0xB: "MOS_OVER_TEMP",
        0xC: "ROTOR_OVER_TEMP",
        0xD: "LOST_COMM",
        0xE: "OVERLOAD",
    }
    return {
        "position_rad": _uint_to_double(q_uint, -12.5, 12.5, 16),
        "velocity_rad_s": _uint_to_double(
            dq_uint, -velocity_max, velocity_max, 12
        ),
        "torque_nm": _uint_to_double(tau_uint, -torque_max, torque_max, 12),
        "mos_temp_c": int(data[6]),
        "rotor_temp_c": int(data[7]),
        "status_byte": int(data[0]),
        # 0x0 and 0x1 are the normal DISABLED/ENABLED states.  Only the
        # documented 0x8..0xE range represents a motor fault.
        "state_code": state_code,
        "state_name": state_names.get(state_code, f"UNKNOWN_{state_code:X}"),
        "fault_code": state_code if 0x8 <= state_code <= 0xE else 0,
    }


class OpenArmController:
    """Own two Classic-CAN sockets and a bounded position-control loop."""

    def __init__(
        self,
        repo_root: pathlib.Path,
        *,
        socket_factory: Callable[[str], object] | None = None,
        interface_probe: Callable[[str], dict] | None = None,
        interface_preparer: Callable[[tuple[str, ...]], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        start_worker: bool = True,
    ):
        self.repo_root = pathlib.Path(repo_root).resolve()
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._socket_factory = socket_factory or self._open_socket
        self._interface_probe = interface_probe or self._read_interface
        self._interface_preparer = interface_preparer or self._prepare_can_interfaces
        self._config = self._load_yaml(
            self.repo_root / "openarm" / "config" / "openarm_v1_bimanual.yaml"
        )
        self._joint_limits = self._load_yaml(
            self.repo_root / "openarm" / "config" / "joint_limits.yaml"
        )
        self._specs = self._make_specs()
        self._interfaces = {
            side: str(self._config["interfaces"][side]["name"]) for side in SIDES
        }
        self._configure_interfaces_automatically = bool(
            self._config.get("startup_policy", {}).get(
                "configure_interfaces_automatically", False
            )
        )
        control = self._config["operator_control"]
        self._command_rate_hz = float(control["command_rate_hz"])
        self._refresh_rate_hz = float(control["refresh_rate_hz"])
        self._speed_deg_s = float(control["default_target_speed_deg_s"])
        configured_max_speed = float(control["maximum_target_speed_deg_s"])
        hardware_max_speed = max(
            math.degrees(spec.velocity_limit_rad_s) for spec in self._specs
        )
        self._max_speed_deg_s = min(configured_max_speed, hardware_max_speed)
        self._minimum_trajectory_duration = max(
            0.05, float(control.get("minimum_trajectory_duration_sec", 0.12))
        )
        self._gain_scale = 1.0
        self._max_gain_scale = float(control["maximum_gain_scale"])
        self._calibration_max_velocity = float(
            control.get("calibration_max_velocity_rad_s", 0.05)
        )
        self._calibration_tolerance_deg = float(
            control.get("calibration_zero_tolerance_deg", 2.0)
        )
        self._calibration_step_delay = float(
            control.get("calibration_step_delay_sec", 0.1)
        )
        self._sockets: dict[str, object] = {}
        self._enabled = {side: False for side in SIDES}
        self._interlocks: dict[str, str] = {side: "" for side in SIDES}
        self._desired: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._commanded: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._trajectories: dict[str, dict | None] = {side: None for side in SIDES}
        self._operator_limits_path = (
            self.repo_root / "config" / "local" / "openarm_operator_joint_limits.yaml"
        )
        self._soft_limits = {
            side: [[spec.lower_rad, spec.upper_rad] for spec in self._specs]
            for side in SIDES
        }
        self._load_operator_limits()
        self._motors = {
            side: [self._empty_motor(spec) for spec in self._specs] for side in SIDES
        }
        self._rx_count = {side: 0 for side in SIDES}
        self._tx_count = {side: 0 for side in SIDES}
        self._last_error = ""
        self._last_calibration = ""
        self._calibration = self._empty_calibration()
        self._calibration_feedback_baseline: dict[tuple[str, int], float | None] = {}
        self._log = deque(maxlen=120)
        self._last_refresh = 0.0
        self._last_command = self._clock()
        self._worker = None
        if start_worker:
            self._worker = threading.Thread(
                target=self._run, name="openarm-can", daemon=True
            )
            self._worker.start()

    @staticmethod
    def _load_yaml(path: pathlib.Path) -> dict:
        if not path.is_file():
            raise RuntimeError(f"OpenArm 설정 파일이 없습니다: {path}")
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise RuntimeError(f"OpenArm 설정 형식이 올바르지 않습니다: {path}")
        return data

    def _make_specs(self) -> list[JointSpec]:
        motors = self._config["motors"]
        control = self._config["operator_control"]
        kp_values = [float(value) for value in control["arm_kp"]]
        kd_values = [float(value) for value in control["arm_kd"]]
        specs = []
        for index in range(7):
            key = f"joint_{index + 1}"
            source = motors[key]
            limits = self._joint_limits[f"joint{index + 1}"]["limit"]
            motor_velocity_limit = MOTOR_LIMITS[str(source["type"])][1]
            specs.append(
                JointSpec(
                    key=key,
                    label=f"J{index + 1}",
                    motor_type=str(source["type"]),
                    send_id=int(source["send_id"]),
                    receive_id=int(source["receive_id"]),
                    lower_rad=float(limits["lower"]),
                    upper_rad=float(limits["upper"]),
                    velocity_limit_rad_s=min(
                        float(limits.get("velocity", motor_velocity_limit)),
                        motor_velocity_limit,
                    ),
                    kp=kp_values[index],
                    kd=kd_values[index],
                )
            )
        gripper = motors["gripper"]
        grip_lower, grip_upper = control["gripper_limit_deg"]
        specs.append(
            JointSpec(
                key="gripper",
                label="GRIP",
                motor_type=str(gripper["type"]),
                send_id=int(gripper["send_id"]),
                receive_id=int(gripper["receive_id"]),
                lower_rad=math.radians(float(grip_lower)),
                upper_rad=math.radians(float(grip_upper)),
                velocity_limit_rad_s=min(
                    float(
                        control.get(
                            "gripper_velocity_limit_rad_s",
                            MOTOR_LIMITS[str(gripper["type"])][1],
                        )
                    ),
                    MOTOR_LIMITS[str(gripper["type"])][1],
                ),
                kp=float(control["gripper_kp"]),
                kd=float(control["gripper_kd"]),
            )
        )
        return specs

    @staticmethod
    def _empty_motor(spec: JointSpec) -> dict:
        return {
            "key": spec.key,
            "label": spec.label,
            "motor_type": spec.motor_type,
            "send_id": spec.send_id,
            "receive_id": spec.receive_id,
            "lower_deg": round(spec.lower_deg, 3),
            "upper_deg": round(spec.upper_deg, 3),
            "hard_lower_deg": round(spec.lower_deg, 3),
            "hard_upper_deg": round(spec.upper_deg, 3),
            "velocity_limit_deg_s": round(math.degrees(spec.velocity_limit_rad_s), 3),
            "kp": spec.kp,
            "kd": spec.kd,
            "position_rad": None,
            "velocity_rad_s": None,
            "torque_nm": None,
            "mos_temp_c": None,
            "rotor_temp_c": None,
            "status_byte": None,
            "state_code": None,
            "state_name": "UNKNOWN",
            "fault_code": None,
            "last_rx": None,
            "target_rad": None,
        }

    def _load_operator_limits(self) -> None:
        """Load persisted soft limits, ignoring entries outside hard limits."""
        try:
            raw = yaml.safe_load(self._operator_limits_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            return
        limits = raw.get("limits", {}) if isinstance(raw, dict) else {}
        for side in SIDES:
            side_limits = limits.get(side, {}) if isinstance(limits, dict) else {}
            if not isinstance(side_limits, dict):
                continue
            for index, spec in enumerate(self._specs):
                entry = side_limits.get(spec.key)
                if not isinstance(entry, dict):
                    continue
                try:
                    lower = math.radians(float(entry["lower_deg"]))
                    upper = math.radians(float(entry["upper_deg"]))
                except (KeyError, TypeError, ValueError):
                    continue
                if (
                    math.isfinite(lower)
                    and math.isfinite(upper)
                    and spec.lower_rad <= lower < upper <= spec.upper_rad
                ):
                    self._soft_limits[side][index] = [lower, upper]

    def _save_operator_limits_locked(self) -> None:
        payload = {
            "schema_version": 1,
            "limits": {
                side: {
                    spec.key: {
                        "lower_deg": round(math.degrees(self._soft_limits[side][index][0]), 6),
                        "upper_deg": round(math.degrees(self._soft_limits[side][index][1]), 6),
                    }
                    for index, spec in enumerate(self._specs)
                }
                for side in SIDES
            },
        }
        self._operator_limits_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._operator_limits_path.with_suffix(".yaml.tmp")
        temporary.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        os.replace(temporary, self._operator_limits_path)

    def _soft_bounds_locked(self, side: str, index: int) -> tuple[float, float]:
        lower, upper = self._soft_limits[side][index]
        return float(lower), float(upper)

    def _empty_calibration(self) -> dict:
        return {
            "status": "idle",
            "message": "캘리브레이션 준비 점검 전입니다",
            "ready": False,
            "verified": False,
            "selection_key": "",
            "target_sides": [],
            "target_joints": [],
            "checked_at": "",
            "written_at": "",
            "verified_at": "",
            "preflight": [],
            "results": [],
            "tolerance_deg": self._calibration_tolerance_deg,
            "max_velocity_rad_s": self._calibration_max_velocity,
            "automatic_tool": {
                "available": False,
                "enabled": False,
                "reason": (
                    "원본 기계 한계 자동 보정은 정밀 홈 이동이 미완성이어서 "
                    "웹에서 실행하지 않습니다"
                ),
            },
        }

    def _automatic_calibration_tool(self) -> dict:
        python_bin = self.repo_root / "openarm" / ".venv" / "bin" / "python3"
        tool = (
            self.repo_root
            / "openarm"
            / "src"
            / "openarm_can"
            / "setup"
            / "openarm-can-zero-position-calibration"
        )
        bindings = list(
            (self.repo_root / "openarm" / ".venv" / "lib").glob(
                "python*/site-packages/openarm_can/openarm_can*.so"
            )
        )
        available = python_bin.is_file() and tool.is_file() and bool(bindings)
        return {
            "available": available,
            "enabled": available,
            "version": "v1",
            "reason": (
                "원본 OpenArm 기계 한계 자동 보정 도구가 준비되었습니다"
                if available
                else "OpenArm Python 바인딩을 먼저 준비해야 합니다"
            ),
        }

    @staticmethod
    def _read_interface(name: str) -> dict:
        root = pathlib.Path("/sys/class/net") / name
        if not root.is_dir():
            return {
                "name": name,
                "exists": False,
                "is_can": False,
                "up": False,
                "operstate": "missing",
            }
        try:
            kind = int((root / "type").read_text().strip())
            flags = int((root / "flags").read_text().strip(), 0)
            operstate = (root / "operstate").read_text().strip()
        except (OSError, ValueError):
            kind, flags, operstate = -1, 0, "unknown"
        statistics = {}
        for key in ("rx_packets", "tx_packets", "rx_errors", "tx_errors"):
            try:
                statistics[key] = int((root / "statistics" / key).read_text().strip())
            except (OSError, ValueError):
                statistics[key] = 0
        return {
            "name": name,
            "exists": True,
            "is_can": kind == 280,
            "up": bool(flags & 0x1),
            "operstate": operstate,
            "statistics": statistics,
        }

    @staticmethod
    def _open_socket(interface: str):
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.setblocking(False)
        can_socket.bind((interface,))
        return can_socket

    @staticmethod
    def _prepare_can_interfaces(interfaces: tuple[str, ...]) -> None:
        """Bring validated OpenArm SocketCAN links up without sending CAN frames."""
        helper = pathlib.Path(
            os.environ.get(
                "ROMO_B_OPENARM_CAN_HELPER",
                "/usr/local/sbin/romo-b-openarm-can-link",
            )
        )
        if not helper.is_file():
            raise RuntimeError(
                "CAN 링크 설정 도우미가 설치되지 않았습니다. "
                "scripts/install_openarm_can_helper.sh를 한 번 실행하세요"
            )
        try:
            result = subprocess.run(
                ["/usr/bin/sudo", "-n", str(helper), *interfaces],
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(f"CAN 링크를 준비하지 못했습니다: {error}") from error
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "권한 또는 링크 상태를 확인하세요").strip()
            raise RuntimeError(f"CAN 링크를 준비하지 못했습니다: {detail}")

    def _log_event(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._log.append(f"[{stamp}] {message}")

    @staticmethod
    def _side_list(side: str | None) -> tuple[str, ...]:
        if side in (None, "both"):
            return SIDES
        if side not in SIDES:
            raise ValueError("팔 선택은 left, right 또는 both여야 합니다")
        return (side,)

    def _require_connected(self, sides: tuple[str, ...]) -> None:
        missing = [side for side in sides if side not in self._sockets]
        if missing:
            raise RuntimeError(
                "CAN이 연결되지 않았습니다: " + ", ".join(missing)
            )

    def _send_locked(self, side: str, can_id: int, data: bytes) -> None:
        can_socket = self._sockets.get(side)
        if can_socket is None:
            raise RuntimeError(f"{side} CAN socket이 연결되지 않았습니다")
        sent = can_socket.send(pack_can_frame(can_id, data))
        if sent != CAN_FRAME.size:
            raise RuntimeError(f"{side} CAN frame 전송 길이가 올바르지 않습니다")
        self._tx_count[side] += 1

    def _send_simple_locked(self, side: str, command: int, indices=None) -> None:
        selected = range(len(self._specs)) if indices is None else indices
        data = b"\xff" * 7 + bytes((command,))
        for index in selected:
            self._send_locked(side, self._specs[index].send_id, data)

    def _refresh_locked(self, sides: tuple[str, ...]) -> None:
        for side in sides:
            for spec in self._specs:
                payload = bytes((spec.send_id & 0xFF, 0, 0xCC, 0, 0, 0, 0, 0))
                self._send_locked(side, 0x7FF, payload)
        self._last_refresh = self._clock()

    def _motor_is_online(self, motor: dict, now: float, max_age: float = 1.0) -> bool:
        last_rx = motor.get("last_rx")
        return last_rx is not None and 0.0 <= now - float(last_rx) <= max_age

    def _feedback_problem_locked(self, side: str, max_age: float = 1.0) -> str:
        now = self._clock()
        offline = [
            motor["label"]
            for motor in self._motors[side]
            if not self._motor_is_online(motor, now, max_age)
        ]
        if offline:
            return "피드백 지연: " + ", ".join(offline)
        faults = [
            f'{motor["label"]}(code {int(motor.get("fault_code") or 0)})'
            for motor in self._motors[side]
            if int(motor.get("fault_code") or 0) != 0
        ]
        if faults:
            return "모터 fault: " + ", ".join(faults)
        return ""

    def _require_feedback_ready_locked(self, side: str) -> None:
        problem = self._feedback_problem_locked(side, 1.5)
        if problem:
            raise RuntimeError(f"{side} 팔 제어 준비 실패 - {problem}")

    def _require_motion_ready_locked(self, side: str) -> None:
        self._require_feedback_ready_locked(side)
        if self._interlocks[side]:
            raise RuntimeError(
                f"{side} 팔 명령이 차단되었습니다: {self._interlocks[side]}. "
                "현재 자세 유지를 눌러 명시적으로 재개하세요"
            )

    def _current_positions_locked(self, side: str) -> list[float]:
        self._require_feedback_ready_locked(side)
        return [float(motor["position_rad"]) for motor in self._motors[side]]

    def connect(self, payload: dict) -> dict:
        left = str(payload.get("left_interface", self._interfaces["left"]))
        right = str(payload.get("right_interface", self._interfaces["right"]))
        if not CAN_INTERFACE_PATTERN.fullmatch(left) or not CAN_INTERFACE_PATTERN.fullmatch(right):
            raise ValueError("CAN 인터페이스 이름 형식이 올바르지 않습니다")
        if left == right:
            raise ValueError("좌·우 팔은 서로 다른 CAN 인터페이스를 사용해야 합니다")
        requested = {"left": left, "right": right}
        opened = {}
        with self._lock:
            if self._sockets:
                raise RuntimeError("CAN이 이미 연결되어 있습니다. 먼저 연결 해제하세요")
            down_interfaces = []
            for side, name in requested.items():
                info = self._interface_probe(name)
                if not info.get("exists") or not info.get("is_can"):
                    raise RuntimeError(f"{name}은 사용 가능한 SocketCAN 장치가 아닙니다")
                if not info.get("up"):
                    down_interfaces.append(name)
            if down_interfaces and self._configure_interfaces_automatically:
                self._interface_preparer(tuple(down_interfaces))
            for name in down_interfaces:
                if not self._interface_probe(name).get("up"):
                    raise RuntimeError(f"{name} 인터페이스가 UP 상태가 아닙니다")
            try:
                for side, name in requested.items():
                    opened[side] = self._socket_factory(name)
            except Exception:
                for can_socket in opened.values():
                    try:
                        can_socket.close()
                    except Exception:
                        pass
                raise
            self._interfaces = requested
            self._sockets = opened
            self._enabled = {side: False for side in SIDES}
            self._interlocks = {side: "" for side in SIDES}
            self._desired = {side: None for side in SIDES}
            self._commanded = {side: None for side in SIDES}
            self._trajectories = {side: None for side in SIDES}
            self._calibration = self._empty_calibration()
            self._calibration_feedback_baseline = {}
            self._last_error = ""
            self._log_event(f"CAN 연결: 왼팔 {left}, 오른팔 {right}")
            self._refresh_locked(SIDES)
        message = "OpenArm 양팔 CAN을 연결했습니다"
        if down_interfaces:
            message += " · CAN 링크를 1 Mbps로 자동 준비했습니다"
        return {"accepted": True, "message": message}

    def disconnect(self) -> dict:
        with self._lock:
            if not self._sockets:
                return {"accepted": True, "message": "OpenArm CAN은 이미 연결 해제 상태입니다"}
            for side in SIDES:
                if side in self._sockets:
                    try:
                        self._send_simple_locked(side, 0xFD)
                    except Exception as error:
                        self._last_error = str(error)
            sockets = list(self._sockets.values())
            self._sockets = {}
            self._enabled = {side: False for side in SIDES}
            self._interlocks = {side: "" for side in SIDES}
            self._desired = {side: None for side in SIDES}
            self._commanded = {side: None for side in SIDES}
            self._trajectories = {side: None for side in SIDES}
            self._calibration = self._empty_calibration()
            self._calibration_feedback_baseline = {}
            self._log_event("모터 disable 후 CAN 연결 해제")
        for can_socket in sockets:
            try:
                can_socket.close()
            except Exception:
                pass
        return {"accepted": True, "message": "모터를 disable하고 CAN 연결을 해제했습니다"}

    def refresh(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        with self._lock:
            self._require_connected(sides)
            self._refresh_locked(sides)
            self._log_event("모터 상태 갱신 요청: " + ", ".join(sides))
        return {"accepted": True, "message": "8축 모터 상태 갱신을 요청했습니다"}

    def set_enabled(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        enabled = bool(payload.get("enabled", False))
        with self._lock:
            self._require_connected(sides)
            if enabled:
                current = {side: self._current_positions_locked(side) for side in sides}
                for side in sides:
                    self._desired[side] = list(current[side])
                    self._commanded[side] = list(current[side])
                    self._trajectories[side] = None
                    self._interlocks[side] = ""
                    self._send_simple_locked(side, 0xFC)
                    self._enabled[side] = True
                    self._send_pose_locked(side, current[side])
                self._log_event("현재 자세 유지로 모터 enable: " + ", ".join(sides))
                return {"accepted": True, "message": "현재 자세를 유지하며 모터를 enable했습니다"}
            for side in sides:
                self._send_simple_locked(side, 0xFD)
                self._enabled[side] = False
                self._interlocks[side] = ""
                self._desired[side] = None
                self._commanded[side] = None
                self._trajectories[side] = None
            self._log_event("모터 disable: " + ", ".join(sides))
        return {"accepted": True, "message": "선택한 팔의 모터를 disable했습니다"}

    def clear_errors(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        with self._lock:
            self._require_connected(sides)
            for side in sides:
                self._send_simple_locked(side, 0xFB)
            self._log_event("모터 오류 해제 요청: " + ", ".join(sides))
        return {"accepted": True, "message": "모터 오류 해제 명령을 전송했습니다"}

    def hold_current(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        with self._lock:
            self._require_connected(sides)
            for side in sides:
                if not self._enabled[side]:
                    raise RuntimeError(f"{side} 팔 모터가 enable되지 않았습니다")
                current = self._current_positions_locked(side)
                self._desired[side] = list(current)
                self._commanded[side] = list(current)
                self._trajectories[side] = None
                self._interlocks[side] = ""
                self._send_pose_locked(side, current)
            self._log_event("현재 자세 유지: " + ", ".join(sides))
        return {"accepted": True, "message": "현재 피드백 자세를 목표로 고정했습니다"}

    def _set_options_locked(self, payload: dict) -> None:
        if "speed_deg_s" in payload:
            value = float(payload["speed_deg_s"])
            if not math.isfinite(value):
                raise ValueError("관절 속도는 유한한 숫자여야 합니다")
            self._speed_deg_s = _clamp(value, 1.0, self._max_speed_deg_s)
        if "gain_scale" in payload:
            value = float(payload["gain_scale"])
            if not math.isfinite(value):
                raise ValueError("gain 배율은 유한한 숫자여야 합니다")
            self._gain_scale = _clamp(value, 0.05, self._max_gain_scale)

    def set_joint(self, payload: dict) -> dict:
        side = str(payload.get("side", ""))
        if side not in SIDES:
            raise ValueError("joint 명령에는 left 또는 right 팔을 지정해야 합니다")
        index = int(payload.get("joint", -1))
        if not 0 <= index < len(self._specs):
            raise ValueError("joint 번호는 0..7 범위여야 합니다")
        target_deg = float(payload.get("target_deg"))
        if not math.isfinite(target_deg):
            raise ValueError("목표 각도는 유한한 숫자여야 합니다")
        with self._lock:
            self._require_connected((side,))
            if not self._enabled[side]:
                raise RuntimeError(f"{side} 팔 모터가 enable되지 않았습니다")
            self._require_motion_ready_locked(side)
            self._set_options_locked(payload)
            if self._desired[side] is None:
                self._desired[side] = self._current_positions_locked(side)
            spec = self._specs[index]
            lower, upper = self._soft_bounds_locked(side, index)
            target_rad = _clamp(math.radians(target_deg), lower, upper)
            self._desired[side][index] = target_rad
            self._plan_trajectory_locked(side, self._desired[side])
            self._log_event(f"{side} {spec.label} 목표 {math.degrees(target_rad):.2f}°")
        return {
            "accepted": True,
            "message": f"{side} {self._specs[index].label} 목표를 적용했습니다",
        }

    def set_pose(self, payload: dict) -> dict:
        targets = payload.get("targets")
        if not isinstance(targets, dict):
            raise ValueError("targets는 left/right 배열을 포함해야 합니다")
        requested_sides = tuple(side for side in SIDES if side in targets)
        if not requested_sides:
            raise ValueError("left 또는 right 목표 배열이 필요합니다")
        prepared_degrees = {}
        for side in requested_sides:
            values = targets[side]
            if not isinstance(values, list) or len(values) != len(self._specs):
                raise ValueError(f"{side} 목표는 8개 각도 배열이어야 합니다")
            converted = []
            for spec, raw in zip(self._specs, values):
                value = float(raw)
                if not math.isfinite(value):
                    raise ValueError("모든 목표 각도는 유한한 숫자여야 합니다")
                converted.append(value)
            prepared_degrees[side] = converted
        with self._lock:
            self._require_connected(requested_sides)
            for side in requested_sides:
                if not self._enabled[side]:
                    raise RuntimeError(f"{side} 팔 모터가 enable되지 않았습니다")
                self._require_motion_ready_locked(side)
            self._set_options_locked(payload)
            for side, values in prepared_degrees.items():
                prepared = []
                for index, value in enumerate(values):
                    lower, upper = self._soft_bounds_locked(side, index)
                    prepared.append(_clamp(math.radians(value), lower, upper))
                self._desired[side] = prepared
                if self._commanded[side] is None:
                    self._commanded[side] = self._current_positions_locked(side)
                self._plan_trajectory_locked(side, prepared)
            self._log_event("양팔 자세 목표 반영: " + ", ".join(requested_sides))
        return {"accepted": True, "message": "선택한 팔의 8축 목표 자세를 적용했습니다"}

    def set_joint_limits(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        index = int(payload.get("joint", -1))
        if not 0 <= index < len(self._specs):
            raise ValueError("joint 번호는 0..7 범위여야 합니다")
        spec = self._specs[index]
        reset = bool(payload.get("reset", False))
        if reset:
            lower, upper = spec.lower_rad, spec.upper_rad
        else:
            try:
                lower = math.radians(float(payload["lower_deg"]))
                upper = math.radians(float(payload["upper_deg"]))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("운용 하한과 상한을 숫자로 입력해야 합니다") from error
            if not math.isfinite(lower) or not math.isfinite(upper):
                raise ValueError("운용 하한과 상한은 유한한 숫자여야 합니다")
            if lower >= upper:
                raise ValueError("운용 하한은 상한보다 작아야 합니다")
            tolerance = math.radians(0.001)
            if lower < spec.lower_rad - tolerance or upper > spec.upper_rad + tolerance:
                raise ValueError(
                    f"{spec.label} 운용 범위는 하드 한계 "
                    f"{spec.lower_deg:.1f}°..{spec.upper_deg:.1f}° 안이어야 합니다"
                )
            lower = max(lower, spec.lower_rad)
            upper = min(upper, spec.upper_rad)
        with self._lock:
            for side in sides:
                self._soft_limits[side][index] = [lower, upper]
                # Editing a limit never starts a move.  If a trajectory was in
                # progress, hold its latest commanded position and apply the new
                # soft range only to subsequent target requests.
                if self._commanded[side] is not None:
                    self._desired[side] = list(self._commanded[side])
                self._trajectories[side] = None
            self._save_operator_limits_locked()
            verb = "원본 하드 한계로 복원" if reset else "운용 범위 저장"
            self._log_event(
                f"{','.join(sides)} {spec.label} {verb}: "
                f"{math.degrees(lower):.1f}°..{math.degrees(upper):.1f}°"
            )
        return {
            "accepted": True,
            "message": (
                f"{spec.label} 운용 범위를 {math.degrees(lower):.1f}°.."
                f"{math.degrees(upper):.1f}°로 저장했습니다 · 팔은 움직이지 않았습니다"
            ),
        }

    def _calibration_selection(
        self, payload: dict
    ) -> tuple[tuple[str, ...], tuple[int, ...], str]:
        sides = self._side_list(payload.get("side"))
        joint = payload.get("joint")
        if joint is None:
            indices = tuple(range(len(self._specs)))
        else:
            index = int(joint)
            if not 0 <= index < len(self._specs):
                raise ValueError("joint 번호는 0..7 범위여야 합니다")
            indices = (index,)
        selection_key = (
            ",".join(sides) + ":" + ",".join(str(index) for index in indices)
        )
        return sides, indices, selection_key

    def _calibration_readings_locked(
        self, sides: tuple[str, ...], indices: tuple[int, ...]
    ) -> list[dict]:
        now = self._clock()
        readings = []
        for side in sides:
            for index in indices:
                motor = self._motors[side][index]
                position = motor.get("position_rad")
                velocity = motor.get("velocity_rad_s")
                last_rx = motor.get("last_rx")
                age = None if last_rx is None else max(0.0, now - float(last_rx))
                readings.append(
                    {
                        "side": side,
                        "joint": index,
                        "label": self._specs[index].label,
                        "position_deg": (
                            None
                            if position is None
                            else round(math.degrees(float(position)), 3)
                        ),
                        "velocity_rad_s": (
                            None if velocity is None else round(float(velocity), 4)
                        ),
                        "fault_code": int(motor.get("fault_code") or 0),
                        "online": self._motor_is_online(motor, now, 1.5),
                        "age_sec": None if age is None else round(age, 3),
                    }
                )
        return readings

    def calibration_preflight(self, payload: dict) -> dict:
        sides, indices, selection_key = self._calibration_selection(payload)
        issues = []
        with self._lock:
            missing = [side for side in sides if side not in self._sockets]
            if missing:
                issues.append("CAN 미연결: " + ", ".join(missing))
            for side in sides:
                if side not in self._sockets:
                    continue
                if self._enabled[side]:
                    issues.append(f"{side} 팔 모터를 먼저 disable해야 합니다")
                problem = self._feedback_problem_locked(side, 1.5)
                if problem:
                    issues.append(f"{side} 팔 {problem}")
            readings = self._calibration_readings_locked(sides, indices)
            moving = [
                f'{item["side"]} {item["label"]}'
                for item in readings
                if item["velocity_rad_s"] is None
                or abs(float(item["velocity_rad_s"]))
                > self._calibration_max_velocity
            ]
            if moving:
                issues.append("정지 확인 실패: " + ", ".join(moving))
            ready = not issues
            checked_at = time.strftime("%Y-%m-%d %H:%M:%S")
            self._calibration = {
                **self._empty_calibration(),
                "status": "ready" if ready else "blocked",
                "message": (
                    "영점 저장 준비가 완료되었습니다"
                    if ready
                    else " · ".join(issues)
                ),
                "ready": ready,
                "selection_key": selection_key,
                "target_sides": list(sides),
                "target_joints": [self._specs[index].label for index in indices],
                "checked_at": checked_at,
                "preflight": readings,
            }
            self._log_event(
                "영점 캘리브레이션 준비 점검: "
                + ("통과" if ready else "차단 - " + "; ".join(issues))
            )
            calibration = copy.deepcopy(self._calibration)
        return {
            "accepted": True,
            "ready": ready,
            "message": calibration["message"],
            "calibration": calibration,
        }

    def calibrate_zero(self, payload: dict) -> dict:
        if payload.get("confirmation") != "OPENARM ZERO":
            raise ValueError("영점 저장 확인 문구가 일치하지 않습니다")
        sides, indices, selection_key = self._calibration_selection(payload)
        preflight = self.calibration_preflight(payload)
        if not preflight["ready"]:
            raise RuntimeError(preflight["message"])

        with self._lock:
            self._require_connected(sides)
            if any(self._enabled[side] for side in sides):
                raise RuntimeError("영점 저장 전에 선택한 팔을 disable해야 합니다")
            for side in sides:
                self._require_feedback_ready_locked(side)
            readings = self._calibration_readings_locked(sides, indices)
            if any(
                item["velocity_rad_s"] is None
                or abs(float(item["velocity_rad_s"]))
                > self._calibration_max_velocity
                for item in readings
            ):
                raise RuntimeError("팔이 움직이고 있어 영점을 저장할 수 없습니다")

            self._calibration.update(
                {
                    "status": "writing",
                    "message": "모터 영점 저장 프레임을 전송 중입니다",
                    "ready": False,
                    "verified": False,
                    "results": [],
                }
            )
            self._calibration_feedback_baseline = {
                (side, index): self._motors[side][index].get("last_rx")
                for side in sides
                for index in indices
            }
            try:
                for side in sides:
                    for index in indices:
                        # DaMiao/OpenArm official sequence:
                        # Disable -> Set Zero -> Disable.
                        self._send_simple_locked(side, 0xFD, (index,))
                        self._sleep(self._calibration_step_delay)
                        self._send_simple_locked(side, 0xFE, (index,))
                        self._sleep(self._calibration_step_delay)
                        self._send_simple_locked(side, 0xFD, (index,))
                        self._sleep(self._calibration_step_delay)
                self._refresh_locked(sides)
            except Exception as error:
                self._calibration.update(
                    {
                        "status": "failed",
                        "message": f"영점 저장 CAN 전송 실패: {error}",
                    }
                )
                self._log_event(self._calibration["message"])
                raise

            written_at = time.strftime("%Y-%m-%d %H:%M:%S")
            self._last_calibration = written_at
            self._calibration.update(
                {
                    "status": "awaiting_verification",
                    "message": "영점 저장 완료 · 새 피드백으로 0점을 검증하세요",
                    "selection_key": selection_key,
                    "target_sides": list(sides),
                    "target_joints": [
                        self._specs[index].label for index in indices
                    ],
                    "written_at": written_at,
                }
            )
            description = (
                "전체 8축"
                if len(indices) == len(self._specs)
                else self._specs[indices[0]].label
            )
            self._log_event(
                "공식 3단계 영점 저장: "
                f"{', '.join(sides)} / {description} / 사후 검증 대기"
            )
            calibration = copy.deepcopy(self._calibration)
        return {
            "accepted": True,
            "needs_verification": True,
            "message": calibration["message"],
            "calibration": calibration,
        }

    def verify_zero(self, payload: dict) -> dict:
        sides, indices, selection_key = self._calibration_selection(payload)
        with self._lock:
            self._require_connected(sides)
            if any(self._enabled[side] for side in sides):
                raise RuntimeError("0점 검증 전에 선택한 팔을 disable해야 합니다")
            for side in sides:
                self._require_feedback_ready_locked(side)
            readings = self._calibration_readings_locked(sides, indices)
            results = []
            for item in readings:
                key = (item["side"], item["joint"])
                baseline = self._calibration_feedback_baseline.get(key)
                current_rx = self._motors[item["side"]][item["joint"]].get(
                    "last_rx"
                )
                fresh = baseline is None or (
                    current_rx is not None and float(current_rx) > float(baseline)
                )
                position_ok = (
                    item["position_deg"] is not None
                    and abs(float(item["position_deg"]))
                    <= self._calibration_tolerance_deg
                )
                velocity_ok = (
                    item["velocity_rad_s"] is not None
                    and abs(float(item["velocity_rad_s"]))
                    <= self._calibration_max_velocity
                )
                fault_ok = item["fault_code"] == 0
                passed = fresh and position_ok and velocity_ok and fault_ok
                reasons = []
                if not fresh:
                    reasons.append("영점 저장 후 새 피드백 없음")
                if not position_ok:
                    reasons.append(
                        f'|각도| > {self._calibration_tolerance_deg:.1f}°'
                    )
                if not velocity_ok:
                    reasons.append("관절 움직임 감지")
                if not fault_ok:
                    reasons.append(f'motor fault {item["fault_code"]}')
                results.append(
                    {
                        **item,
                        "fresh_after_write": fresh,
                        "passed": passed,
                        "reason": " · ".join(reasons),
                    }
                )
            verified = all(item["passed"] for item in results)
            verified_at = time.strftime("%Y-%m-%d %H:%M:%S")
            self._calibration.update(
                {
                    "status": "verified" if verified else "failed",
                    "message": (
                        "선택한 관절의 0점 검증을 통과했습니다"
                        if verified
                        else "일부 관절이 0점 검증을 통과하지 못했습니다"
                    ),
                    "ready": False,
                    "verified": verified,
                    "selection_key": selection_key,
                    "target_sides": list(sides),
                    "target_joints": [
                        self._specs[index].label for index in indices
                    ],
                    "verified_at": verified_at,
                    "results": results,
                }
            )
            self._log_event(
                "영점 사후 검증: "
                + ("통과" if verified else "실패")
                + f" ({sum(item['passed'] for item in results)}/{len(results)})"
            )
            calibration = copy.deepcopy(self._calibration)
        return {
            "accepted": True,
            "verified": verified,
            "message": calibration["message"],
            "calibration": calibration,
        }

    def stop(self) -> dict:
        """Best-effort disable for every connected arm.

        This path is also used by the console-wide stop button.  It does not
        require both CAN interfaces to be present so a partial connection can
        still be put into a non-commanding state.
        """
        errors = []
        disabled = []
        with self._lock:
            for side in SIDES:
                if side in self._sockets:
                    try:
                        self._send_simple_locked(side, 0xFD)
                        disabled.append(side)
                    except Exception as error:
                        errors.append(f"{side}: {error}")
                self._enabled[side] = False
                self._interlocks[side] = ""
                self._desired[side] = None
                self._commanded[side] = None
                self._trajectories[side] = None
            if errors:
                self._last_error = "; ".join(errors)
            self._log_event(
                "전역 정지: OpenArm 모터 disable"
                + (f" (전송 오류: {self._last_error})" if errors else "")
            )
        message = "연결된 OpenArm 모터를 disable했습니다"
        if not disabled:
            message = "OpenArm CAN이 연결되지 않아 disable할 모터가 없습니다"
        if errors:
            message += "; 일부 CAN 전송 오류를 확인하세요"
        return {
            "accepted": not errors,
            "message": message,
            "disabled_sides": disabled,
            "errors": errors,
        }

    def action(self, action: str, payload: dict | None = None) -> dict:
        payload = payload or {}
        actions = {
            "connect": self.connect,
            "disconnect": lambda _payload: self.disconnect(),
            "refresh": self.refresh,
            "enable": self.set_enabled,
            "hold": self.hold_current,
            "joint": self.set_joint,
            "pose": self.set_pose,
            "limits": self.set_joint_limits,
            "calibration-check": self.calibration_preflight,
            "calibrate-zero": self.calibrate_zero,
            "verify-zero": self.verify_zero,
            "clear-errors": self.clear_errors,
            "stop": lambda _payload: self.stop(),
        }
        handler = actions.get(action)
        if handler is None:
            raise ValueError(f"알 수 없는 OpenArm 작업입니다: {action}")
        return handler(payload)

    def _send_pose_locked(
        self,
        side: str,
        positions: list[float],
        velocities: list[float] | None = None,
    ) -> None:
        velocities = velocities or [0.0] * len(self._specs)
        for spec, position, velocity in zip(self._specs, positions, velocities):
            data = pack_mit_data(
                spec.motor_type,
                position,
                spec.kp * self._gain_scale,
                spec.kd * self._gain_scale,
                velocity_rad_s=_clamp(
                    velocity,
                    -spec.velocity_limit_rad_s,
                    spec.velocity_limit_rad_s,
                ),
            )
            self._send_locked(side, spec.send_id, data)

    def _plan_trajectory_locked(self, side: str, targets: list[float]) -> None:
        start = self._commanded[side]
        if start is None:
            start = self._current_positions_locked(side)
        start = list(start)
        target = list(targets)
        requested_velocity = math.radians(self._speed_deg_s)
        durations = []
        for spec, current, goal in zip(self._specs, start, target):
            distance = abs(goal - current)
            velocity_limit = max(
                1e-6, min(requested_velocity, spec.velocity_limit_rad_s)
            )
            # The derivative of quintic smoothstep peaks at 1.875.  Scaling
            # duration by that factor guarantees every joint remains below its
            # requested and hardware velocity ceiling.
            durations.append(1.875 * distance / velocity_limit)
        duration = max(durations, default=0.0)
        if duration <= 1e-9:
            self._commanded[side] = target
            self._trajectories[side] = None
            return
        self._trajectories[side] = {
            "start_time": self._clock(),
            "duration": max(duration, self._minimum_trajectory_duration),
            "start": start,
            "target": target,
        }

    @staticmethod
    def _sample_trajectory(trajectory: dict, now: float) -> tuple[list[float], list[float], bool]:
        duration = float(trajectory["duration"])
        progress = _clamp((now - float(trajectory["start_time"])) / duration, 0.0, 1.0)
        # Quintic smoothstep has continuous position, velocity and acceleration
        # with zero velocity/acceleration at both ends.
        blend = progress**3 * (10.0 - 15.0 * progress + 6.0 * progress**2)
        blend_rate = (
            30.0 * progress**2 * (1.0 - progress) ** 2 / duration
        )
        positions = []
        velocities = []
        for start, target in zip(trajectory["start"], trajectory["target"]):
            delta = target - start
            positions.append(start + delta * blend)
            velocities.append(delta * blend_rate)
        complete = now >= float(trajectory["start_time"]) + duration - 1e-9
        return positions, velocities, complete

    def _command_step_locked(self, now: float) -> None:
        self._last_command = now
        for side in SIDES:
            desired = self._desired[side]
            if not self._enabled[side] or desired is None:
                continue
            problem = self._feedback_problem_locked(side)
            if problem:
                if not self._interlocks[side]:
                    self._interlocks[side] = problem
                    self._log_event(f"{side} 팔 명령 인터록: {problem}")
                continue
            if self._interlocks[side]:
                continue
            trajectory = self._trajectories[side]
            if trajectory is None:
                next_values = self._commanded[side]
                if next_values is None:
                    next_values = self._current_positions_locked(side)
                velocities = [0.0] * len(self._specs)
            else:
                next_values, velocities, complete = self._sample_trajectory(
                    trajectory, now
                )
                if complete:
                    self._trajectories[side] = None
            self._commanded[side] = next_values
            self._send_pose_locked(side, next_values, velocities)
            for motor, target in zip(self._motors[side], next_values):
                motor["target_rad"] = target

    def _handle_frame(self, side: str, frame: bytes) -> None:
        can_id, data = unpack_can_frame(frame)
        index = next(
            (i for i, spec in enumerate(self._specs) if spec.receive_id == can_id),
            None,
        )
        if index is None or len(data) < 8:
            return
        decoded = decode_motor_feedback(self._specs[index], data)
        with self._lock:
            self._motors[side][index].update(decoded)
            self._motors[side][index]["last_rx"] = self._clock()
            self._rx_count[side] += 1

    def _drain_socket(self, side: str, can_socket) -> None:
        while True:
            try:
                frame = can_socket.recv(CAN_FRAME.size)
            except (BlockingIOError, InterruptedError):
                return
            if not frame:
                return
            self._handle_frame(side, frame)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                socket_by_object = {value: key for key, value in self._sockets.items()}
            if socket_by_object:
                try:
                    poll_timeout = min(0.01, 0.5 / self._command_rate_hz)
                    ready, _, _ = select.select(
                        list(socket_by_object), [], [], poll_timeout
                    )
                    for can_socket in ready:
                        self._drain_socket(socket_by_object[can_socket], can_socket)
                except (OSError, ValueError) as error:
                    with self._lock:
                        self._last_error = str(error)
            else:
                self._stop_event.wait(0.03)
            now = self._clock()
            with self._lock:
                try:
                    if self._sockets and now - self._last_refresh >= 1.0 / self._refresh_rate_hz:
                        # MIT commands normally produce feedback while a side is
                        # enabled.  Once its command path is interlocked, no MIT
                        # frames are sent, so keep polling status explicitly to
                        # prevent a real fault from also appearing as stale CAN.
                        refresh_sides = tuple(
                            side
                            for side in SIDES
                            if side in self._sockets
                            and (not self._enabled[side] or bool(self._interlocks[side]))
                        )
                        if refresh_sides:
                            self._refresh_locked(refresh_sides)
                    if now - self._last_command >= 1.0 / self._command_rate_hz:
                        self._command_step_locked(now)
                except Exception as error:
                    self._last_error = str(error)
                    self._log_event(f"CAN 제어 오류: {error}")

    def _available_interfaces(self) -> list[dict]:
        result = []
        root = pathlib.Path("/sys/class/net")
        try:
            names = sorted(path.name for path in root.iterdir())
        except OSError:
            names = []
        for name in names:
            info = self._interface_probe(name)
            if info.get("is_can"):
                result.append(info)
        return result

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            arms = {}
            for side in SIDES:
                motors = []
                online_count = 0
                fault_count = 0
                temperatures = []
                feedback_ages = []
                for index, motor in enumerate(self._motors[side]):
                    item = dict(motor)
                    spec = self._specs[index]
                    soft_lower, soft_upper = self._soft_bounds_locked(side, index)
                    item["lower_deg"] = round(math.degrees(soft_lower), 3)
                    item["upper_deg"] = round(math.degrees(soft_upper), 3)
                    item["hard_lower_deg"] = round(spec.lower_deg, 3)
                    item["hard_upper_deg"] = round(spec.upper_deg, 3)
                    item["velocity_limit_deg_s"] = round(
                        math.degrees(spec.velocity_limit_rad_s), 3
                    )
                    last_rx = item.pop("last_rx")
                    age = None if last_rx is None else max(0.0, now - float(last_rx))
                    online = age is not None and age <= 1.0
                    online_count += int(online)
                    fault_count += int(int(item.get("fault_code") or 0) != 0)
                    feedback_ages.append(age)
                    temperatures.extend(
                        float(value)
                        for value in (item.get("mos_temp_c"), item.get("rotor_temp_c"))
                        if value is not None
                    )
                    item["online"] = online
                    item["age_sec"] = None if age is None else round(age, 3)
                    item["position_deg"] = (
                        None
                        if item["position_rad"] is None
                        else round(math.degrees(float(item["position_rad"])), 3)
                    )
                    item["target_deg"] = (
                        None
                        if item["target_rad"] is None
                        else round(math.degrees(float(item["target_rad"])), 3)
                    )
                    item.pop("position_rad", None)
                    item.pop("target_rad", None)
                    for key in ("velocity_rad_s", "torque_nm"):
                        if item[key] is not None:
                            item[key] = round(float(item[key]), 4)
                    motors.append(item)
                connected = side in self._sockets
                enabled = self._enabled[side]
                interlock = self._interlocks[side]
                trajectory = self._trajectories[side]
                if trajectory is None:
                    trajectory_progress = 1.0
                    trajectory_remaining = 0.0
                else:
                    elapsed = max(0.0, now - float(trajectory["start_time"]))
                    duration = float(trajectory["duration"])
                    trajectory_progress = _clamp(elapsed / duration, 0.0, 1.0)
                    trajectory_remaining = max(0.0, duration - elapsed)
                arms[side] = {
                    "interface": self._interfaces[side],
                    "connected": connected,
                    "enabled": enabled,
                    "online_count": online_count,
                    "all_online": online_count == len(self._specs),
                    "stale_count": len(self._specs) - online_count,
                    "fault_count": fault_count,
                    "max_temp_c": round(max(temperatures), 1) if temperatures else None,
                    "oldest_feedback_age_sec": (
                        None
                        if any(age is None for age in feedback_ages)
                        else round(max(feedback_ages), 3)
                    ),
                    "command_interlocked": bool(interlock),
                    "interlock_reason": interlock,
                    "trajectory_active": trajectory is not None,
                    "trajectory_progress": round(trajectory_progress, 4),
                    "trajectory_remaining_sec": round(trajectory_remaining, 3),
                    "control_ready": connected
                    and enabled
                    and online_count == len(self._specs)
                    and fault_count == 0
                    and not interlock,
                    "rx_count": self._rx_count[side],
                    "tx_count": self._tx_count[side],
                    "motors": motors,
                }
            connected = len(self._sockets) == len(SIDES)
            state = {
                "backend": "내장 Classic SocketCAN",
                "library_version": "openarm_can 1.2.9 protocol",
                "connected": connected,
                "any_enabled": any(self._enabled.values()),
                "all_enabled": all(self._enabled.values()),
                "command_rate_hz": self._command_rate_hz,
                "trajectory_profile": "quintic_s_curve",
                "minimum_trajectory_duration_sec": self._minimum_trajectory_duration,
                "target_speed_deg_s": self._speed_deg_s,
                "gain_scale": self._gain_scale,
                "maximum_target_speed_deg_s": self._max_speed_deg_s,
                "maximum_gain_scale": self._max_gain_scale,
                "last_error": self._last_error,
                "last_calibration": self._last_calibration,
                "calibration": copy.deepcopy(self._calibration),
                "arms": arms,
                "log": list(reversed(self._log))[:40],
            }
        state["interfaces"] = {
            side: self._interface_probe(self._interfaces[side]) for side in SIDES
        }
        issues = []
        for side in SIDES:
            label = "왼팔" if side == "left" else "오른팔"
            arm = state["arms"][side]
            stats = state["interfaces"][side].get("statistics", {})
            can_errors = int(stats.get("rx_errors", 0)) + int(stats.get("tx_errors", 0))
            arm["can_error_count"] = can_errors
            if arm["connected"] and arm["stale_count"]:
                stale_labels = ", ".join(
                    motor["label"] for motor in arm["motors"] if not motor["online"]
                )
                issues.append(f"{label} 피드백 지연: {stale_labels}")
            if arm["fault_count"]:
                issues.append(f'{label} motor fault {arm["fault_count"]}축')
            if can_errors:
                issues.append(f"{label} SocketCAN 오류 {can_errors}건")
            if arm["command_interlocked"]:
                issues.append(f'{label} 명령 차단: {arm["interlock_reason"]}')
        if not state["connected"]:
            health_level = "DISCONNECTED"
        elif any(arm["command_interlocked"] for arm in state["arms"].values()):
            health_level = "BLOCKED"
        elif issues:
            health_level = "WARNING"
        elif state["all_enabled"]:
            health_level = "READY"
        else:
            health_level = "STANDBY"
        state["health_level"] = health_level
        state["issues"] = issues
        state["calibration"]["automatic_tool"] = (
            self._automatic_calibration_tool()
        )
        state["all_control_ready"] = all(
            arm["control_ready"] for arm in state["arms"].values()
        )
        state["available_interfaces"] = self._available_interfaces()
        return state

    def shutdown(self) -> None:
        self._stop_event.set()
        try:
            self.disconnect()
        finally:
            if self._worker is not None:
                self._worker.join(timeout=1.0)
