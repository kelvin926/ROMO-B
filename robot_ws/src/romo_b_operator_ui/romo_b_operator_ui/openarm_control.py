"""OpenArm-v1 bimanual SocketCAN control for the browser operator console.

The controller is deliberately independent from ExoArm, ROS topics, cameras,
and learned policies.  Opening the CAN sockets does not enable a motor.  Motor
commands become possible only after explicit feedback, enable, and target
requests from the operator page.
"""

from __future__ import annotations

import math
import pathlib
import re
import select
import socket
import struct
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
    return {
        "position_rad": _uint_to_double(q_uint, -12.5, 12.5, 16),
        "velocity_rad_s": _uint_to_double(
            dq_uint, -velocity_max, velocity_max, 12
        ),
        "torque_nm": _uint_to_double(tau_uint, -torque_max, torque_max, 12),
        "mos_temp_c": int(data[6]),
        "rotor_temp_c": int(data[7]),
        "status_byte": int(data[0]),
        "fault_code": int((data[0] >> 4) & 0xF),
    }


class OpenArmController:
    """Own two Classic-CAN sockets and a bounded position-control loop."""

    def __init__(
        self,
        repo_root: pathlib.Path,
        *,
        socket_factory: Callable[[str], object] | None = None,
        interface_probe: Callable[[str], dict] | None = None,
        clock: Callable[[], float] = time.monotonic,
        start_worker: bool = True,
    ):
        self.repo_root = pathlib.Path(repo_root).resolve()
        self._clock = clock
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._socket_factory = socket_factory or self._open_socket
        self._interface_probe = interface_probe or self._read_interface
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
        control = self._config["operator_control"]
        self._command_rate_hz = float(control["command_rate_hz"])
        self._refresh_rate_hz = float(control["refresh_rate_hz"])
        self._speed_deg_s = float(control["default_target_speed_deg_s"])
        self._max_speed_deg_s = float(control["maximum_target_speed_deg_s"])
        self._gain_scale = 1.0
        self._max_gain_scale = float(control["maximum_gain_scale"])
        self._sockets: dict[str, object] = {}
        self._enabled = {side: False for side in SIDES}
        self._desired: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._commanded: dict[str, list[float] | None] = {side: None for side in SIDES}
        self._motors = {
            side: [self._empty_motor(spec) for spec in self._specs] for side in SIDES
        }
        self._rx_count = {side: 0 for side in SIDES}
        self._tx_count = {side: 0 for side in SIDES}
        self._last_error = ""
        self._last_calibration = ""
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
            specs.append(
                JointSpec(
                    key=key,
                    label=f"J{index + 1}",
                    motor_type=str(source["type"]),
                    send_id=int(source["send_id"]),
                    receive_id=int(source["receive_id"]),
                    lower_rad=float(limits["lower"]),
                    upper_rad=float(limits["upper"]),
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
            "kp": spec.kp,
            "kd": spec.kd,
            "position_rad": None,
            "velocity_rad_s": None,
            "torque_nm": None,
            "mos_temp_c": None,
            "rotor_temp_c": None,
            "status_byte": None,
            "fault_code": None,
            "last_rx": None,
            "target_rad": None,
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

    def _current_positions_locked(self, side: str) -> list[float]:
        now = self._clock()
        if not all(self._motor_is_online(motor, now, 1.5) for motor in self._motors[side]):
            raise RuntimeError(
                f"{side} 팔의 8개 모터 피드백을 먼저 모두 확인해야 합니다"
            )
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
            for side, name in requested.items():
                info = self._interface_probe(name)
                if not info.get("exists") or not info.get("is_can"):
                    raise RuntimeError(f"{name}은 사용 가능한 SocketCAN 장치가 아닙니다")
                if not info.get("up"):
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
            self._desired = {side: None for side in SIDES}
            self._commanded = {side: None for side in SIDES}
            self._last_error = ""
            self._log_event(f"CAN 연결: 왼팔 {left}, 오른팔 {right}")
            self._refresh_locked(SIDES)
        return {"accepted": True, "message": "OpenArm 양팔 CAN을 연결했습니다"}

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
            self._desired = {side: None for side in SIDES}
            self._commanded = {side: None for side in SIDES}
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
                    self._send_simple_locked(side, 0xFC)
                    self._enabled[side] = True
                    self._send_pose_locked(side, current[side])
                self._log_event("현재 자세 유지로 모터 enable: " + ", ".join(sides))
                return {"accepted": True, "message": "현재 자세를 유지하며 모터를 enable했습니다"}
            for side in sides:
                self._send_simple_locked(side, 0xFD)
                self._enabled[side] = False
                self._desired[side] = None
                self._commanded[side] = None
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
            self._set_options_locked(payload)
            if self._desired[side] is None:
                self._desired[side] = self._current_positions_locked(side)
            spec = self._specs[index]
            target_rad = _clamp(math.radians(target_deg), spec.lower_rad, spec.upper_rad)
            self._desired[side][index] = target_rad
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
        prepared = {}
        for side in requested_sides:
            values = targets[side]
            if not isinstance(values, list) or len(values) != len(self._specs):
                raise ValueError(f"{side} 목표는 8개 각도 배열이어야 합니다")
            converted = []
            for spec, raw in zip(self._specs, values):
                value = float(raw)
                if not math.isfinite(value):
                    raise ValueError("모든 목표 각도는 유한한 숫자여야 합니다")
                converted.append(
                    _clamp(math.radians(value), spec.lower_rad, spec.upper_rad)
                )
            prepared[side] = converted
        with self._lock:
            self._require_connected(requested_sides)
            for side in requested_sides:
                if not self._enabled[side]:
                    raise RuntimeError(f"{side} 팔 모터가 enable되지 않았습니다")
            self._set_options_locked(payload)
            for side, values in prepared.items():
                self._desired[side] = list(values)
                if self._commanded[side] is None:
                    self._commanded[side] = self._current_positions_locked(side)
            self._log_event("양팔 자세 목표 반영: " + ", ".join(requested_sides))
        return {"accepted": True, "message": "선택한 팔의 8축 목표 자세를 적용했습니다"}

    def calibrate_zero(self, payload: dict) -> dict:
        sides = self._side_list(payload.get("side"))
        if payload.get("confirmation") != "OPENARM ZERO":
            raise ValueError("영점 저장 확인 문구가 일치하지 않습니다")
        joint = payload.get("joint")
        indices = None
        if joint is not None:
            index = int(joint)
            if not 0 <= index < len(self._specs):
                raise ValueError("joint 번호는 0..7 범위여야 합니다")
            indices = (index,)
        with self._lock:
            self._require_connected(sides)
            if any(self._enabled[side] for side in sides):
                raise RuntimeError("영점 저장 전에 선택한 팔을 disable해야 합니다")
            for side in sides:
                self._current_positions_locked(side)
                self._send_simple_locked(side, 0xFE, indices)
            description = "전체 관절" if indices is None else self._specs[indices[0]].label
            self._last_calibration = time.strftime("%Y-%m-%d %H:%M:%S")
            self._log_event(
                f"현재 자세 영점 저장: {', '.join(sides)} / {description}"
            )
        return {"accepted": True, "message": "현재 자세를 모터 영점으로 저장했습니다"}

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
            "calibrate-zero": self.calibrate_zero,
            "clear-errors": self.clear_errors,
            "stop": lambda _payload: self.set_enabled(
                {"side": "both", "enabled": False}
            ),
        }
        handler = actions.get(action)
        if handler is None:
            raise ValueError(f"알 수 없는 OpenArm 작업입니다: {action}")
        return handler(payload)

    def _send_pose_locked(self, side: str, positions: list[float]) -> None:
        for spec, position in zip(self._specs, positions):
            data = pack_mit_data(
                spec.motor_type,
                position,
                spec.kp * self._gain_scale,
                spec.kd * self._gain_scale,
            )
            self._send_locked(side, spec.send_id, data)

    def _command_step_locked(self, now: float) -> None:
        elapsed = _clamp(now - self._last_command, 0.0, 0.1)
        self._last_command = now
        max_delta = math.radians(self._speed_deg_s) * elapsed
        for side in SIDES:
            desired = self._desired[side]
            if not self._enabled[side] or desired is None:
                continue
            commanded = self._commanded[side]
            if commanded is None:
                commanded = self._current_positions_locked(side)
            next_values = []
            for current, target in zip(commanded, desired):
                delta = _clamp(target - current, -max_delta, max_delta)
                next_values.append(current + delta)
            self._commanded[side] = next_values
            self._send_pose_locked(side, next_values)
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
                    ready, _, _ = select.select(list(socket_by_object), [], [], 0.015)
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
                        idle_sides = tuple(
                            side for side in SIDES if side in self._sockets and not self._enabled[side]
                        )
                        if idle_sides:
                            self._refresh_locked(idle_sides)
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
                for motor in self._motors[side]:
                    item = dict(motor)
                    last_rx = item.pop("last_rx")
                    age = None if last_rx is None else max(0.0, now - float(last_rx))
                    online = age is not None and age <= 1.0
                    online_count += int(online)
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
                arms[side] = {
                    "interface": self._interfaces[side],
                    "connected": side in self._sockets,
                    "enabled": self._enabled[side],
                    "online_count": online_count,
                    "all_online": online_count == len(self._specs),
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
                "target_speed_deg_s": self._speed_deg_s,
                "gain_scale": self._gain_scale,
                "maximum_target_speed_deg_s": self._max_speed_deg_s,
                "maximum_gain_scale": self._max_gain_scale,
                "last_error": self._last_error,
                "last_calibration": self._last_calibration,
                "arms": arms,
                "log": list(reversed(self._log))[:40],
            }
        state["interfaces"] = {
            side: self._interface_probe(self._interfaces[side]) for side in SIDES
        }
        state["available_interfaces"] = self._available_interfaces()
        return state

    def shutdown(self) -> None:
        self._stop_event.set()
        try:
            self.disconnect()
        finally:
            if self._worker is not None:
                self._worker.join(timeout=1.0)
