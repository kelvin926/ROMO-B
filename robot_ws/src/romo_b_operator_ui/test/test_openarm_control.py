import pathlib

import pytest
import yaml

from romo_b_operator_ui.openarm_control import (
    CAN_FRAME,
    OpenArmController,
    pack_can_frame,
    pack_mit_data,
    unpack_can_frame,
)


class FakeSocket:
    def __init__(self, name):
        self.name = name
        self.frames = []
        self.closed = False

    def send(self, frame):
        self.frames.append(frame)
        return len(frame)

    def recv(self, _size):
        raise BlockingIOError

    def close(self):
        self.closed = True


def make_repo(tmp_path: pathlib.Path) -> pathlib.Path:
    config = tmp_path / "openarm" / "config"
    config.mkdir(parents=True)
    (config / "openarm_v1_bimanual.yaml").write_text(
        yaml.safe_dump(
            {
                "interfaces": {"left": {"name": "can1"}, "right": {"name": "can0"}},
                "motors": {
                    **{
                        f"joint_{index}": {
                            "type": "DM8009" if index < 3 else "DM4340" if index < 5 else "DM4310",
                            "send_id": index,
                            "receive_id": index + 16,
                        }
                        for index in range(1, 8)
                    },
                    "gripper": {"type": "DM4310", "send_id": 8, "receive_id": 24},
                },
                "operator_control": {
                    "command_rate_hz": 30.0,
                    "refresh_rate_hz": 5.0,
                    "default_target_speed_deg_s": 15.0,
                    "maximum_target_speed_deg_s": 90.0,
                    "maximum_gain_scale": 1.5,
                    "arm_kp": [20, 20, 12, 12, 5, 5, 4],
                    "arm_kd": [1, 1, 0.8, 0.8, 0.3, 0.3, 0.3],
                    "gripper_kp": 5.0,
                    "gripper_kd": 0.3,
                    "gripper_limit_deg": [-60, 0],
                },
            }
        )
    )
    (config / "joint_limits.yaml").write_text(
        yaml.safe_dump(
            {
                f"joint{index}": {
                    "limit": {"lower": -1.0, "upper": 1.0}
                }
                for index in range(1, 8)
            }
        )
    )
    return tmp_path


def make_controller(tmp_path):
    sockets = {}
    now = [10.0]

    def factory(name):
        sockets[name] = FakeSocket(name)
        return sockets[name]

    controller = OpenArmController(
        make_repo(tmp_path),
        socket_factory=factory,
        interface_probe=lambda name: {
            "name": name,
            "exists": True,
            "is_can": True,
            "up": True,
            "operstate": "unknown",
        },
        clock=lambda: now[0],
        start_worker=False,
    )
    return controller, sockets, now


def feed_all_motors(controller):
    neutral = bytes((0, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 31, 29))
    for side in ("left", "right"):
        for can_id in range(17, 25):
            controller._handle_frame(side, pack_can_frame(can_id, neutral))


def test_classic_can_and_mit_packets_match_openarm_protocol():
    payload = pack_mit_data("DM4310", 0.0, 0.0, 0.0)
    assert payload == bytes.fromhex("7f ff 7f f0 00 00 07 ff")

    packed = pack_can_frame(0x123, payload)
    assert len(packed) == CAN_FRAME.size
    assert unpack_can_frame(packed) == (0x123, payload)


def test_connect_feedback_enable_and_bounded_joint_command(tmp_path):
    controller, sockets, now = make_controller(tmp_path)
    result = controller.action(
        "connect", {"left_interface": "can1", "right_interface": "can0"}
    )
    assert result["accepted"] is True
    assert len(sockets["can1"].frames) == 8
    assert len(sockets["can0"].frames) == 8

    feed_all_motors(controller)
    state = controller.snapshot()
    assert state["arms"]["left"]["all_online"] is True
    assert state["arms"]["right"]["all_online"] is True

    controller.action("enable", {"side": "both", "enabled": True})
    assert controller.snapshot()["all_enabled"] is True
    controller.action(
        "joint",
        {"side": "left", "joint": 0, "target_deg": 45.0, "speed_deg_s": 15.0},
    )
    now[0] += 0.1
    with controller._lock:
        controller._command_step_locked(now[0])
    commanded = controller.snapshot()["arms"]["left"]["motors"][0]["target_deg"]
    assert commanded == pytest.approx(1.5, abs=0.02)

    controller.shutdown()
    assert sockets["can1"].closed is True
    assert sockets["can0"].closed is True


def test_zero_calibration_requires_disable_and_confirmation(tmp_path):
    controller, sockets, _now = make_controller(tmp_path)
    controller.action("connect", {})
    feed_all_motors(controller)
    controller.action("enable", {"side": "left", "enabled": True})

    with pytest.raises(RuntimeError, match="disable"):
        controller.action(
            "calibrate-zero",
            {"side": "left", "confirmation": "OPENARM ZERO"},
        )
    controller.action("enable", {"side": "left", "enabled": False})
    with pytest.raises(ValueError, match="확인 문구"):
        controller.action(
            "calibrate-zero", {"side": "left", "confirmation": "yes"}
        )

    before = len(sockets["can1"].frames)
    result = controller.action(
        "calibrate-zero",
        {"side": "left", "joint": 7, "confirmation": "OPENARM ZERO"},
    )
    assert result["accepted"] is True
    _can_id, payload = unpack_can_frame(sockets["can1"].frames[-1])
    assert len(sockets["can1"].frames) == before + 1
    assert payload[-1] == 0xFE
    controller.shutdown()


def test_unknown_action_and_invalid_interface_are_rejected(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)
    with pytest.raises(ValueError, match="알 수 없는 OpenArm"):
        controller.action("shell")
    with pytest.raises(ValueError, match="서로 다른"):
        controller.action(
            "connect", {"left_interface": "can0", "right_interface": "can0"}
        )
    controller.shutdown()
