import math
import pathlib

import pytest
import yaml

from romo_b_operator_ui.openarm_control import (
    CAN_FRAME,
    OpenArmController,
    decode_motor_feedback,
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
                "operator_coordinates": {
                    "convention": "openarm_v1_native_joint",
                    "direction_signs": {
                        "left": [1, 1, 1, 1, 1, 1, 1, 1],
                        "right": [1, 1, 1, 1, 1, 1, 1, 1],
                    },
                    "hard_limits_deg": {
                        "left": {
                            "joint_1": [-120.0, 60.0],
                            "joint_2": [-110.0, 10.0],
                        },
                        "right": {
                            "joint_1": [-60.0, 120.0],
                            "joint_2": [-10.0, 110.0],
                        },
                    },
                },
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
                    "command_rate_hz": 100.0,
                    "refresh_rate_hz": 5.0,
                    "default_target_speed_deg_s": 30.0,
                    "maximum_target_speed_deg_s": 2000.0,
                    "gripper_velocity_limit_rad_s": 30.0,
                    "minimum_trajectory_duration_sec": 0.12,
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
                    "limit": {
                        "lower": -1.0,
                        "upper": 2.0 if index == 1 else 1.0,
                        "velocity": 2.0,
                    }
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
        sleep=lambda _duration: None,
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


def test_enabled_status_code_is_normal_and_fault_range_starts_at_eight(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)
    spec = controller._specs[0]
    enabled = bytes((0x11, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 31, 29))
    faulted = bytes((0x81, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 31, 29))

    enabled_state = decode_motor_feedback(spec, enabled)
    faulted_state = decode_motor_feedback(spec, faulted)

    assert enabled_state["state_name"] == "ENABLED"
    assert enabled_state["fault_code"] == 0
    assert faulted_state["state_name"] == "OVER_VOLTAGE"
    assert faulted_state["fault_code"] == 8
    controller.shutdown()


def test_connect_feedback_enable_and_smooth_joint_command(tmp_path):
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
    trajectory = controller._trajectories["left"]
    assert trajectory["duration"] == pytest.approx(5.625, abs=0.002)
    start_position, start_velocity, start_done = controller._sample_trajectory(
        trajectory, now[0]
    )
    midpoint = now[0] + trajectory["duration"] / 2.0
    middle_position, middle_velocity, middle_done = controller._sample_trajectory(
        trajectory, midpoint
    )
    end_position, end_velocity, end_done = controller._sample_trajectory(
        trajectory, now[0] + trajectory["duration"]
    )
    assert math.degrees(start_position[0]) == pytest.approx(0.0, abs=0.02)
    assert start_velocity[0] == pytest.approx(0.0)
    assert start_done is False
    assert math.degrees(middle_position[0]) == pytest.approx(22.5, abs=0.02)
    assert math.degrees(middle_velocity[0]) == pytest.approx(15.0, abs=0.02)
    assert middle_done is False
    assert math.degrees(end_position[0]) == pytest.approx(45.0, abs=0.02)
    assert end_velocity[0] == pytest.approx(0.0)
    assert end_done is True

    now[0] = midpoint
    feed_all_motors(controller)
    with controller._lock:
        controller._command_step_locked(now[0])
    commanded = controller.snapshot()["arms"]["left"]["motors"][0]["target_deg"]
    assert commanded == pytest.approx(22.5, abs=0.02)

    controller.shutdown()
    assert sockets["can1"].closed is True
    assert sockets["can0"].closed is True


def test_per_arm_soft_limits_persist_and_do_not_start_motion(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)
    result = controller.action(
        "limits",
        {"side": "left", "joint": 0, "lower_deg": -20.0, "upper_deg": 35.0},
    )
    assert result["accepted"] is True
    state = controller.snapshot()
    left_joint = state["arms"]["left"]["motors"][0]
    right_joint = state["arms"]["right"]["motors"][0]
    assert left_joint["lower_deg"] == pytest.approx(-20.0)
    assert left_joint["upper_deg"] == pytest.approx(35.0)
    assert right_joint["lower_deg"] == pytest.approx(-60.0, abs=0.001)
    assert controller._trajectories["left"] is None
    assert (tmp_path / "config/local/openarm_operator_joint_limits.yaml").is_file()

    with pytest.raises(ValueError, match="하드 한계"):
        controller.action(
            "limits",
            {"side": "left", "joint": 0, "lower_deg": -130.0, "upper_deg": 35.0},
        )

    restored = controller.action(
        "limits", {"side": "left", "joint": 0, "reset": True}
    )
    assert restored["accepted"] is True
    state = controller.snapshot()
    assert state["arms"]["left"]["motors"][0]["lower_deg"] == pytest.approx(-120.0)
    controller.shutdown()


def test_openarm_v1_native_coordinates_keep_sign_and_use_side_limits(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)
    controller.action("connect", {})
    feed_all_motors(controller)

    with controller._lock:
        controller._motors["left"][0].update(
            {"position_rad": 0.4, "velocity_rad_s": 0.2, "torque_nm": 0.3}
        )
        controller._motors["right"][0].update(
            {"position_rad": 0.4, "velocity_rad_s": 0.2, "torque_nm": 0.3}
        )
        controller._motors["right"][7].update(
            {"position_rad": -0.4, "velocity_rad_s": 0.2, "torque_nm": 0.3}
        )

    state = controller.snapshot()
    left_j1 = state["arms"]["left"]["motors"][0]
    right_j1 = state["arms"]["right"]["motors"][0]
    right_gripper = state["arms"]["right"]["motors"][7]
    assert left_j1["position_deg"] == pytest.approx(math.degrees(0.4), abs=0.001)
    assert right_j1["position_deg"] == pytest.approx(math.degrees(0.4), abs=0.001)
    assert right_j1["velocity_rad_s"] == pytest.approx(0.2)
    assert right_j1["torque_nm"] == pytest.approx(0.3)
    assert right_j1["direction_sign"] == 1
    assert left_j1["hard_lower_deg"] == pytest.approx(-120.0)
    assert left_j1["hard_upper_deg"] == pytest.approx(60.0)
    assert right_j1["hard_lower_deg"] == pytest.approx(-60.0)
    assert right_j1["hard_upper_deg"] == pytest.approx(120.0)
    assert right_gripper["position_deg"] == pytest.approx(
        -math.degrees(0.4), abs=0.001
    )
    assert right_gripper["velocity_rad_s"] == pytest.approx(0.2)
    assert right_gripper["torque_nm"] == pytest.approx(0.3)
    assert right_gripper["direction_sign"] == 1

    controller.action("enable", {"side": "right", "enabled": True})
    controller.action(
        "joint", {"side": "right", "joint": 0, "target_deg": 30.0}
    )
    assert math.degrees(controller._desired["right"][0]) == pytest.approx(30.0)
    controller.action(
        "joint", {"side": "right", "joint": 7, "target_deg": -30.0}
    )
    assert math.degrees(controller._desired["right"][7]) == pytest.approx(-30.0)
    controller.action(
        "pose",
        {"targets": {"right": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, -20.0]}},
    )
    assert [math.degrees(value) for value in controller._desired["right"][:7]] == pytest.approx(
        [10.0] * 7
    )
    assert math.degrees(controller._desired["right"][7]) == pytest.approx(-20.0)
    assert state["operator_coordinates"]["direction_signs"]["right"] == [1] * 8
    controller.shutdown()


def test_connect_prepares_down_can_links_when_enabled(tmp_path):
    repo = make_repo(tmp_path)
    config_path = repo / "openarm" / "config" / "openarm_v1_bimanual.yaml"
    config = yaml.safe_load(config_path.read_text())
    config["startup_policy"] = {"configure_interfaces_automatically": True}
    config_path.write_text(yaml.safe_dump(config))
    link_up = {"can0": False, "can1": False}
    prepared = []

    def prepare(names):
        prepared.append(names)
        for name in names:
            link_up[name] = True

    controller = OpenArmController(
        repo,
        socket_factory=FakeSocket,
        interface_probe=lambda name: {
            "name": name,
            "exists": True,
            "is_can": True,
            "up": link_up[name],
            "operstate": "up" if link_up[name] else "down",
        },
        interface_preparer=prepare,
        start_worker=False,
    )
    result = controller.action("connect", {})
    assert result["accepted"] is True
    assert "1 Mbps" in result["message"]
    assert prepared == [("can1", "can0")]
    controller.shutdown()


def test_zero_calibration_requires_disable_and_confirmation(tmp_path):
    controller, sockets, now = make_controller(tmp_path)
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
    assert result["needs_verification"] is True
    command_frames = sockets["can1"].frames[before : before + 3]
    decoded = [unpack_can_frame(frame) for frame in command_frames]
    assert [can_id for can_id, _payload in decoded] == [8, 8, 8]
    assert [payload[-1] for _can_id, payload in decoded] == [0xFD, 0xFE, 0xFD]
    # The official write sequence is followed by eight status-refresh frames.
    assert len(sockets["can1"].frames) == before + 11

    now[0] += 0.1
    feed_all_motors(controller)
    verified = controller.action("verify-zero", {"side": "left", "joint": 7})
    assert verified["verified"] is True
    assert verified["calibration"]["status"] == "verified"
    assert verified["calibration"]["results"][0]["passed"] is True
    controller.shutdown()


def test_zero_calibration_preflight_reports_disconnected_state(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)

    result = controller.action("calibration-check", {"side": "right"})

    assert result["accepted"] is True
    assert result["ready"] is False
    assert result["calibration"]["status"] == "blocked"
    assert "CAN 미연결" in result["message"]
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


def test_fault_feedback_blocks_enable_and_is_reported(tmp_path):
    controller, _sockets, _now = make_controller(tmp_path)
    controller.action("connect", {})
    feed_all_motors(controller)
    fault = bytes((0x80, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 41, 39))
    controller._handle_frame("left", pack_can_frame(17, fault))

    with pytest.raises(RuntimeError, match="모터 fault"):
        controller.action("enable", {"side": "left", "enabled": True})

    state = controller.snapshot()
    assert state["arms"]["left"]["fault_count"] == 1
    assert state["health_level"] == "WARNING"
    assert any("왼팔 motor fault" in issue for issue in state["issues"])
    controller.shutdown()


def test_stale_feedback_latches_command_interlock_until_explicit_hold(tmp_path):
    controller, sockets, now = make_controller(tmp_path)
    controller.action("connect", {})
    feed_all_motors(controller)
    controller.action("enable", {"side": "left", "enabled": True})
    controller.action(
        "joint", {"side": "left", "joint": 0, "target_deg": 30.0}
    )
    before = len(sockets["can1"].frames)

    now[0] += 2.0
    with controller._lock:
        controller._command_step_locked(now[0])
    assert len(sockets["can1"].frames) == before
    state = controller.snapshot()
    assert state["arms"]["left"]["command_interlocked"] is True
    assert state["arms"]["left"]["control_ready"] is False
    assert state["health_level"] == "BLOCKED"

    feed_all_motors(controller)
    with pytest.raises(RuntimeError, match="명령이 차단"):
        controller.action(
            "joint", {"side": "left", "joint": 0, "target_deg": 10.0}
        )
    controller.action("hold", {"side": "left"})
    assert controller.snapshot()["arms"]["left"]["command_interlocked"] is False
    controller.shutdown()


def test_global_stop_disables_every_connected_arm(tmp_path):
    controller, sockets, _now = make_controller(tmp_path)
    controller.action("connect", {})
    feed_all_motors(controller)
    controller.action("enable", {"side": "both", "enabled": True})

    result = controller.action("stop")
    assert result["accepted"] is True
    assert result["disabled_sides"] == ["left", "right"]
    assert controller.snapshot()["any_enabled"] is False
    for interface in ("can1", "can0"):
        _can_id, payload = unpack_can_frame(sockets[interface].frames[-1])
        assert payload[-1] == 0xFD
    controller.shutdown()
