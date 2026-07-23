import pathlib
import time

import pytest

from romo_b_operator_ui.operations import OperationManager


def make_manager(tmp_path: pathlib.Path) -> OperationManager:
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    tracked = scripts / "check_tracked_file_sizes.sh"
    tracked.write_text("#!/bin/sh\nprintf 'tracked-size-check PASS\\n'\n")
    tracked.chmod(0o755)
    (tmp_path / "data" / "local" / "maps").mkdir(parents=True)
    (tmp_path / "data" / "local" / "bags").mkdir(parents=True)
    return OperationManager(tmp_path)


def test_unknown_operations_are_rejected(tmp_path):
    manager = make_manager(tmp_path)
    with pytest.raises(ValueError, match="알 수 없는 작업"):
        manager.start("arbitrary_shell", {})


def test_artifact_ids_cannot_escape_local_roots(tmp_path):
    manager = make_manager(tmp_path)
    with pytest.raises(ValueError, match="올바른 로컬 데이터"):
        manager._map({"map_id": "../outside"})


def test_robot_control_defaults_to_manual_speed_limit(tmp_path):
    manager = make_manager(tmp_path)
    script = tmp_path / "scripts" / "run_robot_control.sh"
    script.write_text("#!/bin/sh\n")
    script.chmod(0o755)

    _command, environment = manager._command("robot_control", {})

    assert environment["MAX_SPEED_MPS"] == "1.5"


def test_map_and_bag_inventory_reports_readiness(tmp_path):
    manager = make_manager(tmp_path)
    map_run = manager.map_root / "mapping-20260721-120000"
    (map_run / "nav2-raycast").mkdir(parents=True)
    (map_run / "map.pcd").write_text("pcd")
    (map_run / "pose_graph.g2o").write_text("graph")
    (map_run / "nav2-raycast" / "map.yaml").write_text("image: map.pgm")
    bag = manager.bag_root / "mapping-20260721-120000"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
    (bag / "mapping_0.db3").write_bytes(b"db")

    artifacts = manager.artifacts()

    assert artifacts["maps"][0]["ready_nav2"] is True
    assert artifacts["maps"][0]["has_pose_graph"] is True
    assert artifacts["bags"][0]["has_metadata"] is True
    assert artifacts["bags"][0]["db3_count"] == 1


def test_allowlisted_process_records_exit_code_and_log(tmp_path):
    manager = make_manager(tmp_path)

    result = manager.start("tracked_sizes", {})
    assert result["accepted"] is True

    deadline = time.monotonic() + 3.0
    task = None
    while time.monotonic() < deadline:
        task = next(
            item
            for item in manager.snapshot()["tasks"]
            if item["id"] == "tracked_sizes"
        )
        if not task["running"]:
            break
        time.sleep(0.02)

    assert task["exit_code"] == 0
    assert "tracked-size-check PASS" in manager.log_tail("tracked_sizes")["tail"]


def test_openarm_auto_calibration_command_is_allowlisted_and_guarded(tmp_path):
    manager = make_manager(tmp_path)
    script = tmp_path / "scripts" / "run_openarm_auto_calibration.sh"
    script.write_text("#!/bin/sh\n")
    script.chmod(0o755)

    command, environment = manager._command(
        "openarm_auto_calibration",
        {
            "side": "left",
            "interface": "can1",
            "confirmation": "OPENARM AUTO CAL",
        },
    )

    assert command == [str(script), "left", "can1"]
    assert environment["PYTHONUNBUFFERED"] == "1"

    with pytest.raises(ValueError, match="확인 문구"):
        manager._command(
            "openarm_auto_calibration",
            {"side": "left", "interface": "can1", "confirmation": "no"},
        )
    with pytest.raises(ValueError, match="인터페이스"):
        manager._command(
            "openarm_auto_calibration",
            {
                "side": "right",
                "interface": "can0;touch",
                "confirmation": "OPENARM AUTO CAL",
            },
        )
