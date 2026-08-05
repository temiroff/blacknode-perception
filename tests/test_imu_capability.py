"""Provider-neutral IMU capability and managed viewer tests."""
import json
import math
from pathlib import Path

import blacknode  # noqa: F401
import pytest
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import run_workflow, validate_workflow


_COMPONENT = Path(__file__).resolve().parents[1] / "components" / "imu"
_before = dict(_NODE_REGISTRY)
_module = _import_nodes_module(
    "blacknode.pkg.blacknode_perception.imu",
    _COMPONENT / "nodes",
)
_tag_new_package_nodes(
    _before,
    "blacknode-perception",
    _COMPONENT / "nodes",
    "imu",
    None,
)


def test_imu_nodes_register_to_provider_neutral_component():
    for name in ("IMU", "IMUTestProvider", "IMUViewer"):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-perception"
        assert fn._bn_component == "imu"
        assert not fn._bn_adapter


def test_mock_provider_and_capability_preserve_normalized_orientation_contract():
    provider = _NODE_REGISTRY["IMUTestProvider"]({
        "roll_deg": 30.0,
        "pitch_deg": -15.0,
        "yaw_deg": 90.0,
    })
    capability = _NODE_REGISTRY["IMU"]({**provider, "attachment_id": "body_imu"})

    orientation = provider["imu"]["orientation"]
    norm = math.sqrt(sum(float(value) ** 2 for value in orientation.values()))
    assert provider["imu"]["kind"] == "blacknode.imu-stream"
    assert norm == pytest.approx(1.0)
    assert capability["ready"] is True
    assert capability["imu_capability"]["kind"] == "blacknode.imu-capability"
    interface = capability["attachment_configuration"]["ros2_interfaces"][0]
    assert interface["message_type"] == "sensor_msgs/msg/Imu"
    assert interface["required"] is True


def test_replay_provider_degrades_structurally_when_sample_is_missing():
    missing = _NODE_REGISTRY["IMUTestProvider"]({"mode": "replay", "replay": {}})
    capability = _NODE_REGISTRY["IMU"](missing)

    assert missing["health"]["state"] == "unavailable"
    assert capability["ready"] is False
    assert capability["health"]["source_fresh"] is False


def test_imu_viewer_tracks_ros_quaternion_and_source_freshness():
    messages = [{
        "header": {"frame_id": "imu_link", "stamp": {"sec": 12, "nanosec": 5}},
        "orientation": {"x": 0.0, "y": 0.0, "z": math.sqrt(0.5), "w": math.sqrt(0.5)},
        "orientation_covariance": [0.01] + [0.0] * 8,
        "angular_velocity": {"x": 0.1, "y": -0.2, "z": 0.3},
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
    }]

    def reader(_source):
        return {
            "message": messages[-1],
            "received": 7,
            "status": {"state": "ready", "source_fresh": True, "received": 7, "age_seconds": 0.02},
        }

    result = _NODE_REGISTRY["IMUViewer"]({
        "action": "start",
        "viewer_id": "imu-test",
        "source": {
            "kind": "blacknode.message-stream",
            "protocol": "ros2",
            "topic": "/imu/data",
            "message_type": "sensor_msgs/msg/Imu",
        },
        "__message_stream_reader__": reader,
        "__node_id__": "imu-viewer-node",
    })

    assert result["running"] is True
    assert result["live"] is True
    assert result["scene"]["primitive"] == "imu-orientation"
    assert result["scene"]["frame"] == "imu_link"
    assert result["scene"]["imu"]["euler_rad"]["yaw"] == pytest.approx(math.pi / 2)
    assert result["scene"]["imu"]["angular_velocity_rps"]["y"] == pytest.approx(-0.2)
    runtime = _module.imu.runtime_status()
    assert runtime["node_outputs"][0]["node_id"] == "imu-viewer-node"
    _module.imu.stop_runtime_services()


def test_imu_viewer_rejects_messages_with_no_orientation_estimate():
    def reader(_source):
        return {
            "message": {
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "orientation_covariance": [-1.0] + [0.0] * 8,
            },
            "received": 1,
            "status": {"state": "ready", "source_fresh": True, "received": 1, "age_seconds": 0.0},
        }

    result = _NODE_REGISTRY["IMUViewer"]({
        "action": "start",
        "viewer_id": "imu-unavailable",
        "source": {"kind": "blacknode.message-stream", "protocol": "ros2", "topic": "/imu/data"},
        "__message_stream_reader__": reader,
    })
    assert result["live"] is False
    assert result["status"]["state"] == "unavailable"
    assert "orientation is unavailable" in result["report"]
    _module.imu.stop_runtime_services()


def test_imu_templates_validate_and_hardware_free_lab_runs():
    for path in sorted((_COMPONENT / "templates").glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        report = validate_workflow(workflow)
        assert report.ok, f"{path.name}: {report.to_dict()}"

    path = _COMPONENT / "templates" / "imu-orientation-lab.json"
    result = run_workflow(json.loads(path.read_text(encoding="utf-8")))
    assert result["value"]["primitive"] == "imu-orientation"
    assert result["value"]["imu"]["source_fresh"] is True
    _module.imu.stop_runtime_services()
