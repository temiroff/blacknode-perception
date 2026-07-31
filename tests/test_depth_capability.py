"""Provider-neutral depth-camera capability tests."""
import json
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import run_workflow, validate_workflow


_COMPONENT = (
    Path(__file__).resolve().parents[1]
    / "components"
    / "depth"
)
_before = dict(_NODE_REGISTRY)
_import_nodes_module(
    "blacknode.pkg.blacknode_perception.depth",
    _COMPONENT / "nodes",
)
_tag_new_package_nodes(
    _before,
    "blacknode-perception",
    _COMPONENT / "nodes",
    "depth",
    None,
)


def _camera(distance_m=1.2, **provider_inputs):
    provider = _NODE_REGISTRY["DepthCameraTestProvider"]({
        "distance_m": distance_m,
        **provider_inputs,
    })
    capability = _NODE_REGISTRY["DepthCamera"](provider)
    return provider, capability


def test_depth_nodes_register_to_provider_neutral_component():
    for name in (
        "DepthCamera",
        "DepthCameraDeviceSelect",
        "DepthCameraTestProvider",
        "DepthObstacleWarning",
    ):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-perception"
        assert fn._bn_component == "depth"
        assert not fn._bn_adapter


def _depth_device_graph():
    return {
        "capabilities": [{
            "kind": "blacknode.robot-capability-candidate",
            "schema_version": 1,
            "capability": "depth_camera",
            "confidence": "high",
            "score": 100,
            "state_topics": [
                "/oak/depth/camera_info",
                "/oak/depth/image_raw",
            ],
            "safe_to_read": True,
            "requires_confirmation": True,
            "evidence": [
                {
                    "name": "/oak/depth/image_raw",
                    "message_type": "sensor_msgs/msg/Image",
                    "role": "state",
                },
                {
                    "name": "/oak/depth/camera_info",
                    "message_type": "sensor_msgs/msg/CameraInfo",
                    "role": "metadata",
                },
            ],
        }],
    }


def test_device_depth_candidate_requires_explicit_confirmation():
    selector = _NODE_REGISTRY["DepthCameraDeviceSelect"]
    base = {
        "device": {
            "device_id": "jetson-01",
            "device_name": "Workshop Jetson",
        },
        "ros2_graph": _depth_device_graph(),
    }

    review = selector(base)
    confirmed = selector({
        **base,
        "confirm": True,
        "hardware_id": "DEPTH-SERIAL-42",
    })

    assert review["found"] is True
    assert review["confirmed"] is False
    assert review["action"] == "stop"
    assert review["depth_topic"] == "/oak/depth/image_raw"
    assert review["camera_info_topic"] == "/oak/depth/camera_info"
    assert confirmed["action"] == "start"
    assert confirmed["provider_state"]["device_id"] == "jetson-01"
    assert confirmed["hardware"]["serial"] == "DEPTH-SERIAL-42"


def test_device_depth_candidate_degrades_and_allows_reviewed_override():
    selector = _NODE_REGISTRY["DepthCameraDeviceSelect"]
    missing = selector({
        "device": {"device_id": "jetson-01"},
        "ros2_graph": {"capabilities": []},
        "confirm": True,
    })
    override = selector({
        "device": {"device_id": "jetson-01"},
        "ros2_graph": {"capabilities": []},
        "depth_topic": "/custom/depth",
        "confirm": True,
    })

    assert missing["found"] is False
    assert missing["action"] == "stop"
    assert override["found"] is True
    assert override["action"] == "start"


def test_mock_provider_implements_stable_depth_camera_contract():
    provider, capability = _camera(1.2)

    assert provider["depth_stream"]["kind"] == "blacknode.depth-stream"
    assert provider["preview"].startswith("data:image/svg+xml;base64,")
    assert capability["ready"] is True
    assert capability["depth_camera"]["kind"] == (
        "blacknode.depth-camera-capability"
    )
    assert capability["depth_camera"]["health"]["source_fresh"] is True
    assert capability["depth_camera"]["hardware_identity"]["id"] == (
        "depth-camera-test-001"
    )


def test_replay_provider_uses_same_contract():
    _, original = _camera(1.5)
    replay = {
        "preview": original["preview"],
        "depth_stream": original["depth_stream"],
        "point_cloud_stream": original["point_cloud_stream"],
        "health": original["health"],
        "hardware": {"id": "recorded-depth-42"},
    }
    _, capability = _camera(
        mode="replay",
        replay=replay,
        hardware_id="ignored-fallback",
    )

    assert capability["ready"] is True
    assert capability["depth_camera"]["kind"] == (
        "blacknode.depth-camera-capability"
    )
    assert capability["hardware"]["id"] == "recorded-depth-42"


def test_attachment_configuration_is_portable_and_depth_specific():
    _, capability = _camera(1.0)
    configuration = capability["attachment_configuration"]

    assert configuration["attachment_type"] == "depth_camera"
    assert configuration["provider_contract"] == (
        "blacknode.depth-camera-capability"
    )
    interfaces = {
        item["name"]: item
        for item in configuration["ros2_interfaces"]
    }
    assert interfaces["depth_image"]["message_type"] == (
        "sensor_msgs/msg/Image"
    )
    assert interfaces["depth_image"]["required"] is True
    assert interfaces["depth_camera_info"]["required"] is False


def test_obstacle_warning_clear_warning_critical_and_unknown():
    cases = [
        (1.2, "clear", True),
        (0.7, "warning", False),
        (0.3, "critical", False),
    ]
    for distance_m, state, safe in cases:
        _, capability = _camera(distance_m)
        result = _NODE_REGISTRY["DepthObstacleWarning"]({
            "depth_camera": capability["depth_camera"],
        })
        assert result["assessment"]["state"] == state
        assert result["safe_to_proceed"] is safe

    _, capability = _camera(1.2)
    capability["depth_camera"]["health"]["source_fresh"] = False
    result = _NODE_REGISTRY["DepthObstacleWarning"]({
        "depth_camera": capability["depth_camera"],
    })
    assert result["assessment"]["state"] == "unknown"
    assert result["measured"] is False
    assert result["safe_to_proceed"] is False


def test_missing_replay_degrades_without_breaking_discovery():
    provider = _NODE_REGISTRY["DepthCameraTestProvider"]({
        "mode": "replay",
        "replay": {},
    })
    capability = _NODE_REGISTRY["DepthCamera"](provider)

    assert capability["ready"] is False
    assert capability["depth_camera"]["kind"] == (
        "blacknode.depth-camera-capability"
    )
    assert capability["health"]["state"] == "unavailable"


def test_unavailable_real_provider_does_not_invent_ros_interfaces():
    capability = _NODE_REGISTRY["DepthCamera"]({
        "provider_state": {
            "state": "unavailable",
            "available": False,
            "ready": False,
        },
        "depth_stream": {},
        "health": {
            "state": "unavailable",
            "source_fresh": False,
        },
        "depth_topic": "",
        "camera_info_topic": "",
    })

    assert capability["ready"] is False
    assert capability["attachment_configuration"]["ros2_interfaces"] == []


def test_depth_component_templates_validate():
    for path in sorted((_COMPONENT / "templates").glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_depth_component_lab_runs_without_saving_profile(monkeypatch, tmp_path):
    robots = tmp_path / "robots"
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(robots))
    path = _COMPONENT / "templates" / "depth-camera-component-lab.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))

    assert "RobotProfileSave" not in {
        node["type"] for node in workflow["node_meta"].values()
    }
    result = run_workflow(workflow)
    assessment, attachment, profile, preview = result["value"]

    assert assessment["state"] == "clear"
    assert attachment["capability"] == "depth_camera"
    assert attachment["hardware_identity"]["id"] == "depth-camera-test-001"
    assert profile["capabilities"] == ["depth_camera"]
    assert preview.startswith("data:image/svg+xml;base64,")
    assert not robots.exists()


def test_real_device_template_defaults_to_stopped_and_unsaved(monkeypatch, tmp_path):
    robots = tmp_path / "robots"
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(robots))
    path = _COMPONENT / "templates" / "depth-camera-device-live.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    node_types = {node["type"] for node in workflow["node_meta"].values()}

    assert "DepthCameraTestProvider" not in node_types
    assert "RobotProfileSave" not in node_types
    assert workflow["node_meta"]["select_depth"]["params"]["confirm"] is False

    result = run_workflow(workflow)
    provider, health, assessment, attachment, profile = result["value"]

    assert provider["state"] == "unavailable"
    assert health["source_fresh"] is False
    assert assessment["state"] == "unknown"
    assert attachment["interfaces"] == []
    assert profile["capabilities"] == ["depth_camera"]
    assert not robots.exists()
