"""Provider-neutral LiDAR capability tests."""
import json
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import run_workflow, validate_workflow


_COMPONENT = Path(__file__).resolve().parents[1] / "components" / "lidar"
_before = dict(_NODE_REGISTRY)
_import_nodes_module(
    "blacknode.pkg.blacknode_perception.lidar",
    _COMPONENT / "nodes",
)
_tag_new_package_nodes(
    _before,
    "blacknode-perception",
    _COMPONENT / "nodes",
    "lidar",
    None,
)


def test_lidar_nodes_register_to_provider_neutral_component():
    for name in ("LiDAR", "LiDARTestProvider"):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-perception"
        assert fn._bn_component == "lidar"
        assert not fn._bn_adapter


def test_mock_provider_implements_stable_lidar_contract():
    provider = _NODE_REGISTRY["LiDARTestProvider"]({
        "sample_count": 72,
        "room_width_m": 6.0,
        "room_height_m": 4.0,
    })
    capability = _NODE_REGISTRY["LiDAR"]({**provider, "attachment_id": "front_scan"})

    assert provider["laser_scan"]["kind"] == "blacknode.laser-scan-stream"
    assert len(provider["laser_scan"]["ranges"]) == 72
    assert capability["ready"] is True
    assert capability["lidar"]["kind"] == "blacknode.lidar-capability"
    assert capability["lidar"]["health"]["source_fresh"] is True
    assert capability["hardware"]["id"] == "lidar-test-001"
    interface = capability["attachment_configuration"]["ros2_interfaces"][0]
    assert interface["message_type"] == "sensor_msgs/msg/LaserScan"
    assert interface["required"] is True


def test_replay_provider_uses_same_contract_and_missing_replay_degrades():
    original = _NODE_REGISTRY["LiDARTestProvider"]({"sample_count": 16})
    replay = _NODE_REGISTRY["LiDARTestProvider"]({
        "mode": "replay",
        "replay": {
            "laser_scan": original["laser_scan"],
            "health": original["health"],
            "hardware": {"id": "recorded-lidar-42"},
        },
    })
    missing = _NODE_REGISTRY["LiDARTestProvider"]({"mode": "replay", "replay": {}})

    assert _NODE_REGISTRY["LiDAR"](replay)["ready"] is True
    assert replay["hardware"]["id"] == "recorded-lidar-42"
    assert _NODE_REGISTRY["LiDAR"](missing)["ready"] is False
    assert missing["health"]["state"] == "unavailable"


def test_lidar_templates_validate_and_static_lab_runs():
    for path in sorted((_COMPONENT / "templates").glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        report = validate_workflow(workflow)
        assert report.ok, f"{path.name}: {report.to_dict()}"

    path = _COMPONENT / "templates" / "lidar-warp-static.json"
    result = run_workflow(json.loads(path.read_text(encoding="utf-8")))
    point_cloud, processing, viewer = result["value"]
    assert point_cloud["kind"] == "blacknode.point-cloud-frame"
    assert point_cloud["point_count"] == 180
    assert processing["implementation"] == "NVIDIA Warp kernel"
    assert viewer["state"] == "stopped"
