"""Provider-neutral LiDAR capability tests."""
import json
import math
from pathlib import Path

import blacknode  # noqa: F401
from blacknode import contracts as bn_contracts
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
    for name in ("LaserScanProcessor", "LiDAR"):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-perception"
        assert fn._bn_component == "lidar"
        assert not fn._bn_adapter


def _lidar_input(sample_count=72, hardware_id="lidar-fixture-001"):
    scan = bn_contracts.laser_scan_stream(
        "laser",
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / sample_count,
        range_min=0.05,
        range_max=12.0,
        ranges=[2.0] * sample_count,
    )
    scan.update({
        "topic": "/scan",
        "message_type": "sensor_msgs/msg/LaserScan",
    })
    return {
        "provider_state": {"state": "ready", "available": True, "ready": True},
        "laser_scan": scan,
        "health": {"state": "ready", "source_fresh": True, "valid_count": sample_count},
        "hardware": {"id": hardware_id, "kind": "lidar"},
    }


def test_laser_scan_processor_normalizes_generic_ros2_stream():
    source = {
        "kind": "blacknode.message-stream",
        "protocol": "ros2",
        "topic": "/scan",
        "message_type": "sensor_msgs/msg/LaserScan",
    }

    def reader(_source):
        return {
            "message": {
                "header": {"frame_id": "laser"},
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": math.pi / 2,
                "range_min": 0.05,
                "range_max": 12.0,
                "ranges": [1.0, 2.0, 3.0, 4.0],
                "intensities": [10.0, 20.0, 30.0, 40.0],
            },
            "status": {"state": "ready", "source_fresh": True},
        }

    result = _NODE_REGISTRY["LaserScanProcessor"]({
        "source": source,
        "__message_stream_reader__": reader,
    })

    assert result["stream"]["processor"] == "LaserScanProcessor"
    assert result["laser_scan"]["kind"] == "blacknode.laser-scan-stream"
    assert result["laser_scan"]["ranges"] == [1.0, 2.0, 3.0, 4.0]
    assert result["health"]["source_fresh"] is True


def test_lidar_capability_accepts_a_normalized_provider_contract():
    provider = _lidar_input()
    capability = _NODE_REGISTRY["LiDAR"]({**provider, "attachment_id": "front_scan"})

    assert provider["laser_scan"]["kind"] == "blacknode.laser-scan-stream"
    assert len(provider["laser_scan"]["ranges"]) == 72
    assert capability["ready"] is True
    assert capability["lidar"]["kind"] == "blacknode.lidar-capability"
    assert capability["lidar"]["health"]["source_fresh"] is True
    assert capability["hardware"]["id"] == "lidar-fixture-001"
    interface = capability["attachment_configuration"]["ros2_interfaces"][0]
    assert interface["message_type"] == "sensor_msgs/msg/LaserScan"
    assert interface["required"] is True


def test_lidar_capability_degrades_when_provider_data_is_missing():
    missing = _NODE_REGISTRY["LiDAR"]({})

    assert missing["ready"] is False
    assert missing["health"]["state"] == "unavailable"


def test_lidar_templates_validate():
    for path in sorted((_COMPONENT / "templates").glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        report = validate_workflow(workflow)
        assert report.ok, f"{path.name}: {report.to_dict()}"
