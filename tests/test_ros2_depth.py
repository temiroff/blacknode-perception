"""Depth capability over ROS 2."""
import json
from pathlib import Path

import pytest

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import validate_workflow

_ADAPTER = (
    Path(__file__).resolve().parents[1]
    / "components"
    / "depth"
    / "adapters"
    / "ros2"
)
_before = dict(_NODE_REGISTRY)
_import_nodes_module(
    "blacknode.pkg.blacknode_perception.depth.adapters.ros2",
    _ADAPTER / "nodes",
)
_tag_new_package_nodes(
    _before,
    "blacknode-perception",
    _ADAPTER / "nodes",
    "depth",
    "ros2",
)

from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt


def test_depth_node_is_registered_to_depth_adapter():
    fn = _NODE_REGISTRY["DepthROS2Subscribe"]
    assert fn._bn_package == "blacknode-perception"
    assert fn._bn_component == "depth"
    assert fn._bn_adapter == "ros2"


def test_depth_stream_preserves_metric_contract_and_preview(monkeypatch):
    depth_node = _NODE_REGISTRY["DepthROS2Subscribe"]
    monkeypatch.setattr(rt, "inspect_topic_interfaces", lambda items: {
        "ok": True,
        "ready": True,
        "backend": "native",
        "interfaces": items,
        "missing": [],
    })
    monkeypatch.setattr(rt, "start_image_stream", lambda **kwargs: {
        "ok": True,
        "backend": "native",
        "stream_url": "http://127.0.0.1:9015/stream.mjpg",
        "snapshot_url": "http://127.0.0.1:9015/snapshot.jpg",
        "health_url": "http://127.0.0.1:9015/health.json",
        "frame_url": "http://127.0.0.1:9015/frame.bin",
    })
    monkeypatch.setitem(
        depth_node.__globals__,
        "_read_stream_health",
        lambda url, wait_seconds: {
            "frames": 4,
            "metadata": {
                "encoding": "16UC1",
                "frame_id": "depth_frame",
                "received_at_ns": depth_node.__globals__["time"].time_ns(),
                "depth_summary_raw": {
                    "encoding": "16UC1",
                    "valid_count": 100,
                    "total_count": 120,
                    "minimum": 450.0,
                    "p05": 500.0,
                    "median": 1200.0,
                    "p95": 3000.0,
                },
            },
            "error": "",
        },
    )

    result = depth_node({
        "topic": "/depth_cam/depth0/image_raw",
        "camera_info_topic": "/depth_cam/depth0/camera_info",
        "points_topic": "/depth_cam/depth0/points",
        "frame_id": "depth_frame",
        "encoding": "16UC1",
        "depth_scale": 0.001,
        "fx": 525.0,
        "fy": 525.0,
    })

    assert result["streaming"] is True
    assert result["preview"].endswith("/stream.mjpg")
    assert result["depth_stream"]["kind"] == "blacknode.depth-stream"
    assert result["depth_stream"]["topic"] == "/depth_cam/depth0/image_raw"
    assert result["depth_stream"]["encoding"] == "16UC1"
    assert result["depth_stream"]["depth_scale"] == 0.001
    assert result["depth_stream"]["summary_m"]["p05"] == 0.5
    assert result["frame_url"].endswith("/frame.bin")
    assert result["depth_stream"]["frame_source"]["transport"] == "http-binary"
    assert result["depth_stream"]["calibration"]["fx"] == 525.0
    assert result["health"]["state"] == "ready"
    assert result["health"]["source_fresh"] is True
    assert result["point_cloud_stream"]["kind"] == (
        "blacknode.point-cloud-stream"
    )


def test_depth_stream_reports_missing_publisher_without_starting(monkeypatch):
    monkeypatch.setattr(rt, "inspect_topic_interfaces", lambda items: {
        "ok": True,
        "ready": False,
        "backend": "native",
        "interfaces": [],
        "missing": [items[0]["topic"]],
    })
    monkeypatch.setattr(
        rt,
        "start_image_stream",
        lambda **kwargs: pytest.fail("missing depth topic must not start preview"),
    )

    result = _NODE_REGISTRY["DepthROS2Subscribe"]({
        "topic": "/depth_cam/depth0/image_raw",
    })

    assert result["streaming"] is False
    assert "/depth_cam/depth0/image_raw" in result["report"]


def test_depth_stream_stop_is_explicit(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "stop_image_stream", lambda stream_id="": (
        captured.update(stream_id=stream_id)
        or {"ok": True, "stopped": 1}
    ))

    result = _NODE_REGISTRY["DepthROS2Subscribe"]({
        "action": "stop",
        "stream_id": "front_depth",
    })

    assert result["streaming"] is False
    assert captured["stream_id"] == "front_depth"


def test_depth_ros2_templates_validate():
    camera_adapter = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "camera"
        / "adapters"
        / "ros2"
    )
    before = dict(_NODE_REGISTRY)
    _import_nodes_module(
        "blacknode.pkg.blacknode_perception.camera.adapters.ros2",
        camera_adapter / "nodes",
    )
    _tag_new_package_nodes(
        before,
        "blacknode-perception",
        camera_adapter / "nodes",
        "camera",
        "ros2",
    )
    for path in sorted((_ADAPTER / "templates").glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"
