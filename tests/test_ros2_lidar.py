"""LiDAR capability over ROS 2."""
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes


_ADAPTER = (
    Path(__file__).resolve().parents[1]
    / "components" / "lidar" / "adapters" / "ros2"
)
_before = dict(_NODE_REGISTRY)
_module = _import_nodes_module(
    "blacknode.pkg.blacknode_perception.lidar.adapters.ros2",
    _ADAPTER / "nodes",
)
_tag_new_package_nodes(
    _before,
    "blacknode-perception",
    _ADAPTER / "nodes",
    "lidar",
    "ros2",
)
laser_scan_module = _module.laser_scan


class _Runtime:
    def __init__(self):
        self.started = None
        self.stopped = None

    def run_topic_subscriber_once(self, **kwargs):
        return {
            "ok": True,
            "backend": "native",
            "latest": {"message": {
                "header": {"stamp": {"sec": 12, "nanosec": 34}, "frame_id": "laser_front"},
                "angle_min": -1.0,
                "angle_max": 1.0,
                "angle_increment": 0.5,
                "time_increment": 0.001,
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 8.0,
                "ranges": [0.05, 1.0, 2.0, 9.0],
                "intensities": [1.0, 2.0, 3.0, 4.0],
            }},
        }

    def detect_backend(self):
        return {"backend": "native"}

    def inspect_topic_interfaces(self, items):
        return {"ok": True, "ready": True, "interfaces": items, "missing": []}

    def start_ros2_python_node(self, **kwargs):
        self.started = kwargs
        return {"ok": True, "running": True, "backend": "native"}

    def stop_ros2_python_node(self, run_id):
        self.stopped = run_id
        return {"ok": True, "stopped": 1}


def test_lidar_ros2_nodes_register_to_adapter():
    for name in ("LiDARROS2Scan", "LiDARROS2WarpViewer"):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-perception"
        assert fn._bn_component == "lidar"
        assert fn._bn_adapter == "ros2"
    assert _NODE_REGISTRY["LiDARROS2WarpViewer"]._bn_hidden is True


def test_laser_scan_capture_normalizes_message(monkeypatch):
    runtime = _Runtime()
    monkeypatch.setattr(laser_scan_module, "_runtime", lambda: runtime)
    result = _NODE_REGISTRY["LiDARROS2Scan"]({"topic": "/front/scan"})

    assert result["captured"] is True
    assert result["laser_scan"]["kind"] == "blacknode.laser-scan-stream"
    assert result["laser_scan"]["frame"] == "laser_front"
    assert result["laser_scan"]["source_time_ns"] == 12_000_000_034
    assert result["health"]["sample_count"] == 4
    assert result["health"]["valid_count"] == 2


def test_live_warp_viewer_starts_and_stops_managed_native_process(monkeypatch):
    runtime = _Runtime()
    monkeypatch.setattr(laser_scan_module, "_runtime", lambda: runtime)
    started = _NODE_REGISTRY["LiDARROS2WarpViewer"]({
        "action": "start",
        "topic": "/front/scan",
        "viewer_id": "front",
        "downsample_stride": 2,
    })
    stopped = _NODE_REGISTRY["LiDARROS2WarpViewer"]({
        "action": "stop",
        "viewer_id": "front",
    })

    assert started["running"] is True
    assert started["source_ready"] is True
    assert runtime.started["run_id"] == "front"
    assert "--stride" in runtime.started["arguments"]
    assert stopped["running"] is False
    assert runtime.stopped == "front"
