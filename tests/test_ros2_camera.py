"""blacknode-perception — camera capability over ROS 2 (adapter contracts).

All tests run without ROS: the ros2 runtime helpers are monkeypatched, so the
topic-type resolution, stream lifecycle, and USB bridging logic are exercised
pure. Every node must return a structured report instead of raising.
"""
import json
from pathlib import Path

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import validate_workflow

_ADAPTER = Path(__file__).resolve().parents[1] / "components" / "camera" / "adapters" / "ros2"
_ADAPTER_NODES = _ADAPTER / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_perception.camera.adapters.ros2", _ADAPTER_NODES)
_tag_new_package_nodes(_before, "blacknode-perception", _ADAPTER_NODES, "camera", "ros2")

from blacknode.pkg.blacknode_perception.camera.adapters.ros2 import camera_stream as cam
from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt

NEW_NODES = [
    "CameraROS2Provider",
    "CameraROS2Publish",
    "CameraROS2Http",
]


def test_new_nodes_registered_with_category_and_package():
    for name in NEW_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "Perception"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-perception"


def test_camera_image_processor_normalizes_generic_ros2_stream():
    result = _NODE_REGISTRY["CameraImageProcessor"]({
        "source": {
            "kind": "blacknode.message-stream",
            "protocol": "ros2",
            "state": "ready",
            "stream_id": "rgb",
            "topic": "/camera/image_raw",
            "message_type": "sensor_msgs/msg/Image",
            "stream_url": "http://robot.local:19001/stream.mjpg",
            "snapshot_url": "http://robot.local:19001/snapshot.jpg",
            "health_url": "http://robot.local:19001/health",
        },
    })

    assert result["ready"] is True
    assert result["preview"].endswith("/stream.mjpg")
    assert result["frame_stream"]["kind"] == "blacknode.frame-stream"


# --- CameraROS2Provider -------------------------------------------------------

def test_usb_camera_provider_starts_named_process_and_verifies_topics(monkeypatch):
    captured = {}

    def fake_start(key, args):
        captured.update(key=key, args=args)
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_start)
    monkeypatch.setattr(rt, "wait_for_topic_interfaces", lambda items, timeout: {
        "ok": True,
        "ready": True,
        "backend": "native",
        "interfaces": items,
        "missing": [],
    })

    result = _NODE_REGISTRY["CameraROS2Provider"]({
        "action": "start",
        "run_id": "front camera",
        "profile": "usb_cam",
        "rgb_topic": "/front/image_raw",
        "rgb_info_topic": "/front/camera_info",
        "depth_topic": "",
        "depth_info_topic": "",
        "points_topic": "",
    })

    assert result["running"] is True
    assert result["ready"] is True
    assert captured["key"] == "front_camera"
    assert captured["args"] == [
        "launch",
        "perception_camera",
        "usb_camera.launch.py",
        "image_topic:=/front/image_raw",
        "camera_info_topic:=/front/camera_info",
    ]


def test_blacknode_provider_uses_normalized_rgbd_interfaces(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "run_ros2_managed", lambda key, args: (
        captured.update(key=key, args=args)
        or {"ok": True, "backend": "native"}
    ))

    def fake_wait(items, timeout):
        captured["interfaces"] = items
        return {
            "ok": True,
            "ready": True,
            "backend": "native",
            "interfaces": items,
            "missing": [],
        }

    monkeypatch.setattr(rt, "wait_for_topic_interfaces", fake_wait)
    result = _NODE_REGISTRY["CameraROS2Provider"]({
        "action": "start",
        "profile": "blacknode_rgbd",
        "require_depth": True,
    })

    assert result["ready"] is True
    assert captured["args"] == [
        "launch",
        "perception_camera",
        "rgbd_camera.launch.py",
    ]
    by_name = {item["name"]: item for item in captured["interfaces"]}
    assert by_name["rgb_image"]["topic"] == "/camera/rgb/image_raw"
    assert by_name["depth_image"]["topic"] == "/camera/depth/image_raw"
    assert by_name["depth_image"]["required"] is True
    assert "point_cloud" not in by_name


def test_retired_depth_profile_migrates_to_blacknode_rgbd(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "run_ros2_managed", lambda key, args: (
        captured.update(key=key, args=args)
        or {"ok": True, "backend": "native"}
    ))
    monkeypatch.setattr(rt, "wait_for_topic_interfaces", lambda items, timeout: {
        "ok": True,
        "ready": True,
        "backend": "native",
        "interfaces": items,
        "missing": [],
    })

    result = _NODE_REGISTRY["CameraROS2Provider"]({
        "action": "start",
        "profile": "retired_depth_profile",
        "require_depth": True,
    })

    assert result["profile"] == "blacknode_rgbd"
    assert captured["args"][:3] == [
        "launch",
        "perception_camera",
        "rgbd_camera.launch.py",
    ]


def test_existing_camera_topics_status_never_starts_process(monkeypatch):
    monkeypatch.setattr(
        rt,
        "run_ros2_managed",
        lambda *args: pytest.fail("existing topics must not start a process"),
    )
    monkeypatch.setattr(rt, "inspect_topic_interfaces", lambda items: {
        "ok": True,
        "ready": True,
        "backend": "native",
        "interfaces": items,
        "missing": [],
    })

    result = _NODE_REGISTRY["CameraROS2Provider"]({
        "action": "status",
        "profile": "existing_topics",
        "rgb_topic": "/camera/image_raw",
    })

    assert result["running"] is True
    assert result["ready"] is True


def test_camera_provider_stop_is_scoped(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": (
        captured.update(key=key, pattern=pattern)
        or {"ok": True, "stopped": 1}
    ))

    result = _NODE_REGISTRY["CameraROS2Provider"]({
        "action": "stop",
        "profile": "usb_cam",
        "run_id": "front camera",
    })

    assert result["running"] is False
    assert captured["key"] == "front_camera"
    assert captured["pattern"] == (
        "ros2 launch perception_camera usb_camera.launch.py"
    )


# --- CameraROS2Publish

def test_publish_bridges_the_wired_stream_and_reads_it_back_out_of_ros(monkeypatch):
    # Capture belongs to whatever is wired in; this node only publishes it and
    # proves the topic carries it by showing the picture read back from ROS.
    bridged = {}
    monkeypatch.setattr(rt, "start_host_camera_publisher",
                        lambda **k: bridged.update(k) or {"ok": True, "backend": "docker"})

    def fake_run(args, timeout=15.0):
        stdout = "sensor_msgs/msg/Image\n" if args[:2] == ["topic", "type"] else "/camera/image_raw\n"
        return {"ok": True, "backend": "docker", "stdout": stdout, "stderr": ""}

    monkeypatch.setattr(rt, "run_ros2", fake_run)
    monkeypatch.setattr(rt, "start_image_stream", lambda **k: {
        "ok": True, "backend": "docker", "stream_url": "http://127.0.0.1:39000/stream.mjpg",
        "snapshot_url": "", "health_url": "",
    })

    result = _NODE_REGISTRY["CameraROS2Publish"]({
        "action": "start",
        "frame_stream": {"stream_url": "http://127.0.0.1:5000/stream.mjpg", "label": "USB Cam"},
    })

    assert bridged["source_url"] == "http://127.0.0.1:5000/stream.mjpg"
    assert result["streaming"] is True
    assert result["camera"] == "USB Cam"
    # the picture must come back out of ROS, not straight from the wired stream
    assert result["preview"] == "http://127.0.0.1:39000/stream.mjpg"


def test_publish_explains_an_unwired_frame_stream(monkeypatch):
    monkeypatch.setattr(rt, "start_host_camera_publisher",
                        lambda **k: pytest.fail("must not bridge without a source"))

    result = _NODE_REGISTRY["CameraROS2Publish"]({"action": "start"})

    assert result["streaming"] is False
    assert "frame_stream" in result["report"]
    assert "Camera" in result["report"]


# --- CameraROS2Http -----------------------------------------------------------

def test_web_video_stream_refuses_the_placeholder_host():
    result = _NODE_REGISTRY["CameraROS2Http"]({"host": "ROBOT_IP", "topic": "/camera/image_raw"})

    assert result["streaming"] is False
    assert result["preview"] == ""
    assert "robot's IP address" in result["report"]


def test_web_video_stream_builds_url_and_reports_live(monkeypatch):
    seen = {}

    def fake_probe(url, timeout):
        seen["url"] = url
        return True, "multipart/x-mixed-replace"

    monkeypatch.setattr(rt, "probe_web_video", fake_probe)

    result = _NODE_REGISTRY["CameraROS2Http"]({
        "host": "192.168.1.50",
        "port": 8080,
        "topic": "/depth_cam/rgb0/image_raw",
        "quality": 70,
    })

    assert result["streaming"] is True
    assert seen["url"].startswith("http://192.168.1.50:8080/stream?")
    assert "topic=/depth_cam/rgb0/image_raw" in seen["url"]
    assert "quality=70" in seen["url"]
    assert result["preview"] == seen["url"]
    assert "LIVE robot camera" in result["report"]


def test_web_video_stream_explains_an_unreachable_robot(monkeypatch):
    monkeypatch.setattr(rt, "probe_web_video", lambda url, timeout: (False, "cannot reach the robot"))

    result = _NODE_REGISTRY["CameraROS2Http"]({"host": "192.168.1.50", "topic": "/camera/image_raw"})

    assert result["streaming"] is False
    assert result["preview"] == ""
    assert "cannot reach the robot" in result["report"]
    assert "8080" in result["report"]


def test_templates_validate():
    for path in sorted((_ADAPTER / "templates").glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_publish_accepts_a_real_camera_frame_stream(monkeypatch):
    # The regression this pins: Camera emits the frame-stream contract, and the
    # publisher must find the video URL in it. Reading a key the contract does
    # not define made a correctly wired graph report "nothing wired".
    bridged = {}
    monkeypatch.setattr(rt, "start_host_camera_publisher",
                        lambda **k: bridged.update(k) or {"ok": True, "backend": "docker"})
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True, "backend": "docker", "stderr": "",
        "stdout": "sensor_msgs/msg/Image" if args[:2] == ["topic", "type"] else "/camera/image_raw",
    })
    monkeypatch.setattr(rt, "start_image_stream", lambda **k: {
        "ok": True, "backend": "docker", "stream_url": "http://127.0.0.1:39000/stream.mjpg",
        "snapshot_url": "", "health_url": "",
    })

    result = _NODE_REGISTRY["CameraROS2Publish"]({
        "action": "start",
        "frame_stream": {
            "kind": "blacknode.frame-stream",
            "schema_version": 1,
            "stream_id": "camera_0",
            "stream_url": "http://127.0.0.1:49361/stream.mjpg",
            "snapshot_url": "http://127.0.0.1:49361/snapshot.jpg",
            "media_type": "image/jpeg",
        },
    })

    assert result["streaming"] is True, result["report"]
    assert bridged["source_url"] == "http://127.0.0.1:49361/stream.mjpg"


def test_publish_falls_back_to_the_snapshot_sibling(monkeypatch):
    # Streams recorded before stream_url joined the contract still publish.
    bridged = {}
    monkeypatch.setattr(rt, "start_host_camera_publisher",
                        lambda **k: bridged.update(k) or {"ok": True, "backend": "docker"})
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True, "backend": "docker", "stderr": "",
        "stdout": "sensor_msgs/msg/Image" if args[:2] == ["topic", "type"] else "/camera/image_raw",
    })
    monkeypatch.setattr(rt, "start_image_stream", lambda **k: {
        "ok": True, "backend": "docker", "stream_url": "http://127.0.0.1:39000/stream.mjpg",
        "snapshot_url": "", "health_url": "",
    })

    result = _NODE_REGISTRY["CameraROS2Publish"]({
        "action": "start",
        "frame_stream": {"stream_id": "camera_0",
                         "snapshot_url": "http://127.0.0.1:49361/snapshot.jpg"},
    })

    assert result["streaming"] is True, result["report"]
    assert bridged["source_url"] == "http://127.0.0.1:49361/stream.mjpg"
