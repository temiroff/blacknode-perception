"""Depth image processing after the generic ROS2 transport node."""
import json
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes
from blacknode.workflow import validate_workflow


_COMPONENT = Path(__file__).resolve().parents[1] / "components" / "depth"
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


def _source(**updates):
    source = {
        "kind": "blacknode.message-stream",
        "schema_version": 1,
        "protocol": "ros2",
        "state": "ready",
        "stream_id": "depth",
        "topic": "/camera/depth/image_raw",
        "message_type": "sensor_msgs/msg/Image",
        "stream_url": "http://robot.local:19002/stream.mjpg",
        "snapshot_url": "http://robot.local:19002/snapshot.jpg",
        "health_url": "",
        "frame_url": "http://robot.local:19002/frame.bin",
        "metadata": {"width": 640, "height": 480, "encoding": "16UC1"},
    }
    source.update(updates)
    return source


def test_depth_processor_is_registered_to_depth_component():
    fn = _NODE_REGISTRY["DepthImageProcessor"]
    assert fn._bn_package == "blacknode-perception"
    assert fn._bn_component == "depth"
    assert not fn._bn_adapter


def test_depth_processor_preserves_metric_and_binary_frame_contracts():
    result = _NODE_REGISTRY["DepthImageProcessor"]({
        "source": _source(),
        "frame_id": "camera_depth",
        "depth_scale": 0.001,
        "fx": 525.0,
        "fy": 525.0,
        "cx": 319.5,
        "cy": 239.5,
    })

    assert result["preview"].endswith("/stream.mjpg")
    assert result["depth_stream"]["kind"] == "blacknode.depth-stream"
    assert result["depth_stream"]["frame_source"]["url"].endswith("/frame.bin")
    assert result["depth_stream"]["depth_scale"] == 0.001
    assert result["calibration"]["ready"] is True


def test_depth_processor_reads_camera_info_from_a_second_generic_stream():
    info_source = {
        "kind": "blacknode.message-stream",
        "protocol": "ros2",
        "topic": "/camera/depth/camera_info",
        "message_type": "sensor_msgs/msg/CameraInfo",
    }

    def reader(source):
        assert source is info_source
        return {"message": {
            "width": 640,
            "height": 480,
            "k": [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0],
            "d": [0.1, 0.0, 0.0, 0.0, 0.0],
            "distortion_model": "plumb_bob",
        }}

    result = _NODE_REGISTRY["DepthImageProcessor"]({
        "source": _source(),
        "camera_info_source": info_source,
        "__message_stream_reader__": reader,
    })

    assert result["calibration"]["fx"] == 500.0
    assert result["calibration"]["fy"] == 501.0
    assert result["calibration"]["distortion_model"] == "plumb_bob"
    assert result["depth_stream"]["camera_info_source"] == info_source


def test_depth_processor_rejects_non_image_stream():
    result = _NODE_REGISTRY["DepthImageProcessor"]({
        "source": _source(message_type="sensor_msgs/msg/LaserScan"),
    })

    assert result["depth_stream"] == {}
    assert result["health"]["state"] == "unavailable"


def test_depth_templates_validate():
    roots = [
        _COMPONENT / "templates",
        _COMPONENT / "adapters" / "ros2" / "templates",
    ]
    for root in roots:
        for path in sorted(root.glob("*.json")):
            report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
            assert report.ok, f"{path.name}: {report.to_dict()}"


def test_rgbd_template_subscribes_to_existing_device_topics():
    path = (
        _COMPONENT
        / "adapters"
        / "ros2"
        / "templates"
        / "blacknode-rgbd-camera.json"
    )
    workflow = json.loads(path.read_text(encoding="utf-8"))
    nodes = workflow["node_meta"]

    assert "provider" not in nodes
    assert nodes["rgb"]["type"] == "ROS2"
    assert nodes["rgb"]["params"]["topic"] == "/depth_cam/rgb0/image_raw"
    assert nodes["depth"]["type"] == "ROS2"
    assert nodes["depth"]["params"]["topic"] == "/depth_cam/depth0/image_raw"
    assert nodes["ir"]["type"] == "ROS2"
    assert nodes["ir"]["params"]["topic"] == "/depth_cam/ir0/image_raw"
    assert (
        nodes["depth_camera_info"]["params"]["topic"]
        == "/depth_cam/depth0/camera_info"
    )
    assert nodes["rgb_processor"]["type"] == "CameraImageProcessor"
    assert nodes["depth_processor"]["type"] == "DepthImageProcessor"
    assert nodes["ir_processor"]["type"] == "CameraImageProcessor"
    assert nodes["rgb_out"]["type"] == "CameraViewer"
    assert nodes["depth_out"]["type"] == "DepthViewer"
    assert nodes["ir_out"]["type"] == "CameraViewer"
    assert nodes["depth_projector"]["type"] == "WarpDepthProjector"
    assert nodes["cloud_out"]["type"] == "DepthCloudViewer"
    assert nodes["cloud_out"]["params"]["color_mode"] == "rgb"
    assert {edge["to_port"] for edge in workflow["edges"] if edge["to"] == "cloud_out"} == {
        "source",
        "rgb_source",
        "ir_source",
        "depth_projection",
    }
    assert "blacknode-cuda" in workflow["metadata"]["required_packages"]
