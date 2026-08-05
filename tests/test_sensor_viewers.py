import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY


def test_camera_viewer_uses_processed_frame_stream():
    result = _NODE_REGISTRY["CameraViewer"]({
        "source": {
            "kind": "blacknode.frame-stream",
            "stream_url": "http://robot.local/camera.mjpg",
        },
        "health": {"state": "ready", "source_fresh": True},
    })

    assert result["preview"].endswith("camera.mjpg")
    assert result["status"]["viewer_role"] == "camera"
    assert result["status"]["state"] == "ready"


def test_depth_viewer_uses_processed_metric_depth_stream():
    result = _NODE_REGISTRY["DepthViewer"]({
        "source": {
            "kind": "blacknode.depth-stream",
            "snapshot_url": "http://robot.local/depth.png",
            "encoding": "16UC1",
            "depth_scale": 0.001,
        },
        "health": {"state": "ready", "source_fresh": True},
    })

    assert result["preview"].endswith("depth.png")
    assert result["status"]["viewer_role"] == "depth"
    assert result["status"]["depth_scale"] == 0.001


def test_depth_viewer_exposes_raw_frame_for_local_display_controls():
    result = _NODE_REGISTRY["DepthViewer"]({
        "source": {
            "kind": "blacknode.depth-stream",
            "stream_url": "http://robot.local/stream.mjpg?token=one",
            "encoding": "16UC1",
            "depth_scale": 0.001,
            "frame_source": {
                "kind": "blacknode.depth-frame-source",
                "transport": "http-binary",
                "url": "http://robot.local/frame.bin",
            },
        },
        "health": {"state": "ready", "source_fresh": True},
        "auto_range": False,
        "near_m": 0.25,
        "far_m": 1.75,
        "palette": "turbo",
        "invalid_color": "magenta",
    })

    assert result["preview"] == "http://robot.local/stream.mjpg?token=one"
    assert result["status"]["frame_url"] == "http://robot.local/frame.bin"
    assert result["status"]["display"] == {
        "range": "fixed",
        "near_m": 0.25,
        "far_m": 1.75,
        "palette": "turbo",
        "invalid_color": "magenta",
    }
