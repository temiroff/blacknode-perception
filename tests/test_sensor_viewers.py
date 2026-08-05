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


def test_depth_viewer_adds_fixed_metric_display_controls_to_preview_url():
    result = _NODE_REGISTRY["DepthViewer"]({
        "source": {
            "kind": "blacknode.depth-stream",
            "stream_url": "http://robot.local/stream.mjpg?token=one",
            "encoding": "16UC1",
            "depth_scale": 0.001,
        },
        "health": {"state": "ready", "source_fresh": True},
        "auto_range": False,
        "near_m": 0.25,
        "far_m": 1.75,
        "palette": "turbo",
        "invalid_color": "magenta",
    })

    preview = result["preview"]
    assert "token=one" in preview
    assert "depth_range=fixed" in preview
    assert "depth_scale=0.001" in preview
    assert "depth_near_m=0.25" in preview
    assert "depth_far_m=1.75" in preview
    assert "depth_palette=turbo" in preview
    assert "depth_invalid=magenta" in preview
    assert result["status"]["display"] == {
        "range": "fixed",
        "near_m": 0.25,
        "far_m": 1.75,
        "palette": "turbo",
        "invalid_color": "magenta",
    }
