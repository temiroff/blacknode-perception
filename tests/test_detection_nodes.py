"""blacknode-perception — detection component (no camera, no ROS).

Runtime spawning is monkeypatched, so the node's contract handling is exercised
pure: it must consume a frame_stream, resolve a snapshot source, and report
clearly when nothing is wired.
"""
import blacknode  # noqa: F401  triggers discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_perception.detection import detection_runtime as rt


def test_detection_node_registered():
    assert "DetectionStream" in _NODE_REGISTRY
    fn = _NODE_REGISTRY["DetectionStream"]
    assert fn._bn_category == "Detection"
    assert fn._bn_component == "detection"
    assert "frame_stream" in fn._bn_inputs and "frame_stream" in fn._bn_outputs


def test_detection_consumes_a_wired_frame_stream(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "start_detection_stream",
                        lambda **k: captured.update(k) or {
                            "ok": True, "stream_url": "http://127.0.0.1:9/stream.mjpg",
                            "snapshot_url": "http://127.0.0.1:9/snapshot.jpg",
                            "detection_url": "http://127.0.0.1:9/detection.json"})

    result = _NODE_REGISTRY["DetectionStream"]({
        "action": "start", "mode": "object",
        "frame_stream": {"kind": "blacknode.frame-stream",
                         "stream_url": "http://127.0.0.1:5/stream.mjpg",
                         "snapshot_url": "http://127.0.0.1:5/snapshot.jpg"},
    })

    # Detects on the per-frame snapshot, not the blocking MJPEG stream.
    assert captured["source_url"] == "http://127.0.0.1:5/snapshot.jpg"
    assert result["streaming"] is True
    assert result["detection_stream"]["kind"] == "blacknode.detection-stream"


def test_detection_explains_an_unwired_input(monkeypatch):
    monkeypatch.setattr(rt, "start_detection_stream",
                        lambda **k: (_ for _ in ()).throw(AssertionError("must not start without a source")))
    result = _NODE_REGISTRY["DetectionStream"]({"action": "start"})
    assert result["streaming"] is False
    assert "frame_stream" in result["report"]
