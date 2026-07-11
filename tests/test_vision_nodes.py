"""blacknode-vision package contracts."""
import base64
import json
from pathlib import Path

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.workflow import validate_workflow

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"

EXPECTED_NODES = {
    "CV2ColorObjectTracker": "CV2",
    "CV2HSVMask": "CV2",
    "CV2TrackerPythonExport": "CV2",
    "VisionDetectionPrompt": "Vision",
    "VisionFramePrompt": "Vision",
    "VisionReasoningDashboard": "Vision",
    "VisionStreamStatus": "Vision",
    "VisionVLMDescribe": "Vision",
}


def test_nodes_registered_with_package_and_category():
    for name, category in EXPECTED_NODES.items():
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-vision"
        assert _NODE_REGISTRY[name]._bn_category == category


def test_templates_validate():
    for path in sorted(TEMPLATE_DIR.glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_camera_console_defaults_to_bundled_usb_camera():
    path = TEMPLATE_DIR / "vision-camera-console.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    params = workflow["node_meta"]["camera_run"]["params"]
    assert params["package"] == "blacknode_usb_camera"
    assert params["executable"] == "usb_camera"
    assert "/camera/image_raw" in params["arguments"]


def test_frame_prompt_summarizes_context():
    result = _NODE_REGISTRY["VisionFramePrompt"]({
        "image": "data:image/png;base64,abc",
        "question": "Is the table clear?",
        "context": "bench camera",
        "robot_task": "pick cube",
    })
    assert "Is the table clear?" in result["prompt"]
    assert "pick cube" in result["prompt"]
    assert result["summary"]["has_image"] is True
    assert result["summary"]["image_kind"] == "data-url"


def test_detection_prompt_summarizes_cv2_output():
    result = _NODE_REGISTRY["VisionDetectionPrompt"]({
        "detection": {
            "found": True,
            "label": "cube",
            "center": {"x": 320, "y": 240},
            "area": 1200.0,
        },
        "detections": [{"label": "cube"}],
        "question": "Should the robot move left or right?",
    })
    assert "CV2 detections" in result["prompt"]
    assert '"x": 320' in result["prompt"]
    assert result["summary"]["found"] is True


def test_stream_status_ready_dashboard():
    result = _NODE_REGISTRY["VisionStreamStatus"]({
        "camera_topic": "/camera/image_raw",
        "stream_url": "http://127.0.0.1:9000/stream.mjpg",
        "streaming": True,
    })
    assert result["ready"] is True
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")
    assert "LIVE" in result["report"]


def test_stream_status_wraps_long_dashboard_text():
    long_report = (
        "ROS 2 run process running: blacknode_usb_camera usb_camera; "
        "/camera/image_raw is discoverable via native backend with a long status message"
    )
    result = _NODE_REGISTRY["VisionStreamStatus"]({
        "camera_topic": "/camera/image_raw",
        "stream_url": "http://127.0.0.1:12345/stream.mjpg?with=a-long-query-string-that-would-overflow",
        "streaming": True,
        "run_report": long_report,
        "stream_report": long_report,
    })
    svg = base64.b64decode(result["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert "<tspan" in svg
    assert 'height="380"' not in svg
    assert "/camera/image_raw is discoverable" in svg


def test_reasoning_dashboard_includes_image_and_answer():
    result = _NODE_REGISTRY["VisionReasoningDashboard"]({
        "image": "data:image/jpeg;base64,abc",
        "prompt": "Describe what the robot sees.",
        "answer": "Summary: a workbench is visible. Evidence: flat surface and tools. Next action: wait.",
        "report": "VLM describe OK via test-model",
    })
    svg = base64.b64decode(result["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert result["ready"] is True
    assert "VISIBLE REASONING" in svg
    assert "Summary:" in svg
    assert "data:image/jpeg;base64,abc" in svg


def test_vlm_describe_ollama_text_only(monkeypatch):
    calls = []

    def fake_post_json(url, body, headers, timeout=90.0):
        calls.append({"url": url, "body": body, "headers": headers, "timeout": timeout})
        return {"message": {"content": "move slightly left"}}

    fn = _NODE_REGISTRY["VisionVLMDescribe"]
    monkeypatch.setitem(fn.__globals__, "_post_json", fake_post_json)
    result = fn({
        "image": "",
        "question": "Detection center x is 420. What next?",
        "provider": "ollama",
        "model": "qwen2.5vl:7b",
        "endpoint_url": "http://127.0.0.1:11434",
        "allow_text_only": True,
    })
    assert result["text"] == "move slightly left"
    assert result["report"] == "VLM describe OK via ollama/qwen2.5vl:7b"
    assert calls[0]["url"] == "http://127.0.0.1:11434/api/chat"
    assert calls[0]["body"]["stream"] is False
    assert "images" not in calls[0]["body"]["messages"][-1]


def test_vlm_describe_anthropic_image(monkeypatch):
    calls = []

    def fake_post_json(url, body, headers, timeout=90.0):
        calls.append({"url": url, "body": body, "headers": headers, "timeout": timeout})
        return {"content": [{"type": "text", "text": "A cube is visible."}]}

    fn = _NODE_REGISTRY["VisionVLMDescribe"]
    monkeypatch.setitem(fn.__globals__, "_post_json", fake_post_json)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-anthropic-key")
    result = fn({
        "image": "data:image/png;base64,abc",
        "question": "What do you see?",
        "provider": "anthropic",
        "model": "claude-sonnet-4-5",
        "endpoint_url": "https://api.anthropic.com/v1",
    })
    assert result["text"] == "A cube is visible."
    assert calls[0]["url"] == "https://api.anthropic.com/v1/messages"
    assert calls[0]["headers"]["x-api-key"] == "test-anthropic-key"
    source = calls[0]["body"]["messages"][0]["content"][0]["source"]
    assert source == {"type": "base64", "media_type": "image/png", "data": "abc"}


def test_cv2_tracker_reports_missing_or_detects_green_cube():
    fn = _NODE_REGISTRY["CV2ColorObjectTracker"]
    if fn.__globals__["cv2"] is None:
        result = fn({"image": "data:image/png;base64,abc"})
        assert result["found"] is False
        assert "OpenCV is not installed" in result["report"]
        return

    cv2 = fn.__globals__["cv2"]
    np = fn.__globals__["np"]
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[30:80, 60:110] = (0, 255, 0)
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    source = "data:image/png;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
    result = fn({
        "image": source,
        "label": "cube",
        "lower_hsv": "35,60,60",
        "upper_hsv": "85,255,255",
        "min_area": 100,
    })
    assert result["found"] is True
    assert 75 <= result["center_x"] <= 95
    assert 45 <= result["center_y"] <= 65
    assert result["overlay"].startswith("data:image/jpeg;base64,")


def test_cv2_tracker_python_export_contains_config():
    result = _NODE_REGISTRY["CV2TrackerPythonExport"]({
        "label": "cube",
        "lower_hsv": "35,60,60",
        "upper_hsv": "85,255,255",
        "camera_device": 1,
    })
    assert "CAMERA_DEVICE = 1" in result["source"]
    assert "LOWER_HSV" in result["source"]


def test_vlm_describe_requires_image():
    result = _NODE_REGISTRY["VisionVLMDescribe"]({"image": ""})
    assert result["text"] == ""
    assert "FAILED" in result["report"]


def test_vlm_describe_requires_key_for_remote(monkeypatch):
    monkeypatch.delenv("VISION_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("NVIDIA_API_KEY", raising=False)
    result = _NODE_REGISTRY["VisionVLMDescribe"]({
        "image": "data:image/png;base64,abc",
        "endpoint_url": "https://api.openai.com/v1",
        "api_key": "",
    })
    assert result["text"] == ""
    assert "api_key" in result["report"]
