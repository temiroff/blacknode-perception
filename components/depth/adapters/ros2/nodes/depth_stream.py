"""Managed ROS 2 depth-image preview and metric stream contracts."""
from __future__ import annotations

import json
import math
import time
import urllib.error
import urllib.request

from blacknode import contracts as bn_contracts
from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, Text, node


class _LazyRos2Runtime:
    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import ros2_runtime

        return getattr(ros2_runtime, name)


rt = _LazyRos2Runtime()
_CATEGORY = "Perception"


def _blank(stream_id: str, report: str) -> dict:
    return {
        "preview": "",
        "streaming": False,
        "stream_url": "",
        "snapshot_url": "",
        "health_url": "",
        "frame_url": "",
        "stream_id": stream_id,
        "depth_stream": {},
        "point_cloud_stream": {},
        "health": {
            "state": "unavailable",
            "worker_alive": False,
            "source_fresh": False,
            "frames": 0,
            "age_seconds": None,
            "summary_m": {},
            "error": report,
        },
        "report": report,
    }


def _metric_health(
    payload: dict,
    *,
    depth_scale: float,
    stale_after_seconds: float,
) -> dict:
    metadata = (
        payload.get("metadata")
        if isinstance(payload.get("metadata"), dict)
        else {}
    )
    raw_summary = (
        metadata.get("depth_summary_raw")
        if isinstance(metadata.get("depth_summary_raw"), dict)
        else {}
    )
    encoding = str(
        raw_summary.get("encoding")
        or metadata.get("encoding")
        or ""
    ).strip()
    scale = 1.0 if encoding.lower() == "32fc1" else max(0.0, depth_scale)
    summary_m: dict[str, float | int] = {}
    for name in ("minimum", "p05", "median", "p95"):
        try:
            value = float(raw_summary.get(name))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            summary_m[name] = value * scale
    for name in ("valid_count", "total_count"):
        try:
            summary_m[name] = max(0, int(raw_summary.get(name)))
        except (TypeError, ValueError):
            continue
    received_at_ns = int(metadata.get("received_at_ns") or 0)
    age_seconds = (
        max(0.0, (time.time_ns() - received_at_ns) / 1_000_000_000.0)
        if received_at_ns > 0
        else None
    )
    frames = max(0, int(payload.get("frames") or 0))
    error = str(payload.get("error") or "").strip()
    worker_alive = not error
    source_fresh = bool(
        frames > 0
        and age_seconds is not None
        and age_seconds <= max(0.1, stale_after_seconds)
    )
    state = (
        "error"
        if error
        else "ready"
        if source_fresh and summary_m.get("valid_count", 0) > 0
        else "stale"
        if frames > 0
        else "waiting"
    )
    return {
        "state": state,
        "worker_alive": worker_alive,
        "source_fresh": source_fresh,
        "frames": frames,
        "age_seconds": age_seconds,
        "encoding": encoding,
        "frame_id": str(metadata.get("frame_id") or ""),
        "summary_m": summary_m,
        "error": error,
    }


def _read_stream_health(url: str, wait_seconds: float) -> dict:
    clean_url = str(url or "").strip()
    if not clean_url:
        return {}
    deadline = time.monotonic() + max(0.0, min(10.0, wait_seconds))
    while True:
        try:
            request = urllib.request.Request(
                clean_url,
                headers={"Accept": "application/json"},
            )
            with urllib.request.urlopen(request, timeout=1.0) as response:
                raw = response.read(128 * 1024 + 1)
            if len(raw) > 128 * 1024:
                return {"error": "depth health response exceeded 128 KB"}
            payload = json.loads(raw.decode("utf-8"))
            if isinstance(payload, dict):
                if int(payload.get("frames") or 0) > 0:
                    return payload
                latest = payload
            else:
                latest = {"error": "depth health response was not an object"}
        except (
            OSError,
            TimeoutError,
            UnicodeDecodeError,
            json.JSONDecodeError,
            urllib.error.URLError,
        ) as exc:
            latest = {"error": f"{type(exc).__name__}: {exc}"}
        if time.monotonic() >= deadline:
            return latest
        time.sleep(0.1)


@node(
    name="DepthROS2Subscribe",
    live=True,
    category=_CATEGORY,
    description=(
        "Preview a ROS 2 depth image while preserving its metric depth and "
        "optional point-cloud interfaces for downstream workflows."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "stream_id": Text(default="depth_camera"),
        "topic": Text(default="/camera/depth/image_raw"),
        "camera_info_topic": Text(default="/camera/depth/camera_info"),
        "points_topic": Text(default=""),
        "frame_id": Text(default="camera_depth"),
        "encoding": Enum(["auto", "16UC1", "32FC1"], default="auto"),
        "depth_scale": Float(default=0.001),
        "fx": Float(default=0.0),
        "fy": Float(default=0.0),
        "cx": Float(default=0.0),
        "cy": Float(default=0.0),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=0),
        "max_fps": Float(default=10.0),
        "max_width": Int(default=960),
        "jpeg_quality": Int(default=80),
        "stale_after_seconds": Float(default=2.0),
        "health_wait_seconds": Float(default=2.0),
    },
    outputs={
        "preview": Image,
        "streaming": Bool,
        "stream_url": Text,
        "snapshot_url": Text,
        "health_url": Text,
        "frame_url": Text,
        "stream_id": Text,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
        "health": Dict,
        "report": Text,
    },
)
def ros2_depth_stream(ctx: dict) -> dict:
    stream_id = str(ctx.get("stream_id") or "depth_camera").strip()
    stream_id = stream_id or "depth_camera"
    action = str(ctx.get("action") or "start").strip().lower()
    if action == "stop":
        result = rt.stop_image_stream(stream_id)
        return _blank(
            stream_id,
            f"stopped {result.get('stopped', 0)} depth preview stream(s)",
        )

    topic = str(
        ctx.get("topic") or "/camera/depth/image_raw"
    ).strip()
    interface = rt.inspect_topic_interfaces([{
        "name": "depth_image",
        "topic": topic,
        "message_type": "sensor_msgs/msg/Image",
        "required": True,
    }])
    if not interface.get("ok") or not interface.get("ready"):
        missing = ", ".join(interface.get("missing") or [])
        return _blank(
            stream_id,
            "depth stream FAILED: "
            + str(interface.get("error") or missing or f"{topic} is not publishing"),
        )

    host = str(ctx.get("host") or "127.0.0.1").strip() or "127.0.0.1"
    port = max(0, int(ctx.get("port") or 0))
    max_fps = max(0.1, min(60.0, float(ctx.get("max_fps") or 10.0)))
    max_width = max(0, int(ctx.get("max_width") or 960))
    jpeg_quality = max(1, min(100, int(ctx.get("jpeg_quality") or 80)))
    frame_id = str(ctx.get("frame_id") or "depth_camera_link").strip()
    requested_encoding = str(ctx.get("encoding") or "auto").strip()
    depth_scale = max(0.0, float(ctx.get("depth_scale") or 0.001))
    stale_after_seconds = max(
        0.1,
        float(ctx.get("stale_after_seconds") or 2.0),
    )

    if ctx.get("__run_mode__") == "once":
        shot = rt.capture_image_snapshot(
            topic=topic,
            message_type="raw",
            timeout=15.0,
            output_format="jpeg",
            jpeg_quality=jpeg_quality,
        )
        if not shot.get("ok"):
            return _blank(
                stream_id,
                "depth frame FAILED: "
                + str(shot.get("error") or "unknown error"),
            )
        metadata = dict(shot.get("metadata") or {})
        encoding = (
            str(metadata.get("encoding") or requested_encoding)
            if requested_encoding == "auto"
            else requested_encoding
        )
        result = _blank(
            stream_id,
            f"captured one depth frame from {topic}; press Go Live to stream",
        )
        result["preview"] = str(shot.get("image") or "")
        result["depth_stream"] = bn_contracts.depth_stream(
            str(metadata.get("frame_id") or frame_id),
            snapshot_url=result["preview"],
            encoding=encoding or "16UC1",
            depth_scale=depth_scale,
        )
        health = _metric_health(
            {
                "frames": 1,
                "metadata": metadata,
                "error": "",
            },
            depth_scale=depth_scale,
            stale_after_seconds=stale_after_seconds,
        )
        result["health"] = health
        result["depth_stream"]["health"] = health
        if health["summary_m"]:
            result["depth_stream"]["summary_m"] = health["summary_m"]
        return result

    started = rt.start_image_stream(
        stream_id=stream_id,
        topic=topic,
        message_type="raw",
        host=host,
        port=port,
        max_fps=max_fps,
        max_width=max_width,
        jpeg_quality=jpeg_quality,
    )
    if not started.get("ok"):
        return _blank(
            stream_id,
            "depth stream FAILED: "
            + str(started.get("error") or "could not start preview"),
        )
    stream_url = str(started.get("stream_url") or "")
    snapshot_url = str(started.get("snapshot_url") or "")
    health_url = str(started.get("health_url") or "")
    encoding = "16UC1" if requested_encoding == "auto" else requested_encoding
    depth = bn_contracts.depth_stream(
        frame_id,
        snapshot_url=snapshot_url,
        encoding=encoding,
        depth_scale=depth_scale,
    )
    depth.update({
        "stream_url": stream_url,
        "health_url": health_url,
        "topic": topic,
        "camera_info_topic": str(ctx.get("camera_info_topic") or "").strip(),
    })
    health_payload = _read_stream_health(
        health_url,
        max(0.0, float(ctx.get("health_wait_seconds") or 0.0)),
    )
    health = _metric_health(
        health_payload,
        depth_scale=depth_scale,
        stale_after_seconds=stale_after_seconds,
    )
    depth["health"] = health
    frame_url = str(started.get("frame_url") or "")
    metadata = (
        health_payload.get("metadata")
        if isinstance(health_payload.get("metadata"), dict)
        else {}
    )
    width = max(0, int(metadata.get("width") or 0))
    height = max(0, int(metadata.get("height") or 0))
    fx = max(0.0, float(ctx.get("fx") or 0.0))
    fy = max(0.0, float(ctx.get("fy") or 0.0))
    cx = float(ctx.get("cx") or ((width - 1) * 0.5 if width else 0.0))
    cy = float(ctx.get("cy") or ((height - 1) * 0.5 if height else 0.0))
    calibration = {
        "kind": "blacknode.camera-calibration",
        "schema_version": 1,
        "camera_model": "pinhole",
        "width": width,
        "height": height,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "distortion_model": "none",
        "distortion": [],
        "camera_info_topic": str(ctx.get("camera_info_topic") or "").strip(),
        "ready": bool(width and height and fx > 0.0 and fy > 0.0),
    }
    depth.update({
        "calibration": calibration,
        "frame_source": {
            "kind": "blacknode.depth-frame-source",
            "schema_version": 1,
            "transport": "http-binary",
            "url": frame_url,
            "frame": frame_id,
            "encoding": encoding,
            "depth_scale": depth_scale,
        },
    })
    if health["summary_m"]:
        depth["summary_m"] = health["summary_m"]
    points_topic = str(ctx.get("points_topic") or "").strip()
    points = (
        bn_contracts.point_cloud_stream(
            frame_id,
            source_url=f"ros2://{points_topic}",
        )
        if points_topic
        else {}
    )
    if points:
        points["topic"] = points_topic
    return {
        "preview": stream_url,
        "streaming": True,
        "stream_url": stream_url,
        "snapshot_url": snapshot_url,
        "health_url": health_url,
        "frame_url": frame_url,
        "stream_id": stream_id,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "health": health,
        "report": (
            f"LIVE DEPTH running on {stream_url} from {topic}; "
            f"source {health['state']}; metric data remains on ROS 2 with "
            f"scale {depth_scale:g}"
        ),
    }
