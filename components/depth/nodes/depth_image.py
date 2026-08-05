"""Normalize image-bearing message streams into metric depth contracts."""
from __future__ import annotations

import json
import math
import time
import urllib.error
import urllib.request

from blacknode import contracts as bn_contracts
from blacknode.node import Dict, Enum, Float, Image, Text, node


def _read_health(url: str, wait_seconds: float) -> dict:
    if not url:
        return {}
    deadline = time.monotonic() + max(0.0, min(10.0, wait_seconds))
    latest: dict = {}
    while True:
        try:
            request = urllib.request.Request(url, headers={"Accept": "application/json"})
            with urllib.request.urlopen(request, timeout=1.0) as response:
                payload = json.loads(response.read(128 * 1024 + 1).decode("utf-8"))
            latest = payload if isinstance(payload, dict) else {}
            if int(latest.get("frames") or 0) > 0:
                return latest
        except (OSError, TimeoutError, UnicodeDecodeError, json.JSONDecodeError, urllib.error.URLError) as exc:
            latest = {"error": f"{type(exc).__name__}: {exc}"}
        if time.monotonic() >= deadline:
            return latest
        time.sleep(0.1)


def _message(outputs: dict) -> dict:
    value = outputs.get("message") if isinstance(outputs.get("message"), dict) else {}
    return value.get("message") if isinstance(value.get("message"), dict) else value


@node(
    name="DepthImageProcessor",
    category="Perception",
    description=(
        "Process a raw depth image stream from the generic ROS2 node into "
        "metric depth, calibration, health, and optional point-cloud contracts."
    ),
    inputs={
        "source": Dict,
        "camera_info_source": Dict,
        "point_cloud_source": Dict,
        "frame_id": Text(default="camera_depth"),
        "encoding": Enum(["auto", "16UC1", "32FC1"], default="auto"),
        "depth_scale": Float(default=0.001),
        "fx": Float(default=0.0),
        "fy": Float(default=0.0),
        "cx": Float(default=0.0),
        "cy": Float(default=0.0),
        "stale_after_seconds": Float(default=2.0),
        "health_wait_seconds": Float(default=0.0),
    },
    outputs={
        "preview": Image,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
        "health": Dict,
        "calibration": Dict,
        "report": Text,
    },
    primary_inputs=["source", "camera_info_source", "point_cloud_source"],
    primary_outputs=["depth_stream", "point_cloud_stream", "preview", "health"],
)
def depth_image_processor(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    message_type = str(source.get("message_type") or "")
    valid = (
        source.get("kind") == "blacknode.message-stream"
        and source.get("protocol") == "ros2"
        and message_type == "sensor_msgs/msg/Image"
    )
    if not valid:
        return {
            "preview": "",
            "depth_stream": {},
            "point_cloud_stream": {},
            "health": {"state": "unavailable", "source_fresh": False, "error": "connect ROS2.stream carrying sensor_msgs/Image"},
            "calibration": {},
            "report": "Depth image processor needs a raw sensor_msgs/Image ROS2 stream",
        }
    frame_id = str(ctx.get("frame_id") or "camera_depth").strip() or "camera_depth"
    depth_scale = max(0.0, float(ctx.get("depth_scale") or 0.001))
    stale_after = max(0.1, float(ctx.get("stale_after_seconds") or 2.0))
    preview = str(source.get("image") or source.get("stream_url") or source.get("snapshot_url") or "")
    health_payload = _read_health(
        str(source.get("health_url") or ""),
        float(ctx.get("health_wait_seconds") or 0.0),
    )
    metadata = (
        health_payload.get("metadata")
        if isinstance(health_payload.get("metadata"), dict)
        else source.get("metadata")
        if isinstance(source.get("metadata"), dict)
        else {}
    )
    encoding = str(ctx.get("encoding") or "auto")
    if encoding == "auto":
        encoding = str(metadata.get("encoding") or "16UC1")
    summary = metadata.get("depth_summary_raw") if isinstance(metadata.get("depth_summary_raw"), dict) else {}
    received_at = int(metadata.get("received_at_ns") or 0)
    age = max(0.0, (time.time_ns() - received_at) / 1_000_000_000.0) if received_at else None
    frames = int(health_payload.get("frames") or (1 if source.get("image") else 0))
    fresh = bool(frames and (age is None or age <= stale_after) and not health_payload.get("error"))
    health = {
        "state": "ready" if fresh else "stale" if frames else "waiting",
        "worker_alive": bool(source.get("state") not in {"error", "unavailable", "stopped"}),
        "source_fresh": fresh,
        "frames": frames,
        "age_seconds": age,
        "encoding": encoding,
        "summary_m": {
            key: float(value) * depth_scale
            for key, value in summary.items()
            if key in {"minimum", "p05", "median", "p95"}
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
        } | {
            key: int(summary.get(key) or 0)
            for key in ("valid_count", "total_count")
            if key in summary
        },
        "error": str(health_payload.get("error") or ""),
    }
    reader = ctx.get("__message_stream_reader__")
    camera_info = {}
    info_source = ctx.get("camera_info_source") if isinstance(ctx.get("camera_info_source"), dict) else {}
    if info_source and callable(reader):
        camera_info = _message(reader(info_source))
    k = camera_info.get("k") if isinstance(camera_info.get("k"), list) else []
    width = max(0, int(camera_info.get("width") or metadata.get("width") or 0))
    height = max(0, int(camera_info.get("height") or metadata.get("height") or 0))
    fx = max(0.0, float(ctx.get("fx") or (k[0] if len(k) > 0 else 0.0)))
    fy = max(0.0, float(ctx.get("fy") or (k[4] if len(k) > 4 else 0.0)))
    cx = float(ctx.get("cx") or (k[2] if len(k) > 2 else ((width - 1) * 0.5 if width else 0.0)))
    cy = float(ctx.get("cy") or (k[5] if len(k) > 5 else ((height - 1) * 0.5 if height else 0.0)))
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
        "distortion_model": str(camera_info.get("distortion_model") or "none"),
        "distortion": list(camera_info.get("d") or []),
        "ready": bool(width and height and fx > 0.0 and fy > 0.0),
    }
    depth = bn_contracts.depth_stream(
        frame_id,
        snapshot_url=str(source.get("snapshot_url") or source.get("image") or ""),
        encoding=encoding,
        depth_scale=depth_scale,
    )
    depth.update({
        "stream_url": str(source.get("stream_url") or ""),
        "health_url": str(source.get("health_url") or ""),
        "topic": str(source.get("topic") or ""),
        "calibration": calibration,
        "health": health,
        "frame_source": {
            "kind": "blacknode.depth-frame-source",
            "schema_version": 1,
            "transport": "http-binary",
            "url": str(source.get("frame_url") or ""),
            "frame": frame_id,
            "encoding": encoding,
            "depth_scale": depth_scale,
        },
    })
    if health["summary_m"]:
        depth["summary_m"] = health["summary_m"]
    point_source = ctx.get("point_cloud_source") if isinstance(ctx.get("point_cloud_source"), dict) else {}
    points = {}
    if point_source.get("kind") == "blacknode.message-stream":
        points = bn_contracts.point_cloud_stream(frame_id, source_url=f"ros2://{point_source.get('topic') or ''}")
        points["source"] = point_source
    return {
        "preview": preview,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "health": health,
        "calibration": calibration,
        "report": f"depth image processor {health['state']} for {source.get('topic') or '(topic not set)'}",
    }
