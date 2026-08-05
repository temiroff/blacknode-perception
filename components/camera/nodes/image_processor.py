"""Normalize image-bearing message streams for camera consumers."""
from __future__ import annotations

from blacknode.node import Bool, Dict, Image, Text, node


@node(
    name="CameraImageProcessor",
    category="Perception",
    description=(
        "Process an image stream from the generic ROS2 node into a portable "
        "camera frame stream and preview."
    ),
    inputs={"source": Dict},
    outputs={
        "preview": Image,
        "ready": Bool,
        "stream_url": Text,
        "snapshot_url": Text,
        "frame_stream": Dict,
        "health": Dict,
        "report": Text,
    },
    primary_inputs=["source"],
    primary_outputs=["frame_stream", "preview", "health"],
)
def camera_image_processor(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    message_type = str(source.get("message_type") or "")
    valid = (
        source.get("kind") == "blacknode.message-stream"
        and source.get("protocol") == "ros2"
        and message_type in {
            "sensor_msgs/msg/Image",
            "sensor_msgs/msg/CompressedImage",
        }
    )
    preview = str(
        source.get("image")
        or source.get("stream_url")
        or source.get("snapshot_url")
        or ""
    )
    state = str(source.get("state") or ("ready" if preview else "unavailable"))
    ready = bool(valid and preview and state not in {"error", "unavailable", "stopped"})
    health = {
        "kind": "blacknode.stream-status",
        "schema_version": 1,
        "state": "ready" if ready else state,
        "available": valid,
        "source_fresh": bool(ready and (source.get("image") or state == "ready")),
        "error": "" if valid else "connect an image-bearing ROS2.stream",
    }
    if not valid:
        return {
            "preview": "",
            "ready": False,
            "stream_url": "",
            "snapshot_url": "",
            "frame_stream": {},
            "health": health,
            "report": "Camera image processor needs ROS2.stream carrying sensor_msgs/Image",
        }
    frame_stream = {
        "kind": "blacknode.frame-stream",
        "schema_version": 1,
        "stream_id": str(source.get("stream_id") or "camera"),
        "stream_url": str(source.get("stream_url") or ""),
        "snapshot_url": str(source.get("snapshot_url") or source.get("image") or ""),
        "health_url": str(source.get("health_url") or ""),
        "media_type": str(source.get("media_type") or "image/jpeg"),
        "mode": "latest",
        "clock": "unix_ns",
        "topic": str(source.get("topic") or ""),
        "message_type": message_type,
        "backend": str(source.get("backend") or ""),
        "device_id": str(source.get("device_id") or ""),
    }
    return {
        "preview": preview,
        "ready": ready,
        "stream_url": frame_stream["stream_url"],
        "snapshot_url": frame_stream["snapshot_url"],
        "frame_stream": frame_stream,
        "health": health,
        "report": (
            f"camera image processor ready for {frame_stream['topic']}"
            if ready
            else f"camera image processor waiting for {frame_stream['topic']}"
        ),
    }
