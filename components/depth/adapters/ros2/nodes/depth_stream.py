"""Managed ROS 2 depth-image preview and metric stream contracts."""
from __future__ import annotations

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
        "stream_id": stream_id,
        "depth_stream": {},
        "point_cloud_stream": {},
        "report": report,
    }


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
        "host": Text(default="127.0.0.1"),
        "port": Int(default=0),
        "max_fps": Float(default=10.0),
        "max_width": Int(default=960),
        "jpeg_quality": Int(default=80),
    },
    outputs={
        "preview": Image,
        "streaming": Bool,
        "stream_url": Text,
        "snapshot_url": Text,
        "health_url": Text,
        "stream_id": Text,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
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
        "stream_id": stream_id,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "report": (
            f"LIVE DEPTH running on {stream_url} from {topic}; metric data "
            f"remains on ROS 2 with scale {depth_scale:g}"
        ),
    }
