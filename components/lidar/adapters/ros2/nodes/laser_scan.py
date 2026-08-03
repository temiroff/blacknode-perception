"""ROS 2 LaserScan capture and managed Warp viewer for the LiDAR capability."""
from __future__ import annotations

import copy
import math
import time
from pathlib import Path
from typing import Any

from blacknode import contracts as bn_contracts
from blacknode.node import Bool, Dict, Enum, Float, Int, Text, node


_CATEGORY = "Perception"
_LASER_SCAN_TYPE = "sensor_msgs/msg/LaserScan"


def _runtime():
    # Keep package discovery functional when blacknode-ros2 is absent or has
    # not been loaded yet. The adapter dependency is resolved when used.
    from blacknode.pkg.blacknode_ros2 import ros2_runtime

    return ros2_runtime


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def normalize_laser_scan(payload: Any, *, topic: str = "/scan") -> dict:
    envelope = payload if isinstance(payload, dict) else {}
    message = envelope.get("message") if isinstance(envelope.get("message"), dict) else envelope
    ranges_value = message.get("ranges") if isinstance(message.get("ranges"), list) else []
    ranges = [_finite(value, float("nan")) for value in ranges_value[:100_000]]
    header = message.get("header") if isinstance(message.get("header"), dict) else {}
    stamp = header.get("stamp") if isinstance(header.get("stamp"), dict) else {}
    source_time_ns = int(stamp.get("sec") or 0) * 1_000_000_000 + int(stamp.get("nanosec") or 0)
    frame_id = str(header.get("frame_id") or "laser").strip()
    scan = bn_contracts.laser_scan_stream(
        frame_id,
        angle_min=_finite(message.get("angle_min"), -math.pi),
        angle_max=_finite(message.get("angle_max"), math.pi),
        angle_increment=_finite(message.get("angle_increment"), 0.0),
        range_min=max(0.0, _finite(message.get("range_min"), 0.0)),
        range_max=max(0.0, _finite(message.get("range_max"), 0.0)),
        ranges=ranges,
    )
    scan.update({
        "topic": topic,
        "message_type": _LASER_SCAN_TYPE,
        "source_time_ns": source_time_ns or time.time_ns(),
        "receive_time_ns": time.time_ns(),
        "scan_time": max(0.0, _finite(message.get("scan_time"), 0.0)),
        "time_increment": max(0.0, _finite(message.get("time_increment"), 0.0)),
        "intensities": list(message.get("intensities") or [])[:100_000],
    })
    return scan


def _blank(report: str) -> dict:
    return {
        "captured": False,
        "provider_state": {
            "kind": "blacknode.provider-state",
            "schema_version": 1,
            "provider_id": "ros2_lidar",
            "provider": {
                "package": "blacknode-perception",
                "component": "lidar",
                "adapter": "ros2",
            },
            "state": "unavailable",
            "available": False,
            "ready": False,
        },
        "laser_scan": {},
        "health": {
            "state": "unavailable",
            "worker_alive": False,
            "source_fresh": False,
            "sample_count": 0,
            "valid_count": 0,
            "error": report,
        },
        "hardware": {},
        "report": report,
    }


@node(
    name="LiDARROS2Scan",
    category=_CATEGORY,
    description="Capture and normalize one sensor_msgs/LaserScan message from a ROS 2 topic.",
    inputs={
        "topic": Text(default="/scan"),
        "timeout_seconds": Float(default=10.0),
        "hardware_id": Text(default=""),
    },
    outputs={
        "captured": Bool,
        "provider_state": Dict,
        "laser_scan": Dict,
        "health": Dict,
        "hardware": Dict,
        "report": Text,
    },
)
def lidar_ros2_scan(ctx: dict) -> dict:
    rt = _runtime()
    topic = str(ctx.get("topic") or "/scan").strip()
    timeout = max(0.1, min(120.0, float(ctx.get("timeout_seconds") or 10.0)))
    result = rt.run_topic_subscriber_once(
        topic=topic,
        message_type=_LASER_SCAN_TYPE,
        node_name="blacknode_lidar_scan_once",
        timeout=timeout,
    )
    if not result.get("ok") or not isinstance(result.get("latest"), dict):
        return _blank(
            "LaserScan capture failed: "
            + str(result.get("error") or f"no message received from {topic}")
        )
    scan = normalize_laser_scan(result["latest"], topic=topic)
    valid = [
        value for value in scan["ranges"]
        if isinstance(value, (int, float)) and math.isfinite(value)
        and scan["range_min"] <= value <= scan["range_max"]
    ]
    health = {
        "state": "ready",
        "worker_alive": True,
        "source_fresh": True,
        "scans": 1,
        "age_seconds": max(0.0, (time.time_ns() - scan["receive_time_ns"]) / 1e9),
        "sample_count": len(scan["ranges"]),
        "valid_count": len(valid),
        "minimum_m": min(valid) if valid else None,
        "maximum_m": max(valid) if valid else None,
        "error": "",
    }
    hardware_id = str(ctx.get("hardware_id") or "").strip()
    hardware = ({"id": hardware_id, "serial": hardware_id, "kind": "lidar"} if hardware_id else {})
    provider_state = {
        "kind": "blacknode.provider-state",
        "schema_version": 1,
        "provider_id": f"ros2:{topic}",
        "provider": {
            "package": "blacknode-perception",
            "component": "lidar",
            "adapter": "ros2",
        },
        "state": "ready",
        "available": True,
        "ready": True,
        "backend": result.get("backend", ""),
        "health": copy.deepcopy(health),
    }
    return {
        "captured": True,
        "provider_state": provider_state,
        "laser_scan": scan,
        "health": health,
        "hardware": hardware,
        "report": f"captured {len(valid)} valid points from {topic} in frame {scan['frame']}",
    }


@node(
    name="LiDARROS2WarpViewer",
    hidden=True,
    category=_CATEGORY,
    description=(
        "Start or stop one native ROS 2 LaserScan subscription rendered through "
        "NVIDIA Warp's interactive OpenGL point viewer."
    ),
    inputs={
        "action": Enum(["stop", "start"], default="stop"),
        "topic": Text(default="/scan"),
        "viewer_id": Text(default="front_lidar_warp"),
        "device": Enum(["cuda:0", "cpu"], default="cuda:0"),
        "filter_min_m": Float(default=0.1),
        "filter_max_m": Float(default=12.0),
        "downsample_stride": Int(default=1),
        "sensor_x_m": Float(default=0.0),
        "sensor_y_m": Float(default=0.0),
        "sensor_yaw_rad": Float(default=0.0),
        "show_raw": Bool(default=True),
        "show_filtered": Bool(default=True),
        "point_radius_m": Float(default=0.025),
        "fps": Int(default=30),
    },
    outputs={
        "running": Bool,
        "source_ready": Bool,
        "viewer": Dict,
        "report": Text,
    },
)
def lidar_ros2_warp_viewer(ctx: dict) -> dict:
    rt = _runtime()
    viewer_id = str(ctx.get("viewer_id") or "front_lidar_warp").strip()
    topic = str(ctx.get("topic") or "/scan").strip()
    if str(ctx.get("action") or "stop") == "stop":
        stopped = rt.stop_ros2_python_node(viewer_id)
        return {
            "running": False,
            "source_ready": False,
            "viewer": {"viewer_id": viewer_id, "topic": topic, "state": "stopped"},
            "report": f"Warp LiDAR viewer stopped {int(stopped.get('stopped') or 0)} process(es)",
        }

    backend = rt.detect_backend()
    if backend.get("backend") != "native":
        reason = (
            "the Warp OpenGL viewer requires native ROS 2 in this graphical session; "
            + str(backend.get("error") or "the active ROS backend is not native")
        )
        return {
            "running": False,
            "source_ready": False,
            "viewer": {"viewer_id": viewer_id, "topic": topic, "state": "unavailable", "error": reason},
            "report": reason,
        }
    interface = rt.inspect_topic_interfaces([{
        "name": "laser_scan",
        "topic": topic,
        "message_type": _LASER_SCAN_TYPE,
        "required": True,
    }])
    if not interface.get("ready"):
        reason = str(interface.get("error") or f"{topic} has no LaserScan publisher")
        return {
            "running": False,
            "source_ready": False,
            "viewer": {"viewer_id": viewer_id, "topic": topic, "state": "unavailable", "error": reason},
            "report": reason,
        }
    script = Path(__file__).resolve().parents[1] / "scripts" / "ros2_warp_lidar_viewer.py"
    arguments = [
        "--topic", topic,
        "--device", str(ctx.get("device") or "cuda:0"),
        "--filter-min", str(max(0.0, float(ctx.get("filter_min_m") or 0.1))),
        "--filter-max", str(max(0.0, float(ctx.get("filter_max_m") or 12.0))),
        "--stride", str(max(1, int(ctx.get("downsample_stride") or 1))),
        "--sensor-x", str(float(ctx.get("sensor_x_m") or 0.0)),
        "--sensor-y", str(float(ctx.get("sensor_y_m") or 0.0)),
        "--sensor-yaw", str(float(ctx.get("sensor_yaw_rad") or 0.0)),
        "--point-radius", str(max(0.001, float(ctx.get("point_radius_m") or 0.025))),
        "--fps", str(max(1, min(120, int(ctx.get("fps") or 30)))),
    ]
    if ctx.get("show_raw", True):
        arguments.append("--show-raw")
    if ctx.get("show_filtered", True):
        arguments.append("--show-filtered")
    started = rt.start_ros2_python_node(
        run_id=viewer_id,
        source_mode="file",
        script_path=str(script),
        code="",
        arguments=arguments,
    )
    if not started.get("ok"):
        reason = str(started.get("error") or "could not start Warp LiDAR viewer")
        return {
            "running": False,
            "source_ready": True,
            "viewer": {"viewer_id": viewer_id, "topic": topic, "state": "failed", "error": reason},
            "report": reason,
        }
    viewer = {
        "kind": "blacknode.lidar-viewer",
        "schema_version": 1,
        "viewer_id": viewer_id,
        "topic": topic,
        "message_type": _LASER_SCAN_TYPE,
        "device": str(ctx.get("device") or "cuda:0"),
        "state": "running",
        "backend": started.get("backend", "native"),
        "controls": {"space": "cycle raw / filtered / both", "escape": "close"},
    }
    return {
        "running": True,
        "source_ready": True,
        "viewer": viewer,
        "report": f"Warp LiDAR viewer running from {topic}; Space cycles raw, filtered, and both",
    }
