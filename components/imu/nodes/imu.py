"""Provider-neutral IMU contracts and live viewer."""
from __future__ import annotations

import copy
import math
from typing import Any

from blacknode.node import Bool, Dict, Enum, Float, Text, node

from . import imu_runtime


runtime_status = imu_runtime.runtime_status
stop_runtime_services = imu_runtime.stop_runtime_services

_CATEGORY = "Perception"
_CAPABILITY_KIND = "blacknode.imu-capability"


def _health(imu: dict[str, Any], *, fresh: bool, error: str = "") -> dict[str, Any]:
    orientation = imu.get("orientation") if isinstance(imu.get("orientation"), dict) else {}
    return {
        "state": "ready" if fresh else "unavailable",
        "worker_alive": fresh,
        "source_fresh": fresh,
        "samples": 1 if fresh else 0,
        "age_seconds": 0.0 if fresh else None,
        "orientation_available": bool(orientation),
        "error": error,
    }


@node(
    name="IMUProcessor",
    category=_CATEGORY,
    description=(
        "Process a generic ROS2 IMU stream into a normalized sample while "
        "preserving its managed stream for the live orientation viewer."
    ),
    inputs={"source": Dict},
    outputs={"stream": Dict, "imu": Dict, "health": Dict, "report": Text},
    primary_inputs=["source"],
    primary_outputs=["stream", "imu", "health"],
)
def imu_processor(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    valid = (
        source.get("kind") == "blacknode.message-stream"
        and source.get("protocol") == "ros2"
        and source.get("message_type") == "sensor_msgs/msg/Imu"
    )
    stream = copy.deepcopy(source) if valid else {}
    if stream:
        stream["processor"] = "IMUProcessor"
    sample, status = imu_runtime._normalize_source(
        source,
        ctx.get("__message_stream_reader__"),
        1.0,
    ) if valid else ({}, {"state": "unavailable", "source_fresh": False})
    health = _health(
        sample,
        fresh=bool(status.get("source_fresh") and sample),
        error="" if valid else "connect ROS2.stream carrying sensor_msgs/msg/Imu",
    )
    return {
        "stream": stream,
        "imu": sample,
        "health": health,
        "report": (
            f"IMU processor ready for {source.get('topic') or '(topic not set)'}"
            if valid
            else "IMU processor needs a generic ROS2 IMU stream"
        ),
    }


@node(
    name="IMU",
    category=_CATEGORY,
    description="Expose a replaceable IMU capability with normalized orientation, motion, health, and attachment metadata.",
    inputs={
        "provider_state": Dict,
        "imu": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_id": Text(default="body_imu"),
        "topic": Text(default="/imu/data"),
        "frame_id": Text(default="imu_link"),
        "stale_after_seconds": Float(default=1.0),
    },
    outputs={
        "ready": Bool,
        "imu_capability": Dict,
        "imu": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_configuration": Dict,
        "report": Text,
    },
    primary_inputs=["provider_state", "imu", "health", "attachment_id"],
    primary_outputs=["imu_capability", "imu", "health", "report"],
)
def imu(ctx: dict) -> dict:
    sample = copy.deepcopy(ctx.get("imu")) if isinstance(ctx.get("imu"), dict) else {}
    provider_state = copy.deepcopy(ctx.get("provider_state")) if isinstance(ctx.get("provider_state"), dict) else {}
    health = copy.deepcopy(ctx.get("health")) if isinstance(ctx.get("health"), dict) else _health(
        sample, fresh=False, error="IMU provider has no health status",
    )
    hardware = copy.deepcopy(ctx.get("hardware")) if isinstance(ctx.get("hardware"), dict) else {}
    correct_kind = sample.get("kind") == "blacknode.imu-stream"
    ready = bool(correct_kind and health.get("source_fresh") and provider_state.get("available", True))
    if not correct_kind:
        health.update(state="unavailable", source_fresh=False, error="Provider did not supply blacknode.imu-stream")
        ready = False
    attachment_id = str(ctx.get("attachment_id") or "body_imu").strip() or "body_imu"
    topic = str(ctx.get("topic") or "/imu/data").strip() or "/imu/data"
    frame_id = str(ctx.get("frame_id") or sample.get("frame") or "imu_link").strip() or "imu_link"
    attachment = {
        "kind": "blacknode.robot-attachment-configuration",
        "schema_version": 1,
        "attachment_id": attachment_id,
        "capability": "imu",
        "hardware_identity": dict(hardware),
        "ros2_interfaces": [{
            "kind": "topic",
            "direction": "output",
            "topic": topic,
            "message_type": "sensor_msgs/msg/Imu",
            "frame_id": frame_id,
            "required": True,
        }],
        "stale_after_seconds": max(0.05, float(ctx.get("stale_after_seconds") or 1.0)),
    }
    capability = {
        "kind": _CAPABILITY_KIND,
        "schema_version": 1,
        "capability": "imu",
        "ready": ready,
        "imu": sample,
        "health": health,
        "hardware": hardware,
        "attachment": attachment,
    }
    return {
        "ready": ready,
        "imu_capability": capability,
        "imu": sample,
        "health": health,
        "hardware": hardware,
        "attachment_configuration": attachment,
        "report": "IMU capability ready" if ready else f"IMU unavailable: {health.get('error') or health.get('state') or 'not ready'}",
    }


@node(
    name="IMUViewer",
    category=_CATEGORY,
    description="Show a live IMU quaternion as a rotating 3D B-logo robot with body and world XYZ axes in the editor.",
    inputs={
        "action": Enum(["status", "start", "stop"], default="status"),
        "source": Dict,
        "viewer_id": Text(default="imu_viewer"),
        "stale_after_seconds": Float(default=1.0),
        "robot_length_m": Float(default=0.36),
        "robot_width_m": Float(default=0.28),
        "robot_height_m": Float(default=0.12),
        "body_frame": Text(default="base_link"),
        "sensor_mount_roll_deg": Float(default=0.0),
        "sensor_mount_pitch_deg": Float(default=0.0),
        "sensor_mount_yaw_deg": Float(default=0.0),
    },
    outputs={"running": Bool, "live": Bool, "scene": Dict, "status": Dict, "viewer": Dict, "report": Text},
    primary_inputs=["source", "action"],
    primary_outputs=["scene", "status", "report"],
    live=True,
)
def imu_viewer(ctx: dict) -> dict:
    action = str(ctx.get("action") or "status").strip().lower()
    viewer_id = str(ctx.get("viewer_id") or "imu_viewer").strip() or "imu_viewer"
    if action == "stop":
        stopped = int(imu_runtime.stop_imu_viewer(viewer_id).get("stopped") or 0)
        return {
            "running": False,
            "live": False,
            "scene": {},
            "status": {"kind": "blacknode.viewer-status", "schema_version": 1, "state": "stopped"},
            "viewer": {"viewer_id": viewer_id, "sensor": "imu", "state": "stopped"},
            "report": f"IMU Viewer stopped {stopped} session(s)",
        }
    if action == "status":
        return imu_runtime.imu_viewer_status(viewer_id)
    if action != "start":
        return {
            "running": False,
            "live": False,
            "scene": {},
            "status": {"state": "error", "error": "action must be status, start, or stop"},
            "viewer": {},
            "report": "IMU Viewer action must be status, start, or stop",
        }
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    return imu_runtime.start_imu_viewer(
        viewer_id=viewer_id,
        node_id=str(ctx.get("__node_id__") or ""),
        source=source,
        source_reader=ctx.get("__message_stream_reader__"),
        options={
            "stale_after_seconds": max(0.05, float(ctx.get("stale_after_seconds") or 1.0)),
            "robot_length_m": max(0.05, float(ctx.get("robot_length_m") or 0.36)),
            "robot_width_m": max(0.05, float(ctx.get("robot_width_m") or 0.28)),
            "robot_height_m": max(0.02, float(ctx.get("robot_height_m") or 0.12)),
            "body_frame": str(ctx.get("body_frame") or "base_link").strip() or "base_link",
            "sensor_mount_roll_deg": float(ctx.get("sensor_mount_roll_deg") or 0.0),
            "sensor_mount_pitch_deg": float(ctx.get("sensor_mount_pitch_deg") or 0.0),
            "sensor_mount_yaw_deg": float(ctx.get("sensor_mount_yaw_deg") or 0.0),
        },
    )
