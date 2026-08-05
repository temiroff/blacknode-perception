"""Provider-neutral IMU contracts, deterministic test data, and live viewer."""
from __future__ import annotations

import copy
import math
import time
from typing import Any

from blacknode import contracts as bn_contracts
from blacknode.node import Bool, Dict, Enum, Float, Text, node

from . import imu_runtime


runtime_status = imu_runtime.runtime_status
stop_runtime_services = imu_runtime.stop_runtime_services

_CATEGORY = "Perception"
_CAPABILITY_KIND = "blacknode.imu-capability"


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

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
    name="IMUTestProvider",
    category=_CATEGORY,
    description="Provide deterministic mock orientation or replay a normalized IMU sample for hardware-free viewer testing.",
    inputs={
        "mode": Enum(["mock", "replay"], default="mock"),
        "provider_id": Text(default="imu_test"),
        "hardware_id": Text(default="imu-test-001"),
        "frame_id": Text(default="imu_link"),
        "roll_deg": Float(default=0.0),
        "pitch_deg": Float(default=0.0),
        "yaw_deg": Float(default=0.0),
        "replay": Dict,
    },
    outputs={"provider_state": Dict, "imu": Dict, "health": Dict, "hardware": Dict, "report": Text},
    primary_inputs=["mode", "roll_deg", "pitch_deg", "yaw_deg", "replay"],
    primary_outputs=["imu", "health", "report"],
)
def imu_test_provider(ctx: dict) -> dict:
    mode = str(ctx.get("mode") or "mock").strip().lower()
    provider_id = str(ctx.get("provider_id") or "imu_test").strip() or "imu_test"
    hardware_id = str(ctx.get("hardware_id") or "imu-test-001").strip() or "imu-test-001"
    if mode == "replay":
        replay = copy.deepcopy(ctx.get("replay")) if isinstance(ctx.get("replay"), dict) else {}
        imu = replay.get("imu") if isinstance(replay.get("imu"), dict) else replay
        if imu.get("kind") != "blacknode.imu-stream":
            imu = {}
        fresh = bool(imu)
        health = copy.deepcopy(replay.get("health")) if isinstance(replay.get("health"), dict) else _health(
            imu, fresh=fresh, error="" if fresh else "Replay has no blacknode.imu-stream sample",
        )
        hardware = copy.deepcopy(replay.get("hardware")) if isinstance(replay.get("hardware"), dict) else {}
        hardware.setdefault("id", hardware_id)
    else:
        roll = math.radians(float(ctx.get("roll_deg") or 0.0))
        pitch = math.radians(float(ctx.get("pitch_deg") or 0.0))
        yaw = math.radians(float(ctx.get("yaw_deg") or 0.0))
        imu = bn_contracts.imu_stream(
            str(ctx.get("frame_id") or "imu_link").strip() or "imu_link",
            sequence=1,
            orientation=_quaternion_from_euler(roll, pitch, yaw),
            linear_acceleration=(0.0, 0.0, 9.80665),
        )
        health = _health(imu, fresh=True)
        hardware = {"id": hardware_id, "provider": provider_id, "simulated": True}
    hardware.setdefault("provider", provider_id)
    hardware.setdefault("simulated", mode == "mock")
    provider_state = {
        "kind": "blacknode.imu-provider-state",
        "schema_version": 1,
        "provider_id": provider_id,
        "mode": mode,
        "available": bool(imu),
        "sample_time_ns": int(imu.get("receive_time_ns") or time.time_ns()) if imu else 0,
    }
    return {
        "provider_state": provider_state,
        "imu": imu,
        "health": health,
        "hardware": hardware,
        "report": "IMU test sample ready" if imu else str(health.get("error") or "IMU replay unavailable"),
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
        },
    )
