"""Provider-neutral depth-camera capability, test provider, and safe features."""
from __future__ import annotations

import base64
import copy
import html
import math
import time
from typing import Any

from blacknode import contracts as bn_contracts
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node


_CATEGORY = "Perception"
_DEPTH_KIND = "blacknode.depth-camera-capability"


def _number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _positive_samples(values: Any) -> list[float]:
    if not isinstance(values, list):
        return []
    result = []
    for value in values[:10_000]:
        sample = _number(value, -1.0)
        if sample > 0.0:
            result.append(sample)
    return result


def _summary(values: list[float], total_count: int | None = None) -> dict:
    if not values:
        return {}
    ordered = sorted(values)

    def percentile(fraction: float) -> float:
        index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
        return ordered[index]

    return {
        "minimum": ordered[0],
        "p05": percentile(0.05),
        "median": percentile(0.5),
        "p95": percentile(0.95),
        "valid_count": len(ordered),
        "total_count": max(len(ordered), int(total_count or len(ordered))),
    }


def _preview(distance_m: float, label: str) -> str:
    distance = max(0.0, distance_m)
    hue = max(0, min(220, round(distance * 80)))
    safe_label = html.escape(label)
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360">
<defs><linearGradient id="d" x1="0" y1="0" x2="1" y2="1">
<stop stop-color="hsl({hue},90%,58%)"/><stop offset="1" stop-color="#08111f"/>
</linearGradient></defs>
<rect width="640" height="360" fill="url(#d)"/>
<circle cx="320" cy="180" r="78" fill="none" stroke="white" stroke-width="4" opacity=".8"/>
<text x="320" y="177" fill="white" text-anchor="middle" font-family="sans-serif" font-size="44">{distance:.2f} m</text>
<text x="320" y="216" fill="white" text-anchor="middle" font-family="sans-serif" font-size="20">{safe_label}</text>
<text x="24" y="334" fill="white" font-family="sans-serif" font-size="18">Deterministic depth test frame</text>
</svg>"""
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _health(
    value: Any,
    *,
    summary_m: dict | None = None,
    default_fresh: bool = False,
) -> dict:
    result = copy.deepcopy(value) if isinstance(value, dict) else {}
    summary_value = (
        copy.deepcopy(result.get("summary_m"))
        if isinstance(result.get("summary_m"), dict)
        else copy.deepcopy(summary_m or {})
    )
    fresh = bool(result.get("source_fresh", default_fresh))
    state = str(result.get("state") or ("ready" if fresh else "unavailable"))
    result.update({
        "state": state,
        "worker_alive": bool(result.get("worker_alive", fresh)),
        "source_fresh": fresh,
        "frames": max(0, int(result.get("frames") or (1 if fresh else 0))),
        "age_seconds": result.get("age_seconds", 0.0 if fresh else None),
        "summary_m": summary_value,
        "error": str(result.get("error") or ""),
    })
    return result


def _topic_evidence(candidate: dict, suffix: str, role: str) -> str:
    evidence = (
        candidate.get("evidence")
        if isinstance(candidate.get("evidence"), list)
        else []
    )
    for item in evidence:
        if not isinstance(item, dict):
            continue
        message_type = str(item.get("message_type") or "").lower()
        if (
            str(item.get("role") or "") == role
            and message_type.endswith(suffix.lower())
        ):
            return str(item.get("name") or "").strip()
    return ""


@node(
    name="DepthCameraDeviceSelect",
    category=_CATEGORY,
    description=(
        "Select and explicitly confirm a generic depth-camera candidate from "
        "current ROS 2 state from a paired ComputeDevice Runtime."
    ),
    inputs={
        "device": Dict,
        "ros2_graph": Dict,
        "candidate_index": Int(default=0),
        "confirm": Bool(default=False),
        "depth_topic": Text(default=""),
        "camera_info_topic": Text(default=""),
        "points_topic": Text(default=""),
        "frame_id": Text(default="camera_depth"),
        "hardware_id": Text(default=""),
    },
    outputs={
        "found": Bool,
        "confirmed": Bool,
        "action": Text,
        "candidate": Dict,
        "provider_state": Dict,
        "hardware": Dict,
        "depth_topic": Text,
        "camera_info_topic": Text,
        "points_topic": Text,
        "frame_id": Text,
        "report": Text,
    },
)
def depth_camera_device_select(ctx: dict) -> dict:
    device = (
        copy.deepcopy(ctx.get("device"))
        if isinstance(ctx.get("device"), dict)
        else {}
    )
    graph = (
        ctx.get("ros2_graph")
        if isinstance(ctx.get("ros2_graph"), dict)
        else {}
    )
    candidates = [
        copy.deepcopy(item)
        for item in (graph.get("capabilities") or [])
        if isinstance(item, dict)
        and str(item.get("capability") or "") == "depth_camera"
        and bool(item.get("safe_to_read", True))
    ]
    index = max(0, int(ctx.get("candidate_index") or 0))
    candidate = candidates[index] if index < len(candidates) else {}

    configured_depth = str(ctx.get("depth_topic") or "").strip()
    configured_info = str(ctx.get("camera_info_topic") or "").strip()
    configured_points = str(ctx.get("points_topic") or "").strip()
    depth_topic = configured_depth or _topic_evidence(
        candidate,
        "/image",
        "state",
    )
    camera_info_topic = configured_info or _topic_evidence(
        candidate,
        "/camerainfo",
        "metadata",
    )
    points_topic = configured_points
    frame_id = str(ctx.get("frame_id") or "camera_depth").strip()
    hardware_id = str(ctx.get("hardware_id") or "").strip()
    found = bool(depth_topic)
    confirmed = bool(found and ctx.get("confirm"))
    action = "start" if confirmed else "stop"
    hardware = (
        {
            "id": hardware_id,
            "serial": hardware_id,
            "kind": "depth_camera",
        }
        if hardware_id
        else {}
    )
    device_id = str(device.get("device_id") or "").strip()
    provider_state = {
        "kind": "blacknode.provider-state",
        "schema_version": 1,
        "provider_id": (
            f"{device_id}:depth_camera"
            if device_id
            else "depth_camera"
        ),
        "provider": {
            "package": "blacknode-perception",
            "component": "depth",
            "adapter": "ros2",
        },
        "state": (
            "configured"
            if confirmed
            else "awaiting_confirmation"
            if found
            else "unavailable"
        ),
        "available": found,
        "ready": confirmed,
        "device_id": device_id,
        "reason": (
            "candidate confirmed; ROS 2 adapter must still verify live freshness"
            if confirmed
            else "review and confirm the discovered depth topic"
            if found
            else "no readable depth image candidate was discovered"
        ),
    }
    device_label = str(
        device.get("device_name") or device_id or "selected device"
    )
    if not found:
        report = (
            f"{device_label}: no raw depth image candidate found. "
            "Refresh the device inspection while ROS 2 bringup is running, "
            "or enter an advanced depth-topic override."
        )
    elif not confirmed:
        report = (
            f"{device_label}: discovered {depth_topic}. Review the topic, "
            "frame, depth scale, and physical camera identity, then enable "
            "confirm to start the read-only stream."
        )
    else:
        report = (
            f"{device_label}: confirmed depth stream {depth_topic}"
            + (
                f" with hardware identity {hardware_id}."
                if hardware_id
                else ". Add the physical camera serial before saving calibration."
            )
        )
    return {
        "found": found,
        "confirmed": confirmed,
        "action": action,
        "candidate": candidate,
        "provider_state": provider_state,
        "hardware": hardware,
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
        "points_topic": points_topic,
        "frame_id": frame_id,
        "report": report,
    }


@node(
    name="DepthCameraTestProvider",
    category=_CATEGORY,
    description=(
        "Provide deterministic mock or replay depth data that implements the "
        "same contract as a physical depth-camera adapter."
    ),
    inputs={
        "mode": Enum(["mock", "replay"], default="mock"),
        "provider_id": Text(default="depth_test"),
        "hardware_id": Text(default="depth-camera-test-001"),
        "frame_id": Text(default="camera_depth"),
        "distance_m": Float(default=1.2),
        "samples_m": List(default=[]),
        "replay": Dict,
    },
    outputs={
        "preview": Image,
        "provider_state": Dict,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
        "health": Dict,
        "hardware": Dict,
        "report": Text,
    },
)
def depth_camera_test_provider(ctx: dict) -> dict:
    mode = str(ctx.get("mode") or "mock").strip().lower()
    provider_id = str(ctx.get("provider_id") or "depth_test").strip()
    hardware_id = str(
        ctx.get("hardware_id") or "depth-camera-test-001"
    ).strip()
    frame_id = str(ctx.get("frame_id") or "camera_depth").strip()
    replay = ctx.get("replay") if isinstance(ctx.get("replay"), dict) else {}

    if mode == "replay":
        if replay:
            depth = copy.deepcopy(
                replay.get("depth_stream")
                if isinstance(replay.get("depth_stream"), dict)
                else replay
            )
            points = copy.deepcopy(
                replay.get("point_cloud_stream")
                if isinstance(replay.get("point_cloud_stream"), dict)
                else {}
            )
            summary_m = (
                depth.get("summary_m")
                if isinstance(depth.get("summary_m"), dict)
                else {}
            )
            health = _health(
                replay.get("health") or depth.get("health"),
                summary_m=summary_m,
                default_fresh=bool(summary_m),
            )
            available = (
                depth.get("kind") == "blacknode.depth-stream"
                and bool(health.get("source_fresh"))
            )
            preview = str(
                replay.get("preview")
                or depth.get("snapshot_url")
                or ""
            )
            report = (
                f"replay depth provider {provider_id}: "
                + ("ready" if available else "unavailable or stale")
            )
        else:
            depth = {}
            points = {}
            health = _health({
                "state": "unavailable",
                "error": "select a recorded depth artifact",
            })
            preview = ""
            available = False
            report = (
                f"replay depth provider {provider_id}: "
                "unavailable; select a recorded depth artifact"
            )
    else:
        samples = _positive_samples(ctx.get("samples_m"))
        distance_m = max(0.001, _number(ctx.get("distance_m"), 1.2))
        if not samples:
            samples = [
                distance_m * 1.02,
                distance_m,
                distance_m * 0.99,
                distance_m * 1.01,
                distance_m,
            ]
        summary_m = _summary(samples)
        depth = bn_contracts.depth_stream(
            frame_id,
            encoding="32FC1",
            depth_scale=1.0,
        )
        depth.update({
            "provider_id": provider_id,
            "summary_m": summary_m,
            "samples_m": samples,
            "topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/depth/camera_info",
        })
        health = _health({}, summary_m=summary_m, default_fresh=True)
        health["source_time_ns"] = time.time_ns()
        depth["health"] = copy.deepcopy(health)
        points = {}
        preview = _preview(float(summary_m["p05"]), provider_id)
        available = True
        report = (
            f"mock depth provider {provider_id}: "
            f"{summary_m['valid_count']} valid samples, "
            f"nearest robust distance {summary_m['p05']:.3f} m"
        )

    hardware = copy.deepcopy(
        replay.get("hardware")
        if mode == "replay" and isinstance(replay.get("hardware"), dict)
        else {}
    )
    hardware.setdefault("id", hardware_id)
    hardware.setdefault("serial", hardware_id)
    hardware.setdefault("kind", "depth_camera")
    hardware.setdefault("simulated", mode == "mock")
    provider_state = {
        "kind": "blacknode.provider-state",
        "schema_version": 1,
        "provider_id": provider_id,
        "provider": {
            "package": "blacknode-perception",
            "component": "depth",
            "adapter": "test",
        },
        "state": "ready" if available else str(health.get("state") or "unavailable"),
        "available": available,
        "ready": available,
        "health": copy.deepcopy(health),
    }
    return {
        "preview": preview,
        "provider_state": provider_state,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "health": health,
        "hardware": hardware,
        "report": report,
    }


@node(
    name="DepthCamera",
    category=_CATEGORY,
    description=(
        "Normalize any compatible provider into a stable DepthCamera capability "
        "and deployment attachment configuration."
    ),
    inputs={
        "preview": Image,
        "provider_state": Dict,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_id": Text(default="front_depth_camera"),
        "depth_topic": Text(default="/camera/depth/image_raw"),
        "camera_info_topic": Text(default="/camera/depth/camera_info"),
        "points_topic": Text(default=""),
        "frame_id": Text(default="camera_depth"),
        "stale_after_seconds": Float(default=2.0),
    },
    outputs={
        "preview": Image,
        "ready": Bool,
        "depth_camera": Dict,
        "depth_stream": Dict,
        "point_cloud_stream": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_configuration": Dict,
        "report": Text,
    },
)
def depth_camera(ctx: dict) -> dict:
    depth = copy.deepcopy(
        ctx.get("depth_stream")
        if isinstance(ctx.get("depth_stream"), dict)
        else {}
    )
    points = copy.deepcopy(
        ctx.get("point_cloud_stream")
        if isinstance(ctx.get("point_cloud_stream"), dict)
        else {}
    )
    provider_state = copy.deepcopy(
        ctx.get("provider_state")
        if isinstance(ctx.get("provider_state"), dict)
        else {}
    )
    health_value = ctx.get("health") or depth.get("health")
    summary_m = (
        depth.get("summary_m")
        if isinstance(depth.get("summary_m"), dict)
        else {}
    )
    health = _health(health_value, summary_m=summary_m)
    age_seconds = health.get("age_seconds")
    stale_after = max(0.1, _number(ctx.get("stale_after_seconds"), 2.0))
    if isinstance(age_seconds, (int, float)) and age_seconds > stale_after:
        health["source_fresh"] = False
        health["state"] = "stale"

    provider_ready = bool(
        provider_state.get("ready", provider_state.get("available", True))
    )
    ready = bool(
        depth.get("kind") == "blacknode.depth-stream"
        and health.get("source_fresh")
        and provider_ready
    )
    if not ready and health.get("state") == "ready":
        health["state"] = "unavailable"

    frame_id = str(
        depth.get("frame")
        or ctx.get("frame_id")
        or "camera_depth"
    ).strip()
    provider_was_supplied = bool(provider_state)
    depth_topic = str(
        depth.get("topic")
        or ctx.get("depth_topic")
        or ("" if provider_was_supplied else "/camera/depth/image_raw")
    ).strip()
    camera_info_topic = str(
        depth.get("camera_info_topic")
        or ctx.get("camera_info_topic")
        or ("" if provider_was_supplied else "/camera/depth/camera_info")
    ).strip()
    points_topic = str(
        points.get("topic")
        or ctx.get("points_topic")
        or ""
    ).strip()
    interfaces = []
    if depth_topic:
        interfaces.append({
            "name": "depth_image",
            "kind": "topic",
            "direction": "output",
            "topic": depth_topic,
            "candidates": [depth_topic],
            "message_type": "sensor_msgs/msg/Image",
            "frame_id": frame_id,
            "required": True,
        })
    if camera_info_topic:
        interfaces.append({
            "name": "depth_camera_info",
            "kind": "topic",
            "direction": "output",
            "topic": camera_info_topic,
            "candidates": [camera_info_topic],
            "message_type": "sensor_msgs/msg/CameraInfo",
            "frame_id": frame_id,
            "required": False,
        })
    if points_topic:
        interfaces.append({
            "name": "point_cloud",
            "kind": "topic",
            "direction": "output",
            "topic": points_topic,
            "candidates": [points_topic],
            "message_type": "sensor_msgs/msg/PointCloud2",
            "frame_id": frame_id,
            "required": False,
        })
    attachment_id = str(
        ctx.get("attachment_id") or "front_depth_camera"
    ).strip()
    attachment_configuration = {
        "attachment_id": attachment_id,
        "attachment_type": "depth_camera",
        "frame_id": frame_id,
        "ros2_interfaces": interfaces,
        "provider_contract": _DEPTH_KIND,
    }
    hardware = copy.deepcopy(
        ctx.get("hardware")
        if isinstance(ctx.get("hardware"), dict)
        else {}
    )
    capability = {
        "kind": _DEPTH_KIND,
        "schema_version": 1,
        "capability": "depth_camera",
        "ready": ready,
        "provider_state": provider_state,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "health": health,
        "hardware_identity": hardware,
        "attachment_configuration": attachment_configuration,
    }
    reason = (
        f"ready · {health.get('summary_m', {}).get('valid_count', 0)} valid depth samples"
        if ready
        else f"{health.get('state') or 'unavailable'} · fresh metric depth required"
    )
    return {
        "preview": str(ctx.get("preview") or depth.get("snapshot_url") or ""),
        "ready": ready,
        "depth_camera": capability,
        "depth_stream": depth,
        "point_cloud_stream": points,
        "health": health,
        "hardware": hardware,
        "attachment_configuration": attachment_configuration,
        "report": f"DepthCamera {reason}",
    }


@node(
    name="DepthObstacleWarning",
    category=_CATEGORY,
    description=(
        "Assess fresh metric depth for a nearby obstacle. Unknown or stale "
        "data fails closed and never sends motion commands."
    ),
    inputs={
        "depth_camera": Dict,
        "warning_distance_m": Float(default=0.8),
        "critical_distance_m": Float(default=0.4),
        "min_valid_points": Int(default=1),
    },
    outputs={
        "measured": Bool,
        "safe_to_proceed": Bool,
        "warning": Bool,
        "critical": Bool,
        "nearest_m": Float,
        "valid_points": Int,
        "assessment": Dict,
        "report": Text,
    },
)
def depth_obstacle_warning(ctx: dict) -> dict:
    camera = (
        ctx.get("depth_camera")
        if isinstance(ctx.get("depth_camera"), dict)
        else {}
    )
    health = camera.get("health") if isinstance(camera.get("health"), dict) else {}
    depth = (
        camera.get("depth_stream")
        if isinstance(camera.get("depth_stream"), dict)
        else {}
    )
    summary_m = (
        health.get("summary_m")
        if isinstance(health.get("summary_m"), dict)
        else depth.get("summary_m")
        if isinstance(depth.get("summary_m"), dict)
        else {}
    )
    nearest_m = _number(summary_m.get("p05"), -1.0)
    valid_points = max(0, int(summary_m.get("valid_count") or 0))
    minimum = max(1, int(ctx.get("min_valid_points") or 1))
    measured = bool(
        camera.get("ready")
        and health.get("source_fresh")
        and nearest_m > 0.0
        and valid_points >= minimum
    )
    warning_distance = max(0.0, _number(ctx.get("warning_distance_m"), 0.8))
    critical_distance = max(
        0.0,
        min(warning_distance, _number(ctx.get("critical_distance_m"), 0.4)),
    )
    critical = bool(measured and nearest_m <= critical_distance)
    warning = bool(measured and nearest_m <= warning_distance)
    safe_to_proceed = bool(measured and not warning)
    state = (
        "unknown"
        if not measured
        else "critical"
        if critical
        else "warning"
        if warning
        else "clear"
    )
    assessment = {
        "kind": "blacknode.depth-obstacle-assessment",
        "schema_version": 1,
        "state": state,
        "measured": measured,
        "safe_to_proceed": safe_to_proceed,
        "warning": warning,
        "critical": critical,
        "nearest_m": nearest_m if measured else None,
        "valid_points": valid_points,
        "warning_distance_m": warning_distance,
        "critical_distance_m": critical_distance,
    }
    report = (
        "depth obstacle state unknown; stale or insufficient metric data blocks proceed"
        if not measured
        else f"depth obstacle {state}: robust nearest distance {nearest_m:.3f} m"
    )
    return {
        "measured": measured,
        "safe_to_proceed": safe_to_proceed,
        "warning": warning,
        "critical": critical,
        "nearest_m": nearest_m if measured else 0.0,
        "valid_points": valid_points,
        "assessment": assessment,
        "report": report,
    }
