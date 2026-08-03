"""Provider-neutral 2D LiDAR contracts and deterministic test scans."""
from __future__ import annotations

import copy
import math
import time
from typing import Any

from blacknode import contracts as bn_contracts
from blacknode.node import Bool, Dict, Enum, Float, Int, List, Text, node


_CATEGORY = "Perception"
_LIDAR_KIND = "blacknode.lidar-capability"


def _number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _mock_room_scan(sample_count: int, width_m: float, height_m: float) -> list[float]:
    """Return ray distances to an axis-aligned room around the sensor origin."""
    half_width = max(0.25, width_m * 0.5)
    half_height = max(0.25, height_m * 0.5)
    result: list[float] = []
    for index in range(sample_count):
        angle = -math.pi + (2.0 * math.pi * index / sample_count)
        dx = math.cos(angle)
        dy = math.sin(angle)
        candidates: list[float] = []
        if abs(dx) > 1e-8:
            candidates.append(half_width / abs(dx))
        if abs(dy) > 1e-8:
            candidates.append(half_height / abs(dy))
        result.append(round(min(candidates), 6))
    return result


def _scan_health(scan: dict, *, source_fresh: bool, error: str = "") -> dict:
    ranges = scan.get("ranges") if isinstance(scan.get("ranges"), list) else []
    valid_count = 0
    minimum_m: float | None = None
    maximum_m: float | None = None
    for value in ranges:
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
            continue
        clean = float(value)
        valid_count += 1
        minimum_m = clean if minimum_m is None else min(minimum_m, clean)
        maximum_m = clean if maximum_m is None else max(maximum_m, clean)
    return {
        "state": "ready" if source_fresh else "unavailable",
        "worker_alive": source_fresh,
        "source_fresh": source_fresh,
        "scans": 1 if source_fresh else 0,
        "age_seconds": 0.0 if source_fresh else None,
        "sample_count": len(ranges),
        "valid_count": valid_count,
        "minimum_m": minimum_m,
        "maximum_m": maximum_m,
        "error": error,
    }


@node(
    name="LiDARTestProvider",
    category=_CATEGORY,
    description=(
        "Provide a deterministic mock room scan or a recorded LaserScan using "
        "the same normalized contract as a physical LiDAR provider."
    ),
    inputs={
        "mode": Enum(["mock", "replay"], default="mock"),
        "provider_id": Text(default="lidar_test"),
        "hardware_id": Text(default="lidar-test-001"),
        "frame_id": Text(default="laser"),
        "sample_count": Int(default=360),
        "room_width_m": Float(default=6.0),
        "room_height_m": Float(default=4.0),
        "replay": Dict,
    },
    outputs={
        "provider_state": Dict,
        "laser_scan": Dict,
        "health": Dict,
        "hardware": Dict,
        "report": Text,
    },
)
def lidar_test_provider(ctx: dict) -> dict:
    mode = str(ctx.get("mode") or "mock").strip().lower()
    provider_id = str(ctx.get("provider_id") or "lidar_test").strip()
    hardware_id = str(ctx.get("hardware_id") or "lidar-test-001").strip()
    frame_id = str(ctx.get("frame_id") or "laser").strip()
    replay = ctx.get("replay") if isinstance(ctx.get("replay"), dict) else {}

    if mode == "replay":
        scan = copy.deepcopy(
            replay.get("laser_scan")
            if isinstance(replay.get("laser_scan"), dict)
            else replay
        )
        ready = scan.get("kind") == "blacknode.laser-scan-stream"
        health = copy.deepcopy(replay.get("health") or {})
        if not isinstance(health, dict) or not health:
            health = _scan_health(
                scan,
                source_fresh=ready,
                error="" if ready else "select a recorded LaserScan artifact",
            )
        report = (
            f"replay LiDAR provider {provider_id}: "
            + ("ready" if ready else "unavailable; select a recorded LaserScan artifact")
        )
    else:
        sample_count = max(8, min(5_000_000, int(ctx.get("sample_count") or 360)))
        width_m = max(0.5, _number(ctx.get("room_width_m"), 6.0))
        height_m = max(0.5, _number(ctx.get("room_height_m"), 4.0))
        ranges = _mock_room_scan(sample_count, width_m, height_m)
        scan = bn_contracts.laser_scan_stream(
            frame_id,
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=2.0 * math.pi / sample_count,
            range_min=0.05,
            range_max=max(width_m, height_m) * 2.0,
            ranges=ranges,
        )
        scan.update({
            "provider_id": provider_id,
            "topic": "/scan",
            "message_type": "sensor_msgs/msg/LaserScan",
        })
        health = _scan_health(scan, source_fresh=True)
        ready = True
        report = (
            f"mock LiDAR provider {provider_id}: {sample_count} rays in a "
            f"{width_m:g} m x {height_m:g} m room"
        )

    hardware = copy.deepcopy(
        replay.get("hardware")
        if mode == "replay" and isinstance(replay.get("hardware"), dict)
        else {}
    )
    hardware.setdefault("id", hardware_id)
    hardware.setdefault("serial", hardware_id)
    hardware.setdefault("kind", "lidar")
    hardware.setdefault("simulated", mode == "mock")
    provider_state = {
        "kind": "blacknode.provider-state",
        "schema_version": 1,
        "provider_id": provider_id,
        "provider": {
            "package": "blacknode-perception",
            "component": "lidar",
            "adapter": "test",
        },
        "state": "ready" if ready else "unavailable",
        "available": ready,
        "ready": ready,
        "health": copy.deepcopy(health),
    }
    return {
        "provider_state": provider_state,
        "laser_scan": scan,
        "health": health,
        "hardware": hardware,
        "report": report,
    }


@node(
    name="LiDAR",
    category=_CATEGORY,
    description=(
        "Normalize a compatible provider into a stable read-only LiDAR "
        "capability and portable attachment configuration."
    ),
    inputs={
        "provider_state": Dict,
        "laser_scan": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_id": Text(default="front_lidar"),
        "topic": Text(default="/scan"),
        "frame_id": Text(default="laser"),
        "stale_after_seconds": Float(default=1.0),
    },
    outputs={
        "ready": Bool,
        "lidar": Dict,
        "laser_scan": Dict,
        "health": Dict,
        "hardware": Dict,
        "attachment_configuration": Dict,
        "report": Text,
    },
)
def lidar(ctx: dict) -> dict:
    scan = copy.deepcopy(ctx.get("laser_scan") if isinstance(ctx.get("laser_scan"), dict) else {})
    provider_state = copy.deepcopy(
        ctx.get("provider_state") if isinstance(ctx.get("provider_state"), dict) else {}
    )
    health = copy.deepcopy(ctx.get("health") if isinstance(ctx.get("health"), dict) else {})
    if not health:
        health = _scan_health(scan, source_fresh=False, error="provider health is unavailable")
    stale_after = max(0.05, _number(ctx.get("stale_after_seconds"), 1.0))
    age = health.get("age_seconds")
    if isinstance(age, (int, float)) and age > stale_after:
        health["source_fresh"] = False
        health["state"] = "stale"
    ready = bool(
        scan.get("kind") == "blacknode.laser-scan-stream"
        and health.get("source_fresh")
        and provider_state.get("ready", provider_state.get("available", True))
    )
    frame_id = str(scan.get("frame") or ctx.get("frame_id") or "laser").strip()
    topic = str(scan.get("topic") or ctx.get("topic") or "").strip()
    hardware = copy.deepcopy(ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else {})
    attachment_id = str(ctx.get("attachment_id") or "front_lidar").strip()
    interfaces = []
    if topic:
        interfaces.append({
            "name": "laser_scan",
            "kind": "topic",
            "direction": "output",
            "topic": topic,
            "candidates": [topic],
            "message_type": "sensor_msgs/msg/LaserScan",
            "frame_id": frame_id,
            "required": True,
        })
    capability = {
        "kind": _LIDAR_KIND,
        "schema_version": 1,
        "provider_state": provider_state,
        "scan": scan,
        "health": health,
        "hardware_identity": hardware,
        "frame_id": frame_id,
        "ready": ready,
    }
    attachment = {
        "attachment_id": attachment_id,
        "attachment_type": "lidar",
        "capability": "lidar",
        "provider_contract": _LIDAR_KIND,
        "provider": {
            "package": "blacknode-perception",
            "component": "lidar",
        },
        "frame_id": frame_id,
        "hardware_identity": hardware,
        "ros2_interfaces": interfaces,
    }
    report = (
        f"LiDAR ready: {health.get('valid_count', 0)} valid points in {frame_id}"
        if ready
        else f"LiDAR unavailable: {health.get('error') or health.get('state') or 'no fresh scan'}"
    )
    return {
        "ready": ready,
        "lidar": capability,
        "laser_scan": scan,
        "health": health,
        "hardware": hardware,
        "attachment_configuration": attachment,
        "report": report,
    }
