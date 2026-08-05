"""Provider-neutral 2D LiDAR contracts."""
from __future__ import annotations

import copy
import math
import time
from typing import Any

from blacknode import contracts as bn_contracts
from blacknode.node import Bool, Dict, Float, List, Text, node


_CATEGORY = "Perception"
_LIDAR_KIND = "blacknode.lidar-capability"


def _number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


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


def _stream_message(outputs: dict) -> dict:
    envelope = outputs.get("message") if isinstance(outputs.get("message"), dict) else {}
    return envelope.get("message") if isinstance(envelope.get("message"), dict) else envelope


@node(
    name="LaserScanProcessor",
    category=_CATEGORY,
    description=(
        "Process a generic ROS2 LaserScan stream into a normalized scan while "
        "preserving its managed stream for live viewers and optional Warp stages."
    ),
    inputs={"source": Dict},
    outputs={"stream": Dict, "laser_scan": Dict, "health": Dict, "report": Text},
    primary_inputs=["source"],
    primary_outputs=["stream", "laser_scan", "health"],
)
def laser_scan_processor(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    valid = (
        source.get("kind") == "blacknode.message-stream"
        and source.get("protocol") == "ros2"
        and source.get("message_type") == "sensor_msgs/msg/LaserScan"
    )
    stream = copy.deepcopy(source) if valid else {}
    if stream:
        stream["processor"] = "LaserScanProcessor"
    reader = ctx.get("__message_stream_reader__")
    outputs = reader(source) if valid and callable(reader) else {}
    message = _stream_message(outputs) if isinstance(outputs, dict) else {}
    ranges = message.get("ranges") if isinstance(message.get("ranges"), list) else []
    scan = {}
    if ranges:
        header = message.get("header") if isinstance(message.get("header"), dict) else {}
        scan = bn_contracts.laser_scan_stream(
            str(header.get("frame_id") or "laser"),
            angle_min=_number(message.get("angle_min"), -math.pi),
            angle_max=_number(message.get("angle_max"), math.pi),
            angle_increment=_number(message.get("angle_increment"), 0.0),
            range_min=max(0.0, _number(message.get("range_min"), 0.0)),
            range_max=max(0.0, _number(message.get("range_max"), 0.0)),
            ranges=list(ranges)[:100_000],
        )
        stamp = header.get("stamp") if isinstance(header.get("stamp"), dict) else {}
        scan.update(
            topic=str(source.get("topic") or ""),
            message_type="sensor_msgs/msg/LaserScan",
            source_time_ns=int(stamp.get("sec") or 0) * 1_000_000_000 + int(stamp.get("nanosec") or 0),
            receive_time_ns=time.time_ns(),
            scan_time=max(0.0, _number(message.get("scan_time"), 0.0)),
            time_increment=max(0.0, _number(message.get("time_increment"), 0.0)),
            intensities=list(message.get("intensities") or [])[:100_000],
        )
    status = outputs.get("status") if isinstance(outputs, dict) and isinstance(outputs.get("status"), dict) else {}
    health = _scan_health(
        scan,
        source_fresh=bool(status.get("source_fresh") and scan),
        error="" if valid else "connect ROS2.stream carrying sensor_msgs/msg/LaserScan",
    )
    return {
        "stream": stream,
        "laser_scan": scan,
        "health": health,
        "report": (
            f"LaserScan processor ready for {source.get('topic') or '(topic not set)'}"
            if valid
            else "LaserScan processor needs a generic ROS2 LaserScan stream"
        ),
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
