"""Managed, provider-neutral IMU orientation viewer sessions."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable


_SESSIONS: dict[str, dict[str, Any]] = {}
_LOCK = threading.RLock()


def _safe_id(value: str) -> str:
    return "".join(
        character for character in str(value or "")
        if character.isalnum() or character in "_-"
    )[:80]


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _local_stream_reader(source: dict[str, Any]) -> dict[str, Any]:
    if str(source.get("protocol") or "") != "ros2":
        return {"status": {"state": "unavailable", "error": "unsupported IMU stream protocol"}}
    try:
        from blacknode.pkg.blacknode_ros2 import ros2_runtime
    except Exception as exc:  # pragma: no cover - optional package path
        return {
            "status": {
                "state": "unavailable",
                "error": f"blacknode-ros2 is unavailable ({type(exc).__name__}: {exc})",
            }
        }
    topic = str(source.get("topic") or "").strip()
    return ros2_runtime.ros2_topic_outputs(ros2_runtime.topic_subscriber_status(topic))


def _message_payload(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {}
    nested = value.get("message")
    return nested if isinstance(nested, dict) else value


def _normalized_quaternion(value: Any) -> tuple[dict[str, float], str]:
    orientation = value if isinstance(value, dict) else {}
    quaternion = {
        "x": _finite(orientation.get("x")),
        "y": _finite(orientation.get("y")),
        "z": _finite(orientation.get("z")),
        "w": _finite(orientation.get("w"), 1.0),
    }
    norm = math.sqrt(sum(component * component for component in quaternion.values()))
    if norm < 1.0e-8:
        return {}, "IMU orientation quaternion has zero length"
    return {key: component / norm for key, component in quaternion.items()}, ""


def _euler(quaternion: dict[str, float]) -> dict[str, float]:
    x, y, z, w = (quaternion[key] for key in ("x", "y", "z", "w"))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return {"roll": roll, "pitch": pitch, "yaw": yaw}


def _quaternion_from_rpy_degrees(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> dict[str, float]:
    """Return the ROS fixed-axis RPY rotation from sensor frame to body frame."""
    roll = math.radians(roll_deg) * 0.5
    pitch = math.radians(pitch_deg) * 0.5
    yaw = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def _vector(value: Any) -> dict[str, float]:
    vector = value if isinstance(value, dict) else {}
    return {axis: _finite(vector.get(axis)) for axis in ("x", "y", "z")}


def _stamp_ns(message: dict[str, Any]) -> int:
    header = message.get("header") if isinstance(message.get("header"), dict) else {}
    stamp = header.get("stamp") if isinstance(header.get("stamp"), dict) else {}
    return int(_finite(stamp.get("sec"))) * 1_000_000_000 + int(_finite(stamp.get("nanosec")))


def _normalize_source(
    source: dict[str, Any],
    reader: Callable[[dict[str, Any]], dict[str, Any]] | None,
    stale_after_seconds: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    if source.get("kind") == "blacknode.imu-capability":
        source = source.get("imu") if isinstance(source.get("imu"), dict) else {}

    outputs: dict[str, Any] = {}
    if source.get("kind") == "blacknode.message-stream":
        selected_reader = reader if callable(reader) else _local_stream_reader
        outputs = selected_reader(source)
        message = _message_payload(outputs.get("message"))
        source_status = outputs.get("status") if isinstance(outputs.get("status"), dict) else {}
        received = int(outputs.get("received") or source_status.get("received") or 0)
        age_seconds = source_status.get("age_seconds")
        source_fresh = bool(source_status.get("source_fresh"))
        receive_time_ns = int(source_status.get("last_message_time_ns") or 0)
    elif source.get("kind") == "blacknode.imu-stream":
        message = source
        received = max(1, int(source.get("sequence") or 1))
        receive_time_ns = int(source.get("receive_time_ns") or 0)
        age_seconds = (
            max(0.0, (time.time_ns() - receive_time_ns) / 1_000_000_000.0)
            if receive_time_ns else None
        )
        source_fresh = age_seconds is None or age_seconds <= stale_after_seconds
        source_status = {}
    else:
        return {}, {
            "state": "waiting",
            "source_fresh": False,
            "received": 0,
            "age_seconds": None,
            "error": "Connect an IMU capability or ROS2 message stream",
        }

    if not message:
        return {}, {
            "state": str(source_status.get("state") or "waiting"),
            "source_fresh": False,
            "received": received,
            "age_seconds": age_seconds,
            "error": str(source_status.get("error") or "Waiting for the first IMU sample"),
        }

    covariance = message.get("orientation_covariance")
    if isinstance(covariance, list) and covariance and _finite(covariance[0]) < 0.0:
        return {}, {
            "state": "unavailable",
            "source_fresh": False,
            "received": received,
            "age_seconds": age_seconds,
            "error": "The IMU message reports that orientation is unavailable",
        }

    quaternion, error = _normalized_quaternion(message.get("orientation"))
    if error:
        return {}, {
            "state": "error",
            "source_fresh": False,
            "received": received,
            "age_seconds": age_seconds,
            "error": error,
        }

    header = message.get("header") if isinstance(message.get("header"), dict) else {}
    frame = str(message.get("frame") or header.get("frame_id") or source.get("frame") or "imu_link").strip()
    sample = {
        "kind": "blacknode.imu-stream",
        "schema_version": 1,
        "frame": frame or "imu_link",
        "sequence": received,
        "source_time_ns": int(message.get("source_time_ns") or _stamp_ns(message) or 0),
        "receive_time_ns": int(message.get("receive_time_ns") or receive_time_ns or time.time_ns()),
        "orientation": quaternion,
        "euler_rad": _euler(quaternion),
        "angular_velocity": _vector(message.get("angular_velocity")),
        "linear_acceleration": _vector(message.get("linear_acceleration")),
        "topic": str(source.get("topic") or ""),
        "message_type": str(source.get("message_type") or "sensor_msgs/msg/Imu"),
    }
    state = "ready" if source_fresh else "stale"
    return sample, {
        "state": state,
        "source_fresh": source_fresh,
        "received": received,
        "age_seconds": age_seconds,
        "error": str(source_status.get("error") or ""),
    }


def _outputs(session: dict[str, Any]) -> dict[str, Any]:
    status = dict(session.get("status") or {})
    return {
        "running": bool(session.get("running")),
        "live": bool(session.get("live")),
        "scene": dict(session.get("scene") or {}),
        "status": status,
        "viewer": {
            "kind": "blacknode.viewer",
            "schema_version": 1,
            "viewer_id": session["viewer_id"],
            "sensor": "imu",
            "state": status.get("state", "waiting"),
        },
        "report": str(session.get("report") or "IMU Viewer is waiting for source data"),
    }


def _update(session: dict[str, Any]) -> None:
    sample, source_status = _normalize_source(
        session["source"],
        session.get("source_reader"),
        float(session["options"]["stale_after_seconds"]),
    )
    state = str(source_status.get("state") or "waiting")
    session["live"] = state == "ready"
    session["status"] = {
        "kind": "blacknode.viewer-status",
        "schema_version": 1,
        "state": state,
        "source_fresh": bool(source_status.get("source_fresh")),
        "received": int(source_status.get("received") or 0),
        "age_seconds": source_status.get("age_seconds"),
        "error": str(source_status.get("error") or ""),
    }
    if sample:
        options = session["options"]
        session["scene"] = {
            "kind": "blacknode.viewer-scene",
            "schema_version": 1,
            "primitive": "imu-orientation",
            "projection": "xyz",
            "frame": sample["frame"],
            "sequence": sample["sequence"],
            "source_time_ns": sample["source_time_ns"],
            "receive_time_ns": sample["receive_time_ns"],
            "robot": {
                "length_m": options["robot_length_m"],
                "width_m": options["robot_width_m"],
                "height_m": options["robot_height_m"],
            },
            "mounting": {
                "body_frame": options["body_frame"],
                "sensor_frame": sample["frame"],
                "body_from_sensor_rpy_deg": {
                    "roll": options["sensor_mount_roll_deg"],
                    "pitch": options["sensor_mount_pitch_deg"],
                    "yaw": options["sensor_mount_yaw_deg"],
                },
                "body_from_sensor_quaternion": _quaternion_from_rpy_degrees(
                    options["sensor_mount_roll_deg"],
                    options["sensor_mount_pitch_deg"],
                    options["sensor_mount_yaw_deg"],
                ),
            },
            "imu": {
                "orientation": sample["orientation"],
                "euler_rad": sample["euler_rad"],
                "angular_velocity_rps": sample["angular_velocity"],
                "linear_acceleration_mps2": sample["linear_acceleration"],
                "source_fresh": bool(source_status.get("source_fresh")),
                "age_seconds": source_status.get("age_seconds"),
                "topic": sample["topic"],
                "message_type": sample["message_type"],
            },
            "view": {
                "radius_m": max(
                    options["robot_length_m"],
                    options["robot_width_m"],
                    options["robot_height_m"],
                ) * 1.5,
                "units": "meters",
            },
        }
    error = str(source_status.get("error") or "")
    if state == "ready":
        euler = sample["euler_rad"]
        session["report"] = (
            "IMU live: "
            f"roll {math.degrees(euler['roll']):.1f}°, "
            f"pitch {math.degrees(euler['pitch']):.1f}°, "
            f"yaw {math.degrees(euler['yaw']):.1f}°"
        )
    elif state == "stale":
        session["report"] = "IMU Viewer is holding the last sample because the source is stale"
    else:
        session["report"] = error or "IMU Viewer started; waiting for orientation data"


def start_imu_viewer(
    *, viewer_id: str, node_id: str, source: dict[str, Any],
    source_reader: Callable[[dict[str, Any]], dict[str, Any]] | None,
    options: dict[str, float],
) -> dict[str, Any]:
    clean_id = _safe_id(viewer_id) or "imu_viewer"
    with _LOCK:
        session = {
            "viewer_id": clean_id,
            "node_id": str(node_id or ""),
            "source": dict(source),
            "source_reader": source_reader,
            "options": dict(options),
            "running": True,
            "live": False,
            "scene": {},
            "status": {},
            "report": "IMU Viewer started; waiting for orientation data",
        }
        _SESSIONS[clean_id] = session
        _update(session)
        return _outputs(session)


def imu_viewer_status(viewer_id: str) -> dict[str, Any]:
    clean_id = _safe_id(viewer_id)
    with _LOCK:
        session = _SESSIONS.get(clean_id)
        if session is None:
            return {
                "running": False,
                "live": False,
                "scene": {},
                "status": {"kind": "blacknode.viewer-status", "schema_version": 1, "state": "stopped"},
                "viewer": {"viewer_id": clean_id, "sensor": "imu", "state": "stopped"},
                "report": "IMU Viewer is stopped",
            }
        _update(session)
        return _outputs(session)


def stop_imu_viewer(viewer_id: str = "") -> dict[str, Any]:
    clean_id = _safe_id(viewer_id)
    with _LOCK:
        ids = [clean_id] if clean_id else list(_SESSIONS)
        stopped = sum(1 for session_id in ids if _SESSIONS.pop(session_id, None) is not None)
    return {"ok": True, "stopped": stopped}


def runtime_status() -> dict[str, Any]:
    node_outputs: list[dict[str, Any]] = []
    with _LOCK:
        for viewer_id, session in list(_SESSIONS.items()):
            _update(session)
            node_outputs.append({
                "node_type": "IMUViewer",
                "node_id": session.get("node_id", ""),
                "run_id": viewer_id,
                "outputs": _outputs(session),
            })
    return {
        "ok": all(item["outputs"].get("status", {}).get("state") != "error" for item in node_outputs),
        "active": bool(node_outputs),
        "streams": [],
        "managed_runs": [],
        "node_outputs": node_outputs,
        "detached_count": 0,
        "report": f"{len(node_outputs)} IMU Viewer session(s) active" if node_outputs else "no IMU Viewer sessions active",
    }


def stop_runtime_services() -> dict[str, Any]:
    stopped = int(stop_imu_viewer().get("stopped") or 0)
    return {
        "ok": True,
        "stopped": {"streams": stopped, "managed_runs": 0, "detached": 0},
        "report": f"stopped {stopped} IMU Viewer session(s)",
    }
