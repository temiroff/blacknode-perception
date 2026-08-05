"""Depth-image presentation separated from metric 3D reconstruction."""
from __future__ import annotations

from blacknode.node import Bool, Dict, Enum, Float, Image, Text, node


@node(
    name="DepthViewer",
    component="depth",
    category="Perception",
    description=(
        "Show a processed metric depth stream as an image diagnostic. "
        "Use DepthCloudViewer after WarpDepthProjector for calibrated 3D points."
    ),
    inputs={
        "source": Dict,
        "health": Dict,
        "label": Text(default="Depth"),
        "auto_range": Bool(default=True),
        "near_m": Float(default=0.2),
        "far_m": Float(default=2.0),
        "palette": Enum(["grayscale", "turbo"], default="grayscale"),
        "invalid_color": Enum(["black", "magenta"], default="black"),
    },
    outputs={"preview": Image, "status": Dict, "report": Text},
    primary_inputs=["source", "health"],
    primary_outputs=["preview", "status"],
)
def depth_viewer(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    health = ctx.get("health") if isinstance(ctx.get("health"), dict) else {}
    valid = source.get("kind") == "blacknode.depth-stream"
    depth_scale = max(0.0, float(source.get("depth_scale") or 0.0))
    auto_range = bool(ctx.get("auto_range", True))
    near_m = max(0.0, float(ctx.get("near_m") or 0.2))
    far_m = max(near_m + 0.001, float(ctx.get("far_m") or 2.0))
    palette = str(ctx.get("palette") or "grayscale")
    invalid_color = str(ctx.get("invalid_color") or "black")
    preview = str(source.get("stream_url") or source.get("snapshot_url") or "") if valid else ""
    frame_source = source.get("frame_source") if isinstance(source.get("frame_source"), dict) else {}
    nested_health = source.get("health") if isinstance(source.get("health"), dict) else {}
    effective_health = health or nested_health
    source_state = str(effective_health.get("state") or ("ready" if preview else "waiting"))
    ready = bool(valid and preview and source_state not in {"error", "stale", "stopped", "unavailable"})
    error = "" if valid or not source else "source must be a blacknode.depth-stream"
    status = {
        "kind": "blacknode.viewer-status",
        "schema_version": 1,
        "viewer_role": "depth",
        "state": "ready" if ready else source_state if valid else "waiting" if not source else "unavailable",
        "source_fresh": bool(effective_health.get("source_fresh", ready)),
        "error": error or str(effective_health.get("error") or ""),
        "label": str(ctx.get("label") or "Depth"),
        "encoding": str(source.get("encoding") or ""),
        "depth_scale": depth_scale,
        "frame_url": str(frame_source.get("url") or ""),
        "display": {
            "range": "auto" if auto_range else "fixed",
            "near_m": near_m,
            "far_m": far_m,
            "palette": palette,
            "invalid_color": invalid_color,
        },
    }
    return {
        "preview": preview,
        "status": status,
        "report": f"Depth viewer {status['state']}",
    }
