"""Camera-specific presentation for portable frame streams."""
from __future__ import annotations

from blacknode.node import Dict, Image, Text, node


@node(
    name="CameraViewer",
    component="camera",
    category="Camera",
    description="Show a processed camera frame stream with camera health and no spatial-map controls.",
    inputs={"source": Dict, "health": Dict, "label": Text(default="Camera")},
    outputs={"preview": Image, "status": Dict, "report": Text},
    primary_inputs=["source", "health"],
    primary_outputs=["preview", "status"],
)
def camera_viewer(ctx: dict) -> dict:
    source = ctx.get("source") if isinstance(ctx.get("source"), dict) else {}
    health = ctx.get("health") if isinstance(ctx.get("health"), dict) else {}
    valid = source.get("kind") == "blacknode.frame-stream"
    preview = str(source.get("stream_url") or source.get("snapshot_url") or "") if valid else ""
    source_state = str(health.get("state") or ("ready" if preview else "waiting"))
    ready = bool(valid and preview and source_state not in {"error", "stale", "stopped", "unavailable"})
    error = "" if valid or not source else "source must be a blacknode.frame-stream"
    status = {
        "kind": "blacknode.viewer-status",
        "schema_version": 1,
        "viewer_role": "camera",
        "state": "ready" if ready else source_state if valid else "waiting" if not source else "unavailable",
        "source_fresh": bool(health.get("source_fresh", ready)),
        "error": error or str(health.get("error") or ""),
        "label": str(ctx.get("label") or "Camera"),
    }
    return {
        "preview": preview,
        "status": status,
        "report": f"Camera viewer {status['state']}",
    }
