"""Image and colour helpers shared by the camera and tracking components.

This lives in the camera component because camera declares module-root, which
makes it the package module root and therefore importable by sibling
components as `blacknode.pkg.blacknode_perception.image_ops`.
"""
from __future__ import annotations

import base64
import json
import re
import time
import urllib.request
from pathlib import Path
from typing import Any

try:
    import cv2
    import numpy as np

    _CV2_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - machines without OpenCV
    cv2 = None
    np = None
    _CV2_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


_HSV_COLOR_RANGES: dict[str, tuple[tuple[int, int, int], tuple[int, int, int]]] = {
    "red": ((170, 80, 60), (10, 255, 255)),
    "orange": ((5, 80, 60), (25, 255, 255)),
    "yellow": ((20, 80, 80), (35, 255, 255)),
    "green": ((35, 60, 60), (85, 255, 255)),
    "cyan": ((85, 60, 60), (100, 255, 255)),
    "blue": ((100, 60, 50), (130, 255, 255)),
    "purple": ((130, 50, 50), (160, 255, 255)),
    "pink": ((145, 50, 80), (175, 255, 255)),
    "white": ((0, 0, 180), (179, 60, 255)),
    "black": ((0, 0, 0), (179, 255, 70)),
}


_COLOR_ALIASES: dict[str, str] = {
    "red": "red",
    "orange": "orange",
    "yellow": "yellow",
    "green": "green",
    "lime": "green",
    "cyan": "cyan",
    "turquoise": "cyan",
    "teal": "cyan",
    "blue": "blue",
    "purple": "purple",
    "violet": "purple",
    "magenta": "pink",
    "pink": "pink",
    "white": "white",
    "black": "black",
}


_OBJECT_WORDS = (
    "cube",
    "block",
    "box",
    "ball",
    "bottle",
    "cup",
    "marker",
    "object",
    "target",
)


def _normalize_words(value: Any) -> str:
    return re.sub(r"[^a-z0-9]+", " ", str(value or "").lower()).strip()


def _missing_cv2_outputs() -> dict[str, Any]:
    report = (
        "CV2 node FAILED: OpenCV is not installed in this Blacknode Python environment. "
        "Run: blacknode packages setup blacknode-perception"
    )
    if _CV2_IMPORT_ERROR:
        report += f" ({_CV2_IMPORT_ERROR})"
    return {
        "mask": "",
        "preview": "",
        "overlay": "",
        "found": False,
        "center_x": 0,
        "center_y": 0,
        "area": 0.0,
        "metadata": {},
        "detection": {},
        "detections": [],
        "report": report,
    }


def _parse_hsv(value: Any, default: tuple[int, int, int]) -> tuple[int, int, int]:
    raw = value if value not in (None, "") else default
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            parts = list(default)
        else:
            try:
                decoded = json.loads(text)
                parts = list(decoded) if isinstance(decoded, (list, tuple)) else []
            except json.JSONDecodeError:
                parts = [part for part in text.replace(";", ",").replace(" ", ",").split(",") if part]
    elif isinstance(raw, (list, tuple)):
        parts = list(raw)
    else:
        parts = list(default)
    values = list(default)
    for index, part in enumerate(parts[:3]):
        try:
            values[index] = int(float(part))
        except (TypeError, ValueError):
            values[index] = default[index]
    return (
        max(0, min(179, values[0])),
        max(0, min(255, values[1])),
        max(0, min(255, values[2])),
    )


def _format_hsv(value: tuple[int, int, int]) -> str:
    return ",".join(str(int(part)) for part in value)


def _find_color(value: Any) -> str:
    text = _normalize_words(value)
    if not text:
        return ""
    words = set(text.split())
    for alias, color in _COLOR_ALIASES.items():
        if alias in words:
            return color
    return ""


def _find_object_label(value: Any, fallback: str) -> str:
    text = _normalize_words(value)
    if text:
        words = set(text.split())
        for word in _OBJECT_WORDS:
            if word in words:
                return word
    return fallback.strip() or "object"


def _read_reasoning_state_answer(state_url: str, wait_seconds: float) -> tuple[str, str]:
    url = state_url.strip()
    if not url:
        return "", ""
    deadline = time.monotonic() + max(0.0, wait_seconds)
    last_error = ""
    while True:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "BlacknodeCV2TargetHint/0.1"})
            with urllib.request.urlopen(req, timeout=1.5) as response:
                payload = json.loads(response.read().decode("utf-8"))
            answer = str(payload.get("answer") or "").strip()
            report = str(payload.get("report") or "").strip()
            if answer:
                return answer, ""
            last_error = report or "reasoning state has no answer yet"
        except Exception as exc:  # noqa: BLE001
            last_error = f"{type(exc).__name__}: {exc}"
        if time.monotonic() >= deadline:
            break
        time.sleep(0.35)
    return "", last_error


def _bool_value(value: Any, default: bool = False) -> bool:
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _decode_image_bgr(source: Any) -> tuple[Any, str]:
    if cv2 is None or np is None:
        return None, _missing_cv2_outputs()["report"]

    text = str(source or "").strip()
    if not text:
        return None, "CV2 node FAILED: no image provided"

    try:
        if text.startswith("data:"):
            if "," not in text:
                return None, "CV2 node FAILED: invalid image data URL"
            raw = base64.b64decode(text.split(",", 1)[1])
            data = np.frombuffer(raw, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        elif text.startswith(("http://", "https://")):
            with urllib.request.urlopen(text, timeout=20) as response:
                raw = response.read()
            data = np.frombuffer(raw, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        else:
            path = Path(text).expanduser()
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    except Exception as exc:  # noqa: BLE001
        return None, f"CV2 node FAILED: could not decode image: {type(exc).__name__}: {exc}"

    if image is None:
        return None, "CV2 node FAILED: could not decode image; use PNG/JPEG data URL, URL, or file path"
    return image, ""
