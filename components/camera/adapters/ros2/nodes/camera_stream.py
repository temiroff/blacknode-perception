"""Camera capability over ROS 2: image-topic streams, USB camera bridging, and
robot web_video_server viewing.

These adapt the perception camera capability to a ROS 2 graph. The transport
plumbing (running ``ros2``, bridging MJPEG in/out of the graph) is provided by
``blacknode-ros2/core``; every node here returns a structured report instead of
raising, so workflows stay usable on machines without ROS.
"""
from __future__ import annotations

import re
import shlex
import time
import urllib.parse

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, Text, node


class _LazyRos2Runtime:
    """Delay ROS 2 imports so dependency discovery can load in any folder order."""

    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import ros2_runtime

        return getattr(ros2_runtime, name)


rt = _LazyRos2Runtime()

_CATEGORY = "Perception"
_PROVIDER_PROFILES = {
    "existing_topics",
    "usb_cam",
    "blacknode_rgbd",
    "custom_launch",
}


def _provider_profile(ctx: dict) -> str:
    profile = str(ctx.get("profile") or "existing_topics").strip().lower()
    if profile not in _PROVIDER_PROFILES and bool(ctx.get("require_depth")):
        return "blacknode_rgbd"
    return profile


def _service_id(value: object) -> str:
    return (
        re.sub(r"[^a-zA-Z0-9_-]+", "_", str(value or "camera_provider").strip())
        .strip("_")[:64]
        or "camera_provider"
    )


def _camera_interfaces(ctx: dict) -> list[dict]:
    rgbd = _provider_profile(ctx) == "blacknode_rgbd"
    rgb_default = "/camera/rgb/image_raw" if rgbd else "/camera/image_raw"
    rgb_info_default = (
        "/camera/rgb/camera_info" if rgbd else "/camera/camera_info"
    )
    supplied_rgb = str(ctx.get("rgb_topic") or "").strip()
    supplied_rgb_info = str(ctx.get("rgb_info_topic") or "").strip()
    if rgbd and supplied_rgb == "/camera/image_raw":
        supplied_rgb = ""
    if rgbd and supplied_rgb_info == "/camera/camera_info":
        supplied_rgb_info = ""
    interfaces = [{
        "name": "rgb_image",
        "topic": supplied_rgb or rgb_default,
        "message_type": "sensor_msgs/msg/Image",
        "required": True,
    }]
    optional = [
        (
            "rgb_camera_info",
            "rgb_info_topic",
            rgb_info_default,
            "sensor_msgs/msg/CameraInfo",
        ),
        (
            "depth_image",
            "depth_topic",
            "/camera/depth/image_raw" if rgbd else "",
            "sensor_msgs/msg/Image",
        ),
        (
            "depth_camera_info",
            "depth_info_topic",
            "/camera/depth/camera_info" if rgbd else "",
            "sensor_msgs/msg/CameraInfo",
        ),
        (
            "point_cloud",
            "points_topic",
            "",
            "sensor_msgs/msg/PointCloud2",
        ),
    ]
    require_depth = rgbd or bool(ctx.get("require_depth", False))
    for name, field, default, message_type in optional:
        topic = str(ctx.get(field) or default).strip()
        if topic:
            interfaces.append({
                "name": name,
                "topic": topic,
                "message_type": message_type,
                "required": require_depth and name == "depth_image",
            })
    return interfaces


def _provider_command(ctx: dict) -> tuple[list[str], str]:
    profile = _provider_profile(ctx)
    try:
        extra = shlex.split(str(ctx.get("arguments") or ""))
    except ValueError as exc:
        return [], f"invalid provider arguments: {exc}"
    if profile == "existing_topics":
        return [], ""
    if profile == "usb_cam":
        rgb_topic = str(ctx.get("rgb_topic") or "/camera/image_raw").strip()
        rgb_info_topic = str(
            ctx.get("rgb_info_topic") or "/camera/camera_info"
        ).strip()
        return [
            "launch",
            "perception_camera",
            "usb_camera.launch.py",
            f"image_topic:={rgb_topic}",
            f"camera_info_topic:={rgb_info_topic}",
            *extra,
        ], ""
    if profile == "blacknode_rgbd":
        return [
            "launch",
            "perception_camera",
            "rgbd_camera.launch.py",
            *extra,
        ], ""
    if profile == "custom_launch":
        package = str(ctx.get("package") or "").strip()
        launch_file = str(ctx.get("launch_file") or "").strip()
        if not package or not launch_file:
            return [], "custom launch requires package and launch_file"
        return ["launch", package, launch_file, *extra], ""
    return [], (
        "profile must be existing_topics, usb_cam, blacknode_rgbd, or "
        "custom_launch"
    )


@node(
    name="CameraROS2Provider",
    live=True,
    category=_CATEGORY,
    description=(
        "Start, stop, or inspect a managed ROS 2 RGB/RGB-D camera provider, "
        "then verify its expected topic group."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "status", "stop"], default="status"),
        "run_id": Text(default="camera_provider"),
        "profile": Enum(
            [
                "existing_topics",
                "usb_cam",
                "blacknode_rgbd",
                "custom_launch",
            ],
            default="existing_topics",
        ),
        "package": Text(default=""),
        "launch_file": Text(default=""),
        "arguments": Text(default=""),
        "rgb_topic": Text(default="/camera/image_raw"),
        "rgb_info_topic": Text(default="/camera/camera_info"),
        "depth_topic": Text(default=""),
        "depth_info_topic": Text(default=""),
        "points_topic": Text(default=""),
        "require_depth": Bool(default=False),
        "wait_seconds": Float(default=20.0),
    },
    outputs={
        "running": Bool,
        "ready": Bool,
        "run_id": Text,
        "profile": Text,
        "interfaces": Dict,
        "report": Text,
    },
)
def ros2_camera_provider(ctx: dict) -> dict:
    run_id = _service_id(ctx.get("run_id"))
    profile = _provider_profile(ctx)
    action = str(ctx.get("action") or "status").strip().lower()
    command, command_error = _provider_command(ctx)
    base = {
        "running": False,
        "ready": False,
        "run_id": run_id,
        "profile": profile,
        "interfaces": {},
    }
    if command_error:
        return {**base, "report": f"camera provider FAILED: {command_error}"}

    if action == "stop":
        if not command:
            return {
                **base,
                "report": (
                    "existing ROS 2 topics are externally managed; no camera "
                    "provider process was stopped"
                ),
            }
        result = rt.stop_ros2_managed(
            run_id,
            pattern="ros2 " + " ".join(command[:3]),
        )
        return {
            **base,
            "report": (
                f"stopped camera provider {run_id}"
                if result.get("ok")
                else f"camera provider stop FAILED: {result.get('error')}"
            ),
        }

    process_status = {
        "ok": True,
        "running": profile == "existing_topics",
        "backend": "",
    }
    if action == "start" and command:
        process_status = rt.run_ros2_managed(run_id, command)
        process_status["running"] = bool(process_status.get("ok"))
    elif command:
        process_status = rt.ros2_managed_status(run_id)

    if not process_status.get("ok"):
        return {
            **base,
            "report": (
                "camera provider FAILED: "
                + str(process_status.get("error") or "could not inspect process")
            ),
        }

    expected = _camera_interfaces(ctx)
    if action == "start":
        topic_status = rt.wait_for_topic_interfaces(
            expected,
            timeout=max(0.0, float(ctx.get("wait_seconds") or 0.0)),
        )
    else:
        topic_status = rt.inspect_topic_interfaces(expected)
    running = bool(process_status.get("running"))
    ready = bool(topic_status.get("ready"))
    if profile == "existing_topics":
        running = ready
    missing = ", ".join(topic_status.get("missing") or [])
    return {
        **base,
        "running": running,
        "ready": ready,
        "interfaces": topic_status,
        "report": (
            f"camera provider {run_id} is ready via "
            f"{topic_status.get('backend') or process_status.get('backend') or '?'}"
            if ready
            else (
                f"camera provider {run_id} is running, but required topics need "
                f"attention: {missing or topic_status.get('error') or 'unknown'}"
                if running
                else (
                    f"camera provider {run_id} is stopped; required topics need "
                    f"attention: {missing or topic_status.get('error') or 'unknown'}"
                )
            )
        ),
    }


def _resolve_image_message_type(topic: str, requested: str) -> tuple[str, str]:
    value = requested.strip().lower()
    if value in {"raw", "compressed"}:
        return value, ""
    result = rt.run_ros2(["topic", "type", topic], timeout=10)
    if not result.get("ok"):
        # `ros2 topic type` only resolves topics with an active publisher/
        # subscriber right now, and fails with an opaque "exited with code 1"
        # (often no stderr) when nothing is publishing yet. Check topic list
        # membership so the failure explains what to fix instead of just the
        # bare exit code.
        listing = rt.run_ros2(["topic", "list"], timeout=10)
        known_topics = {
            line.strip() for line in listing.get("stdout", "").splitlines() if line.strip()
        } if listing.get("ok") else set()
        if listing.get("ok") and topic not in known_topics:
            return "", (
                f"{topic} has no active publisher right now. Start a camera driver that "
                f"publishes to {topic} (a Camera node, or the 'Camera Livestream' "
                f"template) before starting the stream, or set 'topic' to a topic ros2 "
                f"topic list already shows."
            )
        return "", result.get("error", "could not discover topic type")
    types = [line.strip() for line in result.get("stdout", "").splitlines() if line.strip()]
    if any("sensor_msgs/msg/CompressedImage" in line for line in types):
        return "compressed", ""
    if any("sensor_msgs/msg/Image" in line for line in types):
        return "raw", ""
    return "", f"{topic} is not a sensor_msgs Image topic (types: {', '.join(types) or 'none'})"


@node(
    name="CameraROS2Subscribe",
    live=True,
    category=_CATEGORY,
    description="Live camera feed for a raw or compressed ROS 2 image topic. Go Live streams continuous MJPEG; a plain one-shot Run captures a single frame instead.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "stream_id": Text(default="camera"),
        "topic": Text(default="/camera/image_raw"),
        "message_type": Enum(["auto", "raw", "compressed"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=0),
        "max_fps": Float(default=10.0),
        "max_width": Int(default=960),
        "jpeg_quality": Int(default=80),
    },
    outputs={
        "preview": Image,
        "streaming": Bool,
        "stream_url": Text,
        "snapshot_url": Text,
        "stream_id": Text,
        "report": Text,
        "frame_stream": Dict,
    },
)
def ros2_image_stream(ctx: dict) -> dict:
    stream_id = str(ctx.get("stream_id") or "camera").strip() or "camera"
    action = str(ctx.get("action") or "start").strip().lower()
    if action == "stop":
        result = rt.stop_image_stream(stream_id)
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"stopped {result.get('stopped', 0)} image stream(s)",
        }

    topic = str(ctx.get("topic") or "/camera/image_raw").strip()
    message_type, error = _resolve_image_message_type(topic, str(ctx.get("message_type") or "auto"))
    if error:
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"image stream FAILED: {error}",
        }

    host = str(ctx.get("host") or "127.0.0.1").strip() or "127.0.0.1"
    port = max(0, int(ctx.get("port") or 0))
    max_fps = max(0.1, min(60.0, float(ctx.get("max_fps") or 10.0)))
    max_width = max(0, int(ctx.get("max_width") or 960))
    jpeg_quality = max(1, min(100, int(ctx.get("jpeg_quality") or 80)))

    if ctx.get("__run_mode__") == "once":
        # A one-shot Run has nothing to keep watching a persistent MJPEG URL,
        # and leaving a background stream server running after it would leak.
        # Return a single real frame instead; Go Live starts the live stream.
        shot = rt.capture_image_snapshot(
            topic=topic,
            message_type=message_type,
            timeout=max(1.0, 15.0),
            output_format="jpeg",
            jpeg_quality=jpeg_quality,
        )
        if not shot.get("ok"):
            return {
                "preview": "",
                "streaming": False,
                "stream_url": "",
                "snapshot_url": "",
                "stream_id": stream_id,
                "report": f"image frame FAILED: {shot.get('error', 'unknown error')}",
            }
        metadata = dict(shot.get("metadata") or {})
        return {
            "preview": str(shot.get("image") or ""),
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": (
                f"captured one {metadata.get('width', '?')}x{metadata.get('height', '?')} "
                f"{message_type} frame from {topic} — press Go Live for a continuous stream"
            ),
        }

    result = rt.start_image_stream(
        stream_id=stream_id,
        topic=topic,
        message_type=message_type,
        host=host,
        port=port,
        max_fps=max_fps,
        max_width=max_width,
        jpeg_quality=jpeg_quality,
    )
    if not result.get("ok"):
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"image stream FAILED: {result.get('error', 'unknown error')}",
        }
    stream_url = str(result["stream_url"])
    snapshot_url = str(result["snapshot_url"])
    report = (
        f"LIVE STREAM running on {stream_url} from {topic} "
        f"({message_type}, {max_fps:g} FPS max, width {max_width or 'source'})"
    )
    return {
        "preview": stream_url,
        "streaming": True,
        "stream_url": stream_url,
        "snapshot_url": snapshot_url,
        "stream_id": stream_id,
        "frame_stream": {
            "kind": "blacknode.frame-stream",
            "schema_version": 1,
            "stream_id": stream_id,
            "stream_url": stream_url,
            "snapshot_url": snapshot_url,
            "health_url": str(result.get("health_url") or ""),
            "media_type": "image/jpeg",
            "mode": "latest",
            "clock": "unix_ns",
            "topic": topic,
        },
        "report": report,
    }


@node(
    name="CameraROS2Publish",
    live=True,
    category=_CATEGORY,
    description=(
        "Publish any camera stream onto a ROS 2 sensor_msgs/Image topic, and show the picture "
        "read back from ROS as proof the topic carries it. Wire a Camera (or any node emitting "
        "a frame stream) into frame_stream."
    ),
    primary_inputs=["frame_stream"],
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "frame_stream": Dict(default={}),
        "topic": Text(default="/camera/image_raw"),
        "max_fps": Float(default=15.0),
        "max_width": Int(default=640),
        "jpeg_quality": Int(default=80),
        "wait_seconds": Float(default=25.0),
    },
    outputs={
        "preview": Image,
        "streaming": Bool,
        "topic": Text,
        "camera": Text,
        "stream_url": Text,
        "report": Text,
        "frame_stream": Dict,
    },
)
def ros2_usb_camera(ctx: dict) -> dict:
    """Bridge an already-running camera stream onto a ROS 2 image topic.

    Capture is somebody else's job: wire a Camera (or any node that emits a
    frame stream) into frame_stream. This node only publishes it and reads it
    back, so the preview proves the topic really carries the frames.

    Docker cannot open a USB camera, so capture always happens on this machine
    and only the MJPEG URL crosses into the ROS graph.
    """
    topic = str(ctx.get("topic") or "/camera/image_raw").strip() or "/camera/image_raw"
    stream_id = "ros2_usb_camera"
    blank = {"preview": "", "streaming": False, "topic": topic, "camera": "", "stream_url": ""}

    if str(ctx.get("action") or "start").strip().lower() == "stop":
        rt.stop_host_camera_publisher(stream_id)
        rt.stop_image_stream(stream_id)
        return {**blank, "report": "stopped the ROS publisher and the preview"}

    frame_stream = ctx.get("frame_stream") if isinstance(ctx.get("frame_stream"), dict) else {}
    label = str(frame_stream.get("label") or frame_stream.get("stream_id") or "camera")
    source_url = str(frame_stream.get("stream_url") or "")
    if not source_url:
        # Streams published before stream_url joined the contract only carry the
        # single-frame snapshot; the MJPEG endpoint is its sibling on that server.
        snapshot = str(frame_stream.get("snapshot_url") or "")
        if snapshot.endswith("/snapshot.jpg"):
            source_url = snapshot[: -len("/snapshot.jpg")] + "/stream.mjpg"

    if not frame_stream:
        return {**blank, "report": (
            "publish FAILED: nothing wired to 'frame_stream'.\n"
            "CHECK: connect a Camera node's frame_stream output to this input and cook it "
            "first, so a live stream exists to publish."
        )}
    if not source_url:
        return {**blank, "report": (
            "publish FAILED: the wired frame stream carries no video URL.\n"
            f"CHECK: it provided {sorted(frame_stream)}. Cook the upstream camera so it is "
            "streaming before publishing it."
        )}

    # 2. bridge it into the ROS graph as a real image topic
    published = rt.start_host_camera_publisher(
        run_id=stream_id,
        source_url=source_url,
        topic=topic,
        frame_id="camera_frame",
        max_fps=max(0.1, float(ctx.get("max_fps") or 15.0)),
    )
    if not published.get("ok"):
        return {**blank, "camera": label, "report": (
            f"publish FAILED to reach ROS: {published.get('error', 'unknown error')}"
        )}

    wait_seconds = max(0.0, float(ctx.get("wait_seconds") or 25.0))
    deadline = time.time() + wait_seconds
    discovered = False
    while time.time() < deadline:
        check = rt.run_ros2(["topic", "list"], timeout=10)
        topics = {line.strip().split()[0] for line in check.get("stdout", "").splitlines() if line.strip()}
        if check.get("ok") and topic in topics:
            discovered = True
            break
        time.sleep(1)
    if not discovered:
        return {**blank, "camera": label, "report": (
            f"the stream is publishing, but {topic} did not appear on the ROS graph within "
            f"{wait_seconds:g}s. The camera itself is fine — this is the ROS side."
        )}

    # 3. read it back *out of ROS* so the picture proves the topic works
    message_type, error = _resolve_image_message_type(topic, "auto")
    if not error:
        shown = rt.start_image_stream(
            stream_id=stream_id,
            topic=topic,
            message_type=message_type,
            host="127.0.0.1",
            port=0,
            max_fps=max(0.1, float(ctx.get("max_fps") or 15.0)),
            max_width=max(0, int(ctx.get("max_width") or 640)),
            jpeg_quality=max(1, min(100, int(ctx.get("jpeg_quality") or 80))),
        )
        if shown.get("ok"):
            url = str(shown["stream_url"])
            return {
                "preview": url,
                "streaming": True,
                "topic": topic,
                "camera": label,
                "stream_url": url,
                "report": (
                    f"LIVE: '{label}' publishing to {topic} on ROS, shown from the ROS topic "
                    f"via the {published['backend']} backend"
                ),
            }
        error = str(shown.get("error", "could not read the topic back"))

    # ROS has the topic but reading it back failed; still report the live topic.
    return {
        "preview": "",
        "streaming": True,
        "topic": topic,
        "camera": label,
        "stream_url": "",
        "report": (
            f"'{label}' is publishing to {topic} on ROS, but the preview could not be "
            f"read back: {error}"
        ),
    }


def _web_video_url(host: str, port: int, topic: str, quality: int, width: int, height: int) -> str:
    params = [f"topic={urllib.parse.quote(topic, safe='/')}", "type=mjpeg"]
    if quality > 0:
        params.append(f"quality={quality}")
    if width > 0:
        params.append(f"width={width}")
    if height > 0:
        params.append(f"height={height}")
    return f"http://{host}:{port}/stream?{'&'.join(params)}"


@node(
    name="CameraROS2Http",
    live=True,
    category=_CATEGORY,
    description=(
        "Watch a camera topic published by a robot running web_video_server. The robot "
        "serves MJPEG over HTTP, so this needs no local ROS graph and works even when DDS "
        "discovery cannot reach the robot."
    ),
    inputs={
        "trigger": AnyPort,
        "host": Text(default="ROBOT_IP"),
        "port": Int(default=8080),
        "topic": Text(default="/camera/image_raw"),
        "quality": Int(default=80),
        "width": Int(default=0),
        "height": Int(default=0),
        "timeout": Float(default=10.0),
    },
    outputs={"preview": Image, "streaming": Bool, "stream_url": Text, "report": Text, "frame_stream": Dict,},
)
def ros2_web_video_stream(ctx: dict) -> dict:
    host = str(ctx.get("host") or "").strip()
    port = int(ctx.get("port") or 8080)
    topic = str(ctx.get("topic") or "/camera/image_raw").strip() or "/camera/image_raw"
    timeout = max(1.0, float(ctx.get("timeout") or 10.0))
    blank = {"preview": "", "streaming": False, "stream_url": ""}

    if not host or host == "ROBOT_IP":
        return {
            **blank,
            "report": (
                "robot camera FAILED: set 'host' to your robot's IP address "
                "(the machine running web_video_server), e.g. 192.168.1.50"
            ),
        }

    url = _web_video_url(
        host, port, topic,
        max(0, min(100, int(ctx.get("quality") or 0))),
        max(0, int(ctx.get("width") or 0)),
        max(0, int(ctx.get("height") or 0)),
    )

    # Probe before handing the URL to the canvas: a broken <img> is never
    # retried, so a silent failure would just show an empty node forever.
    ok, detail = rt.probe_web_video(url, timeout)
    if not ok:
        return {
            **blank,
            "report": (
                f"robot camera FAILED: {detail}\n"
                f"tried {url}\n"
                f"CHECK: is the robot at {host} powered and on this network, is web_video_server "
                f"running on port {port}, and does it publish '{topic}'? "
                f"Open http://{host}:{port}/ in a browser to list the robot's camera topics."
            ),
        }

    return {
        "preview": url,
        "streaming": True,
        "stream_url": url,
        "report": f"LIVE robot camera from {topic} on {host}:{port}",
    }
