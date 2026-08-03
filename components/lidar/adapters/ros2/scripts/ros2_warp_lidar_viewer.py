"""Interactive ROS 2 LaserScan viewer using blacknode-cuda Warp processing."""
from __future__ import annotations

import argparse
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _scan_dict(message: LaserScan) -> dict[str, Any]:
    return {
        "kind": "blacknode.laser-scan-stream",
        "schema_version": 1,
        "frame": str(message.header.frame_id or "laser"),
        "source_time_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
        "receive_time_ns": time.time_ns(),
        "angle_min": float(message.angle_min),
        "angle_max": float(message.angle_max),
        "angle_increment": float(message.angle_increment),
        "range_min": float(message.range_min),
        "range_max": float(message.range_max),
        "ranges": [float(value) for value in message.ranges],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="View ROS 2 LaserScan points through NVIDIA Warp")
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--filter-min", type=float, default=0.1)
    parser.add_argument("--filter-max", type=float, default=12.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--sensor-x", type=float, default=0.0)
    parser.add_argument("--sensor-y", type=float, default=0.0)
    parser.add_argument("--sensor-yaw", type=float, default=0.0)
    parser.add_argument("--point-radius", type=float, default=0.025)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--show-raw", action="store_true")
    parser.add_argument("--show-filtered", action="store_true")
    args = parser.parse_args()

    import blacknode  # noqa: F401 - installs stable extension-package aliases
    from blacknode.pkg.blacknode_cuda.warp_points import run_viewer_loop

    latest: dict[str, Any] = {}
    lock = threading.Lock()
    rclpy.init()
    node = Node("blacknode_warp_lidar_viewer")

    def on_scan(message: LaserScan) -> None:
        with lock:
            latest.clear()
            latest.update(_scan_dict(message))

    subscription = node.create_subscription(LaserScan, args.topic, on_scan, qos_profile_sensor_data)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    print(f"waiting for sensor_msgs/LaserScan on {args.topic}", flush=True)
    try:
        run_viewer_loop(
            scan_source=lambda: dict(latest),
            device=args.device,
            filter_min_m=args.filter_min,
            filter_max_m=args.filter_max,
            stride=max(1, args.stride),
            sensor_pose=(args.sensor_x, args.sensor_y, args.sensor_yaw),
            show_raw=args.show_raw,
            show_filtered=args.show_filtered,
            point_radius=args.point_radius,
            fps=max(1, args.fps),
            title=f"Blacknode LiDAR — {args.topic}",
        )
    finally:
        stop_event.set()
        thread.join(timeout=1.0)
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
