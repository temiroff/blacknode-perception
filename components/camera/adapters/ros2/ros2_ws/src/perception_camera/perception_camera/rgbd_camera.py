from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def _device(value):
    return int(value) if isinstance(value, str) and value.isdigit() else value


def _backend(value: str) -> int:
    return cv2.CAP_V4L2 if value.strip().lower() in {"", "v4l2"} else cv2.CAP_ANY


class RgbdCamera(Node):
    """Publish synchronized RGB and metric-depth streams from two video inputs."""

    def __init__(self) -> None:
        super().__init__("perception_rgbd_camera")
        defaults = {
            "rgb_device": "0",
            "depth_device": "1",
            "rgb_topic": "/camera/rgb/image_raw",
            "rgb_info_topic": "/camera/rgb/camera_info",
            "depth_topic": "/camera/depth/image_raw",
            "depth_info_topic": "/camera/depth/camera_info",
            "rgb_frame_id": "camera_rgb",
            "depth_frame_id": "camera_depth",
            "hz": 30.0,
            "width": 640,
            "height": 480,
            "rgb_backend": "v4l2",
            "depth_backend": "v4l2",
            "warmup_frames": 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.rgb_frame_id = str(self.get_parameter("rgb_frame_id").value)
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        hz = max(0.1, float(self.get_parameter("hz").value))
        rgb_device = _device(self.get_parameter("rgb_device").value)
        depth_device = _device(self.get_parameter("depth_device").value)
        self.rgb_cap = cv2.VideoCapture(
            rgb_device,
            _backend(str(self.get_parameter("rgb_backend").value)),
        )
        self.depth_cap = cv2.VideoCapture(
            depth_device,
            _backend(str(self.get_parameter("depth_backend").value)),
        )
        self.depth_cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        for capture in (self.rgb_cap, self.depth_cap):
            if width > 0:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height > 0:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.rgb_cap.isOpened():
            self._release()
            raise RuntimeError(f"Could not open RGB camera device {rgb_device!r}")
        if not self.depth_cap.isOpened():
            self._release()
            raise RuntimeError(
                f"Could not open depth camera device {depth_device!r}. "
                "Set depth_device to the video input that provides 16-bit or "
                "32-bit metric depth."
            )

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(
            Image,
            str(self.get_parameter("rgb_topic").value),
            10,
        )
        self.rgb_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("rgb_info_topic").value),
            10,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("depth_topic").value),
            10,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("depth_info_topic").value),
            10,
        )
        self.last_warn = 0.0
        for _ in range(max(0, int(self.get_parameter("warmup_frames").value))):
            self.rgb_cap.read()
            self.depth_cap.read()
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"publishing RGB device {rgb_device!r} and metric-depth device "
            f"{depth_device!r} at {hz:g} Hz"
        )

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warn > 2.0:
            self.get_logger().warn(message)
            self.last_warn = now

    def tick(self) -> None:
        rgb_ok, rgb = self.rgb_cap.read()
        depth_ok, depth = self.depth_cap.read()
        if not rgb_ok or not depth_ok:
            self._warn("RGB-D frame read failed")
            return
        depth_encoding = ""
        if depth.ndim == 3 and np.array_equal(depth[..., 0], depth[..., 1]) and np.array_equal(
            depth[..., 1], depth[..., 2]
        ):
            depth = depth[..., 0]
        if depth.ndim == 2 and depth.dtype == np.uint16:
            depth_encoding = "16UC1"
        elif depth.ndim == 2 and depth.dtype == np.float32:
            depth_encoding = "32FC1"
        else:
            self._warn(
                "Depth input is not metric 16UC1 or 32FC1 data; no depth "
                "message was published. Select the camera's metric-depth input."
            )
            return

        stamp = self.get_clock().now().to_msg()
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.rgb_frame_id
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding=depth_encoding)
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.depth_frame_id
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.rgb_info_pub.publish(self._info(rgb_msg))
        self.depth_info_pub.publish(self._info(depth_msg))

    @staticmethod
    def _info(image: Image) -> CameraInfo:
        info = CameraInfo()
        info.header = image.header
        info.width = int(image.width)
        info.height = int(image.height)
        return info

    def _release(self) -> None:
        for capture in (getattr(self, "rgb_cap", None), getattr(self, "depth_cap", None)):
            if capture is not None:
                capture.release()

    def destroy_node(self) -> bool:
        self._release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RgbdCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
