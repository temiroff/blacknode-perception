from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = {
        "rgb_device": "0",
        "depth_device": "1",
        "rgb_topic": "/camera/rgb/image_raw",
        "rgb_info_topic": "/camera/rgb/camera_info",
        "depth_topic": "/camera/depth/image_raw",
        "depth_info_topic": "/camera/depth/camera_info",
        "rgb_frame_id": "camera_rgb",
        "depth_frame_id": "camera_depth",
        "hz": "30.0",
        "width": "640",
        "height": "480",
        "rgb_backend": "v4l2",
        "depth_backend": "v4l2",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in arguments.items()
            ],
            Node(
                package="perception_camera",
                executable="rgbd_camera",
                name="perception_rgbd_camera",
                output="screen",
                parameters=[{
                    "rgb_device": LaunchConfiguration("rgb_device"),
                    "depth_device": LaunchConfiguration("depth_device"),
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "rgb_info_topic": LaunchConfiguration("rgb_info_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "depth_info_topic": LaunchConfiguration(
                        "depth_info_topic"
                    ),
                    "rgb_frame_id": LaunchConfiguration("rgb_frame_id"),
                    "depth_frame_id": LaunchConfiguration("depth_frame_id"),
                    "hz": ParameterValue(
                        LaunchConfiguration("hz"),
                        value_type=float,
                    ),
                    "width": ParameterValue(
                        LaunchConfiguration("width"),
                        value_type=int,
                    ),
                    "height": ParameterValue(
                        LaunchConfiguration("height"),
                        value_type=int,
                    ),
                    "rgb_backend": LaunchConfiguration("rgb_backend"),
                    "depth_backend": LaunchConfiguration("depth_backend"),
                }],
            ),
        ]
    )
