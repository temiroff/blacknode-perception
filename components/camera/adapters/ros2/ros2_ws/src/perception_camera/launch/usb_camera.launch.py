from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = {
        "device": "0",
        "image_topic": "/camera/image_raw",
        "camera_info_topic": "/camera/camera_info",
        "frame_id": "camera",
        "hz": "30.0",
        "width": "640",
        "height": "480",
        "rotation": "0",
        "backend": "v4l2",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in arguments.items()
            ],
            Node(
                package="perception_camera",
                executable="usb_camera",
                name="perception_camera",
                output="screen",
                parameters=[{
                    "device": ParameterValue(
                        LaunchConfiguration("device"),
                        value_type=str,
                    ),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration(
                        "camera_info_topic"
                    ),
                    "frame_id": LaunchConfiguration("frame_id"),
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
                    "rotation": ParameterValue(
                        LaunchConfiguration("rotation"),
                        value_type=int,
                    ),
                    "backend": LaunchConfiguration("backend"),
                }],
            ),
        ]
    )
