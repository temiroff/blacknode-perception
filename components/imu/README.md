# IMU

The IMU component exposes normalized orientation, angular velocity, linear
acceleration, health, and attachment metadata through `IMU`. Applications use
that capability contract independently of the physical sensor or transport.

`IMUTestProvider` supplies deterministic roll, pitch, and yaw samples or
replays a recorded `blacknode.imu-stream`, so workflows and the viewer can be
tested with no sensor attached.

`IMUViewer` is a managed live diagnostic surface. Connect either an `IMU`
capability/sample or a generic `ROS2.stream` carrying
`sensor_msgs/msg/Imu`. The editor shows a 3D B-logo robot, body and world XYZ
axes, roll/pitch/yaw, angular velocity, acceleration, source frame, and
freshness. It rejects zero-length quaternions and reports ROS IMU messages whose
orientation covariance marks orientation unavailable.

For a real robot, configure the generic `ROS2` node with the robot's IMU topic
(commonly `/imu/data`), `sensor_msgs/msg/Imu`, sensor-data QoS, and `action =
start`; connect `ROS2.stream` to `IMUViewer.source`, then use **Go live**.
