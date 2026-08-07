# IMU

The IMU component exposes normalized orientation, angular velocity, linear
acceleration, health, and attachment metadata through `IMU`. Applications use
that capability contract independently of the physical sensor or transport.

`IMUProcessor` converts a generic `ROS2.stream` carrying
`sensor_msgs/msg/Imu` into the normalized IMU contract. `IMUViewer` is a
managed live diagnostic surface. The editor shows a 3D B-logo robot, body and world XYZ
axes, roll/pitch/yaw, angular velocity, acceleration, source frame, and
freshness. It rejects zero-length quaternions and reports ROS IMU messages whose
orientation covariance marks orientation unavailable.

`IMUViewer` also accepts the fixed sensor mounting as roll, pitch, and yaw in
degrees. These values describe the ROS transform from the sensor frame into the
robot body frame. The viewer applies that extrinsic before drawing the robot
and can display both robot-body axes and the mounted IMU calibration axes.
Keep these values with the physical robot's calibration or profile; obtain them
from its URDF or TF tree rather than assuming the sensor is aligned to the body.

For a real robot, configure the generic `ROS2` node with the robot's IMU topic
(commonly `/imu/data`), `sensor_msgs/msg/Imu`, sensor-data QoS, and `action =
start`; connect `ROS2.stream → IMUProcessor.source`, then connect
`IMUProcessor.stream → IMUViewer.source` and use **Go live**.
