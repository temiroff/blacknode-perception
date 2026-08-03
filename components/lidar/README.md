# LiDAR

The `lidar` component gives Blacknode workflows a stable 2D scan contract and
a hardware-free path for developing spatial processing before connecting a
robot.

The deterministic provider accepts up to five million samples for GPU stress
tests. Dense visual workflows should process the complete scan while using a
display stride so the debug renderer does not turn every ray into expensive
line geometry.

## Nodes

| Node | Purpose |
|---|---|
| `LiDARTestProvider` | Generates a deterministic rectangular-room scan or replays a recorded normalized scan. |
| `LiDAR` | Validates provider state and freshness, then emits a `blacknode.lidar-capability` plus portable attachment configuration. |
| `LiDARROS2Scan` | Captures one `sensor_msgs/msg/LaserScan` from a configured ROS 2 topic and normalizes it. |
| `LiDARROS2WarpViewer` | Starts or stops a managed native ROS 2 subscription rendered by NVIDIA Warp's OpenGL viewer. |

The real adapter is read-only. It subscribes to `/scan` by default and leaves
the physical LiDAR driver's lifecycle with the robot bringup stack.

## First visual experiment

1. Load **LiDAR Warp Static Lab** and run it with `device=cpu` to verify the
   scan contract and processing data on any development machine.
2. Set the static viewer's `action` to `start` and select `cuda:0` to open the
   interactive GPU view.
3. On the robot computer, confirm `/scan` publishes
   `sensor_msgs/msg/LaserScan`.
4. Load **ROS 2 LiDAR Warp Live Viewer**, run `LiDARROS2Scan` once, then set
   `LiDARROS2WarpViewer.action=start`.

The window displays a red robot origin, coordinate axes, gray raw points, and
cyan distance-colored Warp-filtered points. With animation enabled, recent rays
are blue and the active beam and hit are yellow while the scan accumulates.
Press **Space** to cycle raw, filtered, and combined views, **P** to pause or
resume the sweep, **R** to restart it, and **Escape** to close the window.

Worker-process liveness and source-scan freshness are separate states. The
one-shot capture reports scan age and valid-point counts; the managed viewer
prints its subscription state and exits explicitly through the node's `stop`
action or the window close control.

## Current milestone

This component implements normalized `LaserScan` capture and visualization.
Odometry accumulation, ICP, voxel maps, TSDF surfaces, normal estimation, and
loop closure are later mapping stages and are not reported by these nodes.
