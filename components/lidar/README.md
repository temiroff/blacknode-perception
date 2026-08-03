# LiDAR

The `lidar` component provides a stable 2D scan contract, mock data, and ROS 2
capture.

## Nodes

| Node | Purpose |
|---|---|
| `LiDARTestProvider` | Generates a deterministic rectangular-room scan or replays a recorded normalized scan. |
| `LiDAR` | Validates provider state and freshness, then emits a `blacknode.lidar-capability` plus portable attachment configuration. |
| `LiDARROS2Scan` | Captures one `sensor_msgs/msg/LaserScan` from a configured ROS 2 topic and normalizes it. |

For live visualization, connect `ComputeDevice → ROS2 → Viewer` and choose the
`/scan` topic. `Viewer.mode=editor` embeds the point cloud; `mode=device` opens
the native window where the deployed graph runs.

The older specialized ROS 2 viewer remains loadable for saved workflows but is
hidden from new graphs.
