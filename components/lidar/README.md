# LiDAR

The `lidar` component provides a stable 2D scan contract for live sensor data.

## Nodes

| Node | Purpose |
|---|---|
| `LaserScanProcessor` | Converts a generic ROS 2 `LaserScan` stream into the normalized Blacknode scan contract. |
| `LiDAR` | Validates provider state and freshness, then emits a `blacknode.lidar-capability` plus portable attachment configuration. |

For live visualization, connect
`ComputeDevice → ROS2 → LaserScanProcessor → Viewer` and choose the `/scan`
topic. The CUDA `Viewer` is the explicit Warp path. Workflows that only need a
portable scan contract stop at `LaserScanProcessor` or continue to `LiDAR`.
