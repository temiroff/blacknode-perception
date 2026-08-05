# Adapters

Transport adapters for the `depth` component of `blacknode-perception`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

The ROS 2 adapter is declared in `blacknode-package.toml`:

```toml
[components.depth.adapters.ros2]
description = "Metric depth-image and point-cloud capability over ROS 2."
default = false
capabilities = ["adapter.perception.depth.ros2"]
nodes = ["components/depth/adapters/ros2/nodes"]
node-types = []
```

The adapter templates use generic `ROS2` transport followed by
`DepthImageProcessor`. The MJPEG output visualizes `16UC1`, `mono16`, or
`32FC1` values; downstream spatial workflows consume the original metric frame
handle. `DepthViewer` can request automatic or fixed metric display ranges,
grayscale or turbo coloring, and black or magenta invalid pixels from that
preview endpoint without modifying the transported depth frame.
