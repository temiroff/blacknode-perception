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
node-types = ["DepthROS2Subscribe"]
```

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.

`DepthROS2Subscribe` preserves the metric ROS 2 depth topic and exposes a
normalized `blacknode.depth-stream` handle. Its MJPEG output is a visualization
of `16UC1`, `mono16`, or `32FC1` values for the editor; downstream spatial
workflows consume the original metric interface.
