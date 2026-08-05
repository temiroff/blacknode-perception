# blacknode-perception

`blacknode-perception` provides camera, depth, LiDAR, tracking, detection, and visual-reasoning capabilities for Blacknode workflows.

## Components

| Component | Default | Purpose |
|---|---:|---|
| `camera` | On | Local camera discovery, streaming, calibration, and ROS 2 adapters |
| `vlm` | On | Frame prompting, VLM calls, and reasoning dashboards/streams |
| `depth` | On | Provider-neutral metric depth, live ROS 2 streams, and obstacle state |
| `lidar` | On | Provider-neutral LaserScan data and ROS 2/Warp adapters |
| `detection` | On | Managed OpenCV detection streams |
| `tracking` | On | Deterministic color tracking and target hints |
| `imu` | On | `IMU`, live ROS 2 orientation, and `IMUViewer` |
| `slam`, `localization` | Off | Optional capability contracts |

## Main nodes

Every ROS 2 sensor workflow uses the same visible boundary:

`ComputeDevice → ROS2 transport → sensor processor → optional Warp processor → sensor viewer`

`ROS2` owns discovery, QoS, subscription lifecycle, and binary transport. The
sensor processor owns decoding, validation, units, calibration, freshness, and
the stable Blacknode contract. Warp is selected by adding a CUDA processing or
viewer stage. Camera and metric-depth image diagnostics terminate at
`CameraViewer` or `DepthViewer`; 3D data can continue through Warp to a
specialized spatial viewer. The processor contract also works when a workflow
does not use Warp.

- `Camera` discovers, selects, and streams a local camera.
- `CameraCalibration` produces versioned intrinsics and a calibrated stream.
- Generic `ROS2` owns transport for every sensor; `CameraImageProcessor`, `DepthImageProcessor`, `LaserScanProcessor`, and `IMUProcessor` normalize the streams.
- `DepthCamera` and `DepthObstacleWarning` expose capability and safety state after depth processing.
- `CameraViewer`, `DepthViewer`, and `IMUViewer` present processed sensor contracts with controls specific to that sensor.
- `LiDAR` exposes normalized scans; the optional CUDA `LiDARViewer` is the explicit Warp visualization path.
- `TrackingObject` serves annotated MJPEG, masks, snapshots, and latest detections.
- `VLM`, `ReasoningStream`, and `ReasoningDashboard` support OpenAI-compatible, NVIDIA NIM, Anthropic, and local Ollama endpoints.

## Quick start

```powershell
blacknode packages install https://github.com/temiroff/blacknode-perception.git
blacknode packages setup blacknode-perception
```

Add a `Camera` node with `selection: 0` for the first local camera. Sensor templates expose real local or paired-device sources. ROS adapters declare their dependencies on `blacknode-ros2`; Warp LiDAR workflows also require `blacknode-cuda/spatial-processing`.

Managed camera, tracker, reasoning, IMU, and spatial viewers start or update one background service. New frames do not require graph recooks. Worker health and source freshness are reported separately, and stale data is never presented as live.

## Development

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD="1"
python -m pytest packages/blacknode-perception/tests
Get-ChildItem packages\blacknode-perception -Filter *.json -Recurse | Where-Object { $_.FullName -match '[\\/]templates[\\/]' } | ForEach-Object { blacknode validate $_.FullName }
```

Hardware, ROS, GPU, network, and model paths remain optional and report structured unavailable states. See [AGENTS.md](AGENTS.md) for stream and freshness rules.
