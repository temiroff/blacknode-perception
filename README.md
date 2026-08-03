# blacknode-perception

`blacknode-perception` provides camera, depth, LiDAR, tracking, detection, and visual-reasoning capabilities for Blacknode workflows.

## Components

| Component | Default | Purpose |
|---|---:|---|
| `camera` | On | Local camera discovery, streaming, calibration, and ROS 2 adapters |
| `vlm` | On | Frame prompting, VLM calls, and reasoning dashboards/streams |
| `depth` | On | Provider-neutral metric depth, replay/mock providers, and obstacle state |
| `lidar` | On | Provider-neutral LaserScan data, replay/mock providers, and ROS 2/Warp adapters |
| `detection` | On | Managed OpenCV detection streams |
| `tracking` | On | Deterministic color tracking and target hints |
| `imu`, `slam`, `localization` | Off | Optional capability contracts |

## Main nodes

- `Camera` discovers, selects, and streams a local camera.
- `CameraCalibration` produces versioned intrinsics and a calibrated stream.
- `CameraROS2Provider` and `CameraROS2Subscribe` manage ROS 2 camera sources.
- `DepthCamera`, `DepthCameraTestProvider`, and `DepthObstacleWarning` normalize depth behavior.
- `LiDAR`, `LiDARTestProvider`, and `LiDARROS2Scan` normalize and inspect scans.
- `TrackingObject` serves annotated MJPEG, masks, snapshots, and latest detections.
- `VLM`, `ReasoningStream`, and `ReasoningDashboard` support OpenAI-compatible, NVIDIA NIM, Anthropic, and local Ollama endpoints.

## Quick start

```powershell
blacknode packages install https://github.com/temiroff/blacknode-perception.git
blacknode packages setup blacknode-perception
```

Add a `Camera` node with `selection: 0` for the first local camera. Use the included templates for camera console, live reasoning, RGB-D inspection, depth-device selection, static LiDAR, or a ROS 2 LiDAR viewer. ROS adapters declare their dependencies on `blacknode-ros2`; Warp LiDAR workflows also require `blacknode-cuda/spatial-processing`.

Managed camera, tracker, reasoning, and LiDAR viewers start or update one background service. New frames do not require graph recooks. Worker health and source freshness are reported separately, and stale data is never presented as live.

## Development

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD="1"
python -m pytest packages/blacknode-perception/tests
Get-ChildItem packages\blacknode-perception -Filter *.json -Recurse | Where-Object { $_.FullName -match '[\\/]templates[\\/]' } | ForEach-Object { blacknode validate $_.FullName }
```

Hardware, ROS, GPU, network, and model paths remain optional and report structured unavailable states. See [AGENTS.md](AGENTS.md) for stream and freshness rules.
