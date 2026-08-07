# Depth camera

The depth component is the stable application-facing layer for depth cameras.
A workflow connects to `DepthCamera`; a real provider supplies metric frames
from ROS 2 or a device SDK.

```text
ComputeDevice → ROS2 → DepthImageProcessor → DepthCamera
      ├─ preview and health
      ├─ metric depth-stream contract
      ├─ point-cloud contract
      ├─ DepthObstacleWarning
      └─ RobotAttachment configuration
```

## Nodes

| Node | Purpose |
|---|---|
| `DepthCameraDeviceSelect` | Resolves a generic depth candidate from a selected compute-device inspection and gates live start on explicit confirmation |
| `DepthCamera` | Stable provider-neutral capability, freshness, hardware identity, and attachment configuration |
| `DepthObstacleWarning` | Read-only clear/warning/critical assessment over fresh metric depth |
| `DepthImageProcessor` | Converts a generic image-bearing ROS2 stream into metric depth, calibration, preview, and health contracts |

`DepthObstacleWarning.safe_to_proceed` is false when data is stale, missing, or
insufficient. The node reports an assessment and never commands motion.

## Real device template

Open **Depth Camera on Device** for the complete physical path:

```text
ComputeDevice → DeviceInspect → DepthCameraDeviceSelect
  → ROS2 → DepthImageProcessor → DepthCamera
  → preview + obstacle assessment + RobotAttachment/Profile
```

Choose the registered computer on `ComputeDevice` and press **Run once**.
`DepthCameraDeviceSelect` reads the current generic ROS 2 capability candidate
and fills the raw depth and camera-info topics. It defaults to `confirm=false`,
so the first run remains read-only and keeps the stream stopped.

Review the selected topic, `depth_scale`, frame, physical camera serial, and
mount transform. Set `hardware_id` to the camera's stable serial and set
`confirm=true`. Use **Go live** when the editor computer can see the same ROS 2
graph directly. To execute inside the selected Jetson or robot computer, pair
Blacknode Runtime, open **Deployments**, select the attached robot, choose
**Check setup**, then **Send & run on robot**.

The compute-device node carries stable selection and current read-only ROS
state from the paired Runtime. Generic `ROS2` supplies the managed live frame
path; `DepthImageProcessor` performs depth-specific normalization.

## Provider contract

A compatible provider emits:

- `depth_stream.kind = blacknode.depth-stream`;
- `depth_stream.encoding` and `depth_stream.depth_scale`;
- `depth_stream.frame_source`, a compact inline replay or binary live-frame
  handle that keeps dense pixel arrays out of workflow JSON;
- `depth_stream.calibration`, including pinhole dimensions and `fx`, `fy`,
  `cx`, and `cy` for metric projection;
- `depth_stream.camera_info_source` when calibration comes from a managed ROS 2
  CameraInfo stream, allowing live consumers to resolve a late first sample;
- `health.source_fresh`, `health.state`, and `health.summary_m`;
- stable frame and physical hardware identity;
- an optional `blacknode.point-cloud-stream`;
- an optional preview image.

`summary_m` contains bounded distance statistics in metres:
`minimum`, `p05`, `median`, `p95`, `valid_count`, and `total_count`.
Safety features use `p05` as a noise-resistant near-distance measurement.
The display preview is visualization data and is not used for measurement.

`DepthViewer` keeps display choices separate from metric processing. Leave
`auto_range=true` for per-frame percentile contrast, or set it to false and
choose stable `near_m` and `far_m` limits. The grayscale palette draws nearer
valid samples brighter and farther samples darker. `turbo` provides a color
alternative, and `invalid_color=magenta` distinguishes missing measurements
from valid far-depth pixels. The editor presents these controls directly below
the live depth image and renders them locally from the original binary frame.
Changing them redraws the canvas immediately and persists the viewer setting
without recooking the graph. Click the live depth image to mark a source pixel
and inspect its current distance in metres; the selected pixel continues
updating as new frames arrive. Metric summaries, obstacle assessment, and
downstream projection remain unchanged.

To add another provider:

1. Put its SDK and device-specific code in an adapter or driver extension.
2. Keep imports optional so the depth component still loads when the SDK is
   absent.
3. Emit the contract above and a structured unavailable health state when the
   provider cannot run.
4. Connect it to `DepthCamera`.
5. Run the same mock, replay, freshness, and obstacle contract tests.
6. Save calibration and extrinsics against stable physical hardware identity.
