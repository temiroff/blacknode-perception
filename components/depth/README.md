# Depth camera

The depth component is the stable application-facing layer for depth cameras.
A workflow connects to `DepthCamera`; a provider supplies metric frames from
ROS 2, a device SDK, a recording, or the deterministic test provider.

```text
provider or replay
  → DepthCamera
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
| `DepthCameraTestProvider` | Deterministic mock depth and recorded-contract replay |
| `DepthCamera` | Stable provider-neutral capability, freshness, hardware identity, and attachment configuration |
| `DepthObstacleWarning` | Read-only clear/warning/critical assessment over fresh metric depth |
| `DepthROS2Subscribe` | Managed ROS 2 adapter with live preview, raw depth statistics, and metric normalization |

`DepthObstacleWarning.safe_to_proceed` is false when data is stale, missing, or
insufficient. The node reports an assessment and never commands motion.

## Component Lab

Open **Depth Camera Component Lab** in the template gallery. Its default mock
provider runs on a development computer and produces:

- a deterministic depth preview;
- a fresh `blacknode.depth-camera-capability`;
- a depth obstacle assessment;
- a generic `depth_camera` robot attachment;
- a capability-only robot profile.

Change `distance_m` to test clear, warning, and critical behavior. The template
produces an unsaved profile artifact. Add and run `RobotProfileSave` only after
reviewing the attachment ID, stable hardware identity, frame, mount transform,
and ROS 2 topics.

For a physical camera, replace `DepthCameraTestProvider` with
`DepthROS2Subscribe` and connect `preview`, `depth_stream`,
`point_cloud_stream`, and `health` to the matching `DepthCamera` inputs.
The feature and attachment side of the graph stays unchanged.

## Real device template

Open **Depth Camera on Device** for the complete physical path:

```text
ComputeDevice → DeviceInspect → DepthCameraDeviceSelect
  → DepthROS2Subscribe → DepthCamera
  → preview + obstacle assessment + RobotAttachment/Profile
```

Choose the registered computer on `ComputeDevice` and press **Run once**.
`DepthCameraDeviceSelect` reads the saved generic ROS 2 capability candidate
and fills the raw depth and camera-info topics. It defaults to `confirm=false`,
so the first run remains read-only and keeps the stream stopped.

Review the selected topic, `depth_scale`, frame, physical camera serial, and
mount transform. Set `hardware_id` to the camera's stable serial and set
`confirm=true`. Use **Go live** when the editor computer can see the same ROS 2
graph directly. To execute inside the selected Jetson or robot computer, pair
Blacknode Runtime, open **Deployments**, select the attached robot, choose
**Check setup**, then **Send & run on robot**.

The compute-device node carries selection and inspection data. It does not
proxy live ROS 2 frames through an inspection-only SSH connection. A paired
Runtime or direct DDS visibility supplies the live data path.

## Provider contract

A compatible provider emits:

- `depth_stream.kind = blacknode.depth-stream`;
- `depth_stream.encoding` and `depth_stream.depth_scale`;
- `health.source_fresh`, `health.state`, and `health.summary_m`;
- stable frame and physical hardware identity;
- an optional `blacknode.point-cloud-stream`;
- an optional preview image.

`summary_m` contains bounded distance statistics in metres:
`minimum`, `p05`, `median`, `p95`, `valid_count`, and `total_count`.
Safety features use `p05` as a noise-resistant near-distance measurement.
The display preview is visualization data and is not used for measurement.

To add another provider:

1. Put its SDK and device-specific code in an adapter or driver extension.
2. Keep imports optional so the depth component still loads when the SDK is
   absent.
3. Emit the contract above and a structured unavailable health state when the
   provider cannot run.
4. Connect it to `DepthCamera`.
5. Run the same mock, replay, freshness, and obstacle contract tests.
6. Save calibration and extrinsics against stable physical hardware identity.
