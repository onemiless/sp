# Experimental ROAD object detector

`objectd` remains gated behind the Phase A device/resource acceptance criteria. Normal repository builds do not download or compile its model.

To prepare the manifest-pinned model explicitly:

```bash
scons --objectd
```

The manager starts `objectd` only when all of the following are true at the offroad-to-onroad edge:

- car brand is Tesla;
- `VisionObjectDetectionEnabled` is true;
- the compiled artifact and its ONNX SHA-256 provenance stamp are present.

All feature Params default to false. Validation metadata recording is development-only and starts with the detector when `VisionObjectRecordValidation` is true. Its bounded sidecar files are written beside the normal route-log directory under `vision_object_validation` and never contain camera frames.

This feature is display-only. It must not be connected to controls, planners, CAN, radar, or Panda.

Inference runs at 3 Hz in normal mode and 2 Hz under resource pressure. The supervisor pauses the worker if pressure
persists. The result-age, camera-frame and tracker-prediction windows are sized for the 3 Hz normal cadence; degraded
results remain hidden outside the development debug overlay.

When distance display is enabled, objectd uses the live SP road-camera calibration (camera attitude, calibrated height,
camera intrinsics) to intersect an unclipped detection-box ground contact with a locally flat road. Results are relative
to the camera ground point, are limited to 5-30 m with at most 3 m estimated uncertainty, and are rendered with a `~`
prefix in `coarse` mode. They are not front-bumper distances and are invalid whenever calibration or contact geometry is
unavailable. This geometric estimate has no authority outside the display path and still requires measured-distance road
validation.
