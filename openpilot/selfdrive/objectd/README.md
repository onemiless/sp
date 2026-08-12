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

The detector follows the camera displayed by the onroad UI: WIDE below 10 m/s and ROAD above 15 m/s in experimental
mode, with the same hysteresis as the UI. A stream change restarts the single worker and clears its tracker so boxes from
one camera are never projected onto the other. The status badge reports the brief switching interval.

When distance display is enabled, objectd uses the live SP calibration, the selected ROAD/WIDE camera intrinsics and the
WIDE camera-to-device orientation to intersect an unclipped detection-box ground contact with a locally flat road. Raw
computed results from 5-100 m are displayed to one decimal place. Estimated uncertainty remains recorded but does not hide
the displayed result. Values are relative to the camera ground point, are not front-bumper distances, and remain invalid
when calibration or contact geometry is unavailable. This geometric estimate has no authority outside the display path and
still requires measured-distance road validation, especially for long-range WIDE results.

SP's four model lane lines classify each ranged object as ego-lane, adjacent-lane, boundary overlap or unknown. The normal
UI suppresses only ego-lane car, bus and truck boxes; person, bicycle and motorcycle boxes remain visible. Uncertain lane
geometry fails open, and objectd continues publishing every detection regardless of UI filtering.
