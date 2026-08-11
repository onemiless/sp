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
