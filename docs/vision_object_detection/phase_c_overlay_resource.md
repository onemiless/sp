# Vision object detection Phase C overlay and resource report

Date: 2026-08-11 (Asia/Shanghai)

## Outcome

ROAD-only debug/formal overlay source and resource failover logic are implemented. Phase C acceptance is not passed because dynamic replay, screenshot diff and C3X resource evidence do not exist.

## Changed files

- `openpilot/selfdrive/ui/onroad/vision_object_renderer.py`
- `openpilot/selfdrive/ui/onroad/cameraview.py`
- `openpilot/selfdrive/ui/onroad/augmented_road_view.py`
- `openpilot/selfdrive/ui/sunnypilot/ui_state.py`
- `openpilot/selfdrive/objectd/ui_contract.py`
- `openpilot/selfdrive/objectd/resource.py`
- `openpilot/selfdrive/objectd/tools/validation_recorder.py`
- corresponding objectd unit tests

The renderer uses the displayed frame ID/timestamp and CameraView's centered crop transform. It accepts positive finite crop scales above one, rejects WIDE immediately, rejects service/message invalidity, frames in the future, more than six frames of delta, age over 300 ms, and normal overlays below 4 Hz. Debug mode may show explicitly degraded 2 Hz output with age; normal overlay cannot. The metadata-only recorder is manager-controlled by `VisionObjectRecordValidation`, has 50 MiB per-file and 500 MiB directory caps, and never records camera frames.

## Tests and results

Unit tests passed for 1928x1208 ROAD letterbox mapping, the real 2160x1080/AR0231 1.1x C3X crop transform, invalid boxes, ROAD/WIDE gating, 300 ms/frame-delta gating and degraded debug-only behavior. Ruff, compileall and whitespace checks passed.

## Unverified and C3X evidence

- No UI screenshot diff, rotation/border replay, CameraOffset replay, dynamic box-vs-current-frame IoU measurement or multiple ROAD/WIDE switch replay was run.
- No 30-minute modeld-parallel resource run, latency/drop comparison, PSS, temperature, power or UI frame-time result exists.
- C3X evidence: none.

Validation layers: source/static/unit checks passed; full UI build not passed; replay not passed; device runtime not run; road validation not run. Therefore the formal overlay is implemented but not accepted for operational use.
