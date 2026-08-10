# Vision object detection Phase D conservative-distance report

Date: 2026-08-11 (Asia/Shanghai)

## Outcome

Phase D did not pass. The source includes isolated ray/ground geometry, a conservative distance gate and a road-plane fit primitive, but objectd intentionally publishes all distance fields invalid. No old distance is cached or reused.

## Changed files

- `openpilot/selfdrive/objectd/geometry.py`
- `openpilot/selfdrive/objectd/road_plane.py`
- `openpilot/selfdrive/objectd/objectd.py`
- `openpilot/selfdrive/objectd/tests/test_core.py`

The gate rejects missing vehicle extrinsics, invalid calibration/contact/road plane, acceleration magnitude at or above 0.5 m/s², less than one second of stable motion, values outside 5-30 m, uncertainty above 3 m and results older than 300 ms. The road-plane primitive requires at least two consistent curves and rejects slopes above 2%.

## Tests and results

Synthetic unit tests passed for camera-road to front-bumper translation, valid local ground intersection, parallel/upward-ray rejection, all-invalid default behavior, a gated 10 m example, two-curve plane fitting and steep/single-curve rejection.

## Unverified and C3X evidence

- Vehicle model, measured camera-ground to front-bumper horizontal extrinsics and independent physical camera height are missing.
- liveCalibration/livePose/cameraOdometry/aEgo/brake/throttle and frame-matched modelV2 road data are not connected to distance output.
- No 5/10/15/20/30 m truth set, eligible coverage, error distribution or known-invalid false-release measurement exists.
- C3X evidence: none.

Validation layers: isolated geometry unit checks passed; distance integration/build not passed; replay not run; device runtime not run; road validation not run. This is a detection-box prototype with distance schema reserved and forced invalid, not a validated detection-plus-distance MVP.
