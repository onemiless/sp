# Vision object detection Phase A Go/No-Go report

Date: 2026-08-11 (Asia/Shanghai)

Baseline: `origin/dev-new` at `993a330f5983408f8da93b023ccc6ddf81cf7ab5`

## Decision: NO-GO

Phase A did not meet the mandatory model, data, C3X/QCOM, and resource gates. Per the implementation plan, work stops before Phase B. No reserved schema, service, objectd runtime, UI, distance estimator, manager lifecycle, control path, CAN path, radar path, Panda source, or submodule source is modified.

This is a gate result, not an implementation failure hidden by a desktop substitute. No temporary model, desktop inference, or synthetic UI output is presented as C3X evidence.

## Gate evidence

| Phase A requirement | Evidence | Result |
| --- | --- | --- |
| Versioned fixed validation dataset and annotation contract | No detector validation dataset, manifest, or frozen tuning/holdout split exists in the repository. | Fail |
| Two candidate models with distribution-compatible licenses | YOLOX upstream reports Apache-2.0. The required second candidate is not specified, and no detector license file is frozen in this repository. | Incomplete |
| Frozen ONNX, preprocessing, output and class mapping | No `object_detector.onnx` or `object_detector_manifest.json` exists. tinygrad example YOLO files are examples, not a frozen production detector artifact. | Fail |
| Development-host CPU correctness | Cannot be meaningfully run without the frozen model, manifest, preprocessing contract, and fixed labeled data. No desktop result is substituted. | Not run |
| C3X/larch64 QCOM compile, load and single-process performance | Audit host is x64 Windows, not C3X/tizi; no C3X connection, QCOM runtime, `adb`, or larch64 build environment was available. | Fail / not run |
| 30-minute concurrent run with modeld | `modeld`, `camerad`, and device UI are not running on this host; no C3X device was available. | Fail / not run |
| modeld latency/drop, memory, thermal and power thresholds | Required device metrics cannot be collected on this desktop. | Fail / not run |

YOLOX license evidence was read from the official upstream repository and GitHub repository metadata:

- repository: `https://github.com/Megvii-BaseDetection/YOLOX`
- license: `https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE`
- reported SPDX identifier at audit time: `Apache-2.0`

This partial license result does not satisfy the two-candidate comparison or the repository-local license/weight provenance requirement.

## Audit commands and results

```text
rg --files | rg -i '(^|/)(objectd|.*yolo.*|.*object_detector.*|.*vision_object.*|.*validation.*dataset.*)'
```

Result: only generic tinygrad YOLO examples were found; no production object detector package or fixed detector dataset was found.

```text
rg --files -g '*.onnx' -g '*.tflite' -g '*.pt' -g '*.pth' -g '*.engine' -g '*.pkl'
```

Result: existing openpilot driving/driver-monitoring models and tinygrad profiling fixtures were found. No object detector ONNX/manifest was found.

```text
Get-Command adb,scons,python,python3
Get-Process -Name modeld,camerad,ui
```

Result: required commands were not available on `PATH`; device processes were not running.

```text
Invoke-RestMethod https://api.github.com/repos/Megvii-BaseDetection/YOLOX
Invoke-RestMethod https://api.github.com/repos/Megvii-BaseDetection/YOLOX/license
```

Result: official upstream repository metadata reported Apache License 2.0 at `LICENSE`.

## Stop conditions triggered

The plan explicitly requires stopping before UI integration when QCOM compilation, loading, continuous inference, or the modeld resource gate is not proven. Here those gates were not merely below threshold; they could not be executed because the required C3X environment and frozen model/data inputs were absent.

Phase B through Phase E are therefore intentionally not started. In particular:

- `CustomReserved10` remains at ID `0xcb9fd56c7057593a` and union ordinal `@136` remains unchanged.
- ROAD/WIDE rendering logic is not touched.
- No distance value is produced or cached.
- No objectd frequency/resource policy is claimed as device-tested.
- No integration is made with `controlsd`, `radard`, planner code, `carcontroller`, CAN, or Panda.
- No parameter default is changed. The request to use `VisionObjectDetectionEnabled=true` would take precedence over the plan's `false` default only if a later implementation is authorized by a successful Phase A gate.

## Validation-layer status

These statuses are deliberately separated:

| Validation layer | Status |
| --- | --- |
| Source/build checks | Not passed; no detector artifact or C3X build was attempted |
| Replay validation | Not run |
| C3X device runtime validation | Not run; no device evidence available |
| Road validation | Not run |

## Evidence required before re-running Phase A

- a versioned dataset manifest, annotation schema, tuning/holdout split, hashes, and sufficient labeled samples;
- two named candidate models, exact upstream commits/weights, redistribution licenses, and local license files;
- reproducible ONNX export plus frozen preprocessing, class mapping, decoder, NMS and thresholds;
- access to the target C3X/larch64 QCOM build and runtime environment;
- the target Tesla platform and measured camera/front-bumper geometry with uncertainty;
- a repeatable objectd-off baseline and 30-minute modeld-parallel device run capturing latency, drops, memory, temperature and power.

## Phase report

- Changed files: this Phase A report and the Phase 0 baseline report.
- Tests/commands: repository asset search, tool/device-process audit, schema baseline inspection, and official YOLOX license metadata check.
- Result: Phase A No-Go; later phases stopped as required.
- Unverified: detector correctness, C3X QCOM build/load, performance, modeld regression, UI/replay, geometry, distance, road behavior.
- C3X evidence: none available; no C3X, build, replay, device-runtime, or road-pass claim is made.
