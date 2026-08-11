# Vision object detection Phase B implementation report

Date: 2026-08-11 (Asia/Shanghai)

## Outcome

Phase B source implementation is present, with fail-closed service behavior and no control-chain integration. It is not a C3X build or runtime pass.

## Changed files

- Schema/service: `openpilot/cereal/custom.capnp`, `log.capnp`, `services.py`.
- Build/model: root `SConstruct`, `openpilot/selfdrive/objectd/SConscript`, `compile_object_model.py`, `model_runner.py`.
- Detection: `preprocess.py`, `postprocess.py`, `tracker.py`, `worker.py`, `objectd.py`, `constants.py`.
- Lifecycle/resource: `resource.py`, `supervisor.py`, `vision_object_latch.py`, `process_config.py`, Params registration.
- Tests: `openpilot/selfdrive/objectd/tests/*`, `openpilot/system/manager/tests/test_vision_object_latch.py`.

`VisionObjectStateSP` reuses reserved10 ID `0xcb9fd56c7057593a`; `visionObjectStateSP` keeps union ordinal `@136`; service frequency is fixed at 5 Hz. Objects are capped at 32. Bounding-box, velocity and position-uncertainty components use named schema fields. The worker connects directly to ROAD VisionIPC and only returns bounded metadata through a spawned-process queue.

The manager latch reads Tesla brand, enable parameter and compiled-artifact provenance only at the offroad-to-onroad edge. Worker retry is 1/2/4/8/30 seconds with a five-failures-in-ten-minutes session fuse. ROAD VisionIPC connection has a five-second deadline and enters the same retry path on failure. modeld pressure degrades objectd from 5 Hz to 2 Hz, then pauses it; modeld loss, overheat or 90% memory pauses immediately.

## Tests and results

```text
python -m unittest -v openpilot.selfdrive.objectd.tests.test_core ... test_vision_object_latch
ruff check <changed Python paths>
python -m compileall -q <changed Python paths>
compile(SConscript source, ..., "exec")
git diff --check
```

Results after review corrections: 34 pure-Python unit tests passed; Ruff passed; Python and SConscript syntax passed; Cap'n Proto C++ generation passed; diff whitespace check passed. Tests cover preprocessing, raw-head grid/stride decode, NMS/class mapping, tracker reset/velocity, named schema fields, schema ID/ordinal/service rate, retry/fuse/connect deadline, resource degradation/pause, artifact provenance, session latch behavior, parameter defaults, real C3X crop mapping and forbidden control-chain imports.

The host's repository-level pytest bootstrap requires the unbuilt native `params_pyx`, so the tests were run directly with `unittest`. A pycapnp round-trip succeeded earlier in this implementation session; a later Windows pycapnp repeat crashed in its native parser, so this report does not elevate that to a reproducible cereal toolchain pass.

## Unverified and C3X evidence

- Full SCons build, generated cereal bindings, messaging integration tests, QCOM artifact generation, clean rebuild and device cold load were not run.
- VisionIPC recovery, a real model-load failure, driver-level QCOM hang and actual manager process execution were not exercised.
- C3X evidence: none.

Validation layers: source/static/unit checks passed; full build not passed; replay not run; device runtime not run; road validation not run.
