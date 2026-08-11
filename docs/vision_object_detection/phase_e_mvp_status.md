# Vision object detection Phase E UI and MVP status

Date: 2026-08-11 (Asia/Shanghai)

## Outcome: implementation continued, MVP not complete

Tesla/C3X settings and the normal ROAD overlay source are implemented, but Phase E acceptance and the document's MVP definition are not met. The UI must be treated as experimental pending C3X reports.

## Changed files

- `openpilot/common/params_keys.h`
- `openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/tesla.py`
- `openpilot/selfdrive/ui/sunnypilot/ui_state.py`
- onroad renderer/CameraView/AugmentedRoadView files listed in Phase C

Settings are visible in the Tesla panel without a device-type gate; the detector main switch is only editable offroad and when a manifest-matching compiled artifact exists; the distance-display switch is available but the Phase D producer still fails closed with all distances invalid; debug is development-only. All detector, overlay, distance-display and debug Params default to **false**. The model is prepared only by the explicit `scons --objectd` target.

No setting or message is connected to `controlsd`, `radard`, planners, `carcontroller`, CAN output or Panda. A static forbidden-path search found no such imports in the new feature paths. Submodules remain at their recorded commits and have no source changes.

## Tests and results

The 34 unit tests, Ruff, compileall, SConscript syntax, Cap'n Proto C++ generation and whitespace checks passed. These are source/unit/schema checks only.

## Required validation still missing

| Layer | Status |
| --- | --- |
| Full repository/C3X build | Not passed |
| Replay and screenshot diff | Not passed |
| C3X device runtime and 30-minute bench | Not run |
| Two-hour shadow road run | Not run |
| Detection accuracy holdout | Not run |
| Metric-distance acceptance | Not run; output forced invalid |
| modeld/UI/memory/thermal/power gates | Not run |

C3X evidence: none. The code and reports may be sent to the target device for the next validation cycle, but none of the missing layers may be inferred from desktop tests.
