# Vision object detection Phase 0 baseline

Date: 2026-08-11 (Asia/Shanghai)

## Outcome

Phase 0 repository synchronization and host audit are complete. The implementation baseline is the latest `origin/dev-new` commit `993a330f5983408f8da93b023ccc6ddf81cf7ab5`. The feature branch `new-dev-yolo` was created directly from that commit.

No user changes or untracked files were present before the branch was created. No schema, process, UI, control, CAN, radar, Panda, or submodule source was changed during this phase.

## Repository baseline

Commands:

```text
git fetch origin dev-new --recurse-submodules=on-demand --prune
git rev-parse dev-new
git rev-parse origin/dev-new
git rev-list --left-right --count dev-new...origin/dev-new
git status --short --branch
git submodule status --recursive
```

Results:

```text
local dev-new:  993a330f5983408f8da93b023ccc6ddf81cf7ab5
origin/dev-new: 993a330f5983408f8da93b023ccc6ddf81cf7ab5
ahead/behind:   0/0
baseline worktree: clean
```

Submodules were initialized and matched the commits recorded by the superproject:

| Path | Commit | Recorded ref description |
| --- | --- | --- |
| `msgq_repo` | `bb6cc57ef4d9a3f9952d9ca84bfa580575e7e2a0` | `heads/master` |
| `opendbc_repo` | `85a463402b4db53aeca79dcca1bc754286adfbba` | `remotes/origin/dev-new` |
| `openpilot/sunnypilot/neural_network_data` | `03cac2d30e111e0689c0429cb8c1fe6cb5a905af` | `heads/master` |
| `panda` | `0c3d67194f469389ad9aee34c288b99235ffb17e` | `remotes/origin/dev-new` |
| `rednose_repo` | `9e19086c26ca35708870d50ebcf237d65d0b163e` | detached recorded commit |
| `teleoprtc_repo` | `c0f813f1c4f7e2d29bc0ed6a4d2a7b3268511b0f` | detached recorded commit |
| `tinygrad_repo` | `ac1632ab966c77ba96a7048b893a30f1a714dc87` | detached recorded commit |

## Host and device baseline

Commands:

```text
Get-CimInstance Win32_OperatingSystem
Get-CimInstance Win32_ComputerSystem
Get-Command adb,scons,python,python3
Get-Process -Name modeld,camerad,ui
```

Observed host:

| Item | Result |
| --- | --- |
| Host | Dell OptiPlex Tower Plus 7020 |
| OS | Windows 11 IoT Enterprise LTSC, 64-bit, version 10.0.26100 |
| Architecture | x64 |
| Physical memory | 68,391,063,552 bytes |
| Free physical memory at audit | 40,362,536 KiB |
| `adb` | not available on `PATH` |
| `scons` | not available on `PATH` |
| `python` / `python3` | not available on `PATH` |
| C3X/tizi connection | none discoverable from this host |
| `modeld`, `camerad`, UI processes | not running |

This host is not a C3X/tizi device. Device temperature, C3X free-memory baseline, modelV2 latency, `frameDropPerc`, UI frame time, power, and thermal steady state therefore could not be measured. Desktop host measurements must not be substituted for those device measurements.

## Architecture and schema baseline

The reserved schema slots are unchanged at the baseline:

```text
openpilot/cereal/custom.capnp:
  CustomReserved10 @0xcb9fd56c7057593a

openpilot/cereal/log.capnp:
  customReserved10 @136 :Custom.CustomReserved10
```

No `objectd`, production object detector ONNX, detector manifest, or vision-object schema exists at the baseline. Repository references to `radarState` and Tesla `DAS_object` belong to existing unrelated control/UI/debug paths. This feature has not been based on either source, and Phase A does not modify them.

## Vehicle geometry evidence

The following required Phase 0 evidence is unavailable:

- exact supported Tesla model/platform;
- C3X camera installation location;
- measured `tFrontInCameraRoad = [xForward, yLeft, 0]` from camera-ground road origin to front-bumper ground center;
- independently measured physical camera height and measurement uncertainty.

Consequently, formal metric distance and corridor output are not implementable from the current evidence. Any future code must keep those outputs invalid until trusted, vehicle-specific measurements are recorded and validated.

## Phase report

- Changed files: this Phase 0 report only.
- Tests/commands: repository fetch/status/submodule checks, host inventory, schema slot inspection, and source-path audit listed above.
- Result: repository baseline confirmed; device and vehicle baseline incomplete.
- Unverified: C3X hardware metrics, Tesla model and camera geometry, modeld/UI resource baselines.
- C3X evidence: none available; no device-connected claim is made.
