# Vision object detection Phase A continuation report

Date: 2026-08-11 (Asia/Shanghai)

## Outcome

The prior Phase A decision remains **NO-GO** because no C3X/QCOM, fixed labeled dataset, modeld-parallel, thermal, memory, or power evidence exists. On explicit user instruction, implementation continued beyond that gate. This continuation does not convert the gate to a pass.

## Added artifacts

- `openpilot/selfdrive/objectd/models/object_detector_manifest.json`: frozen YOLOX-Nano release URL, upstream commit, SHA-256, tensor contract, preprocessing, class mapping and thresholds.
- `openpilot/selfdrive/objectd/models/MODEL_LICENSES.md`: YOLOX-Nano and NanoDet-Plus license inventory.
- `openpilot/selfdrive/objectd/fetch_object_model.py`: atomic download with mandatory SHA-256 verification.
- `openpilot/selfdrive/objectd/validation/README.md` and `dataset_manifest.example.json`: fixed-dataset contract and example only.

The ONNX file is deliberately fetched by the build instead of committed. This is a documented deviation from the plan: this fork configures all ONNX files to an external GitLab LFS push endpoint for which this GitHub repository has no upload authority. The frozen URL and hash retain reproducibility without pretending that an inaccessible LFS object was published.

## Tests and results

Commands:

```text
fetch_object_model.py --manifest ... --output <temporary path>
SHA256(object_detector.onnx)
tinygrad OnnxRunner graph inspection
ONNX Runtime CPUExecutionProvider synthetic-input inference and objectd decode
```

Results:

- official YOLOX-Nano ONNX SHA-256 matched `c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d`;
- graph contract matched one `images` float32 input `[1,3,416,416]` and one `output` `[1,3549,85]`;
- a synthetic 1928x1208 black frame completed desktop ONNX Runtime inference, produced finite output, and passed the decoder; one measured desktop run was 7.45 ms;
- tinygrad parsed the graph, but desktop tinygrad execution/compile could not complete because the required LLVM/clang renderer toolchain was absent.

## Post-review corrections

- The frozen release ONNX exposes the raw YOLOX head. `objectd` now applies the upstream grid/stride and exponential width/height decode before NMS. A deterministic differential check matches the frozen upstream `demo_postprocess` implementation.
- Normal builds no longer fetch the detector. `scons --objectd` is required to create the compiled artifact and its ONNX SHA-256 provenance stamp.
- The detector defaults to disabled and the manager refuses to start it when the compiled artifact/stamp is absent or stale.

## Unverified and C3X evidence

- The committed dataset file is only an example; precision, recall, threshold selection, and distance accuracy are unverified.
- NanoDet-Plus is license-inventoried only; its exact weight, ONNX conversion and QCOM behavior are unverified.
- QCOM compile/load/inference, clean larch64 rebuild, modeld coexistence, 30-minute run and all device resource thresholds were not run.
- C3X evidence: none.

Validation layers: source/model-contract checks partially passed; full build not passed; replay not run; device runtime not run; road validation not run.
