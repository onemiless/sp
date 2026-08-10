# Object detector candidate licenses

Audit date: 2026-08-11.

## Selected implementation candidate

- Model: YOLOX-Nano release `0.1.1rc0`
- Repository: <https://github.com/Megvii-BaseDetection/YOLOX>
- Frozen tag commit: `e1052df71842031413f6030723c3607b839c80ce`
- Upstream license: Apache-2.0
- License text: <https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE>
- Weight: <https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx>

The model is fetched only from the frozen URL and accepted only when its SHA-256 matches the manifest. This repository does not silently substitute another weight.

## Comparison candidate

- Model family: NanoDet-Plus
- Repository: <https://github.com/RangiLyu/nanodet>
- Frozen comparison tag: `v1.0.0`
- Frozen tag commit: `d3fb34fa91d6020f273d6d063bf324dcd97bac12`
- Upstream license: Apache-2.0
- License text: <https://github.com/RangiLyu/nanodet/blob/main/LICENSE>

NanoDet-Plus is recorded as the same-class comparison candidate but is not deployed in this MVP. Accuracy, QCOM compatibility, resource impact, and redistribution of any particular pretrained weight remain unverified.

License compatibility here is an engineering inventory, not legal advice. Release packaging must include the applicable upstream notices and be reviewed before distribution.
