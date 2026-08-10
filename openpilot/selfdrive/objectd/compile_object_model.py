#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def validate_graph(onnx_path: Path, manifest: dict) -> None:
  from tinygrad.nn.onnx import OnnxRunner

  runner = OnnxRunner(str(onnx_path))
  expected_input = manifest["input"]
  if set(runner.graph_inputs) != {expected_input["name"]}:
    raise RuntimeError(f"unexpected ONNX inputs: {tuple(runner.graph_inputs)}")
  input_spec = runner.graph_inputs[expected_input["name"]]
  if tuple(input_spec.shape) != tuple(expected_input["shape"]):
    raise RuntimeError(f"input shape mismatch: {input_spec.shape}")
  expected_output = manifest["output"]
  if tuple(runner.graph_outputs) != (expected_output["name"],):
    raise RuntimeError(f"expected exactly one output, got {runner.graph_outputs}")
  output_spec = runner.graph_outputs[expected_output["name"]]
  if tuple(output_spec.shape) != tuple(expected_output["shape"]):
    raise RuntimeError(f"output shape mismatch: {output_spec.shape}")


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--onnx", required=True, type=Path)
  parser.add_argument("--manifest", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--compile3", required=True, type=Path)
  args = parser.parse_args()

  manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
  expected_hash = manifest["model"]["onnxSha256"].lower()
  actual_hash = sha256_file(args.onnx)
  if actual_hash != expected_hash:
    raise RuntimeError(f"ONNX SHA-256 mismatch: expected {expected_hash}, got {actual_hash}")
  validate_graph(args.onnx, manifest)

  required_flags = ("DEV", "IMAGE", "FLOAT16", "NOLOCALS", "JIT_BATCH_SIZE", "OPENPILOT_HACKS")
  print("object detector compile flags: " + " ".join(f"{key}={os.getenv(key, '<unset>')}" for key in required_flags))
  subprocess.run([sys.executable, str(args.compile3), str(args.onnx), str(args.output)], check=True)


if __name__ == "__main__":
  main()
