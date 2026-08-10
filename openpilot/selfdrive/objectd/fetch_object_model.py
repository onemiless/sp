#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request


MODELS_DIR = Path(__file__).resolve().parent / "models"
MANIFEST_PATH = MODELS_DIR / "object_detector_manifest.json"


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def fetch_model(output: Path, manifest_path: Path = MANIFEST_PATH) -> Path:
  manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
  model = manifest["model"]
  expected = model["onnxSha256"].lower()
  if output.is_file() and sha256_file(output) == expected:
    return output

  output.parent.mkdir(parents=True, exist_ok=True)
  fd, temporary_name = tempfile.mkstemp(prefix=output.name + ".", suffix=".tmp", dir=output.parent)
  os.close(fd)
  temporary = Path(temporary_name)
  try:
    request = urllib.request.Request(model["weightUrl"], headers={"User-Agent": "openpilot-objectd-model-fetch/1"})
    with urllib.request.urlopen(request, timeout=120) as response, temporary.open("wb") as stream:
      while chunk := response.read(1024 * 1024):
        stream.write(chunk)
    actual = sha256_file(temporary)
    if actual != expected:
      raise RuntimeError(f"object detector SHA-256 mismatch: expected {expected}, got {actual}")
    temporary.replace(output)
  finally:
    temporary.unlink(missing_ok=True)
  return output


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--output", type=Path, default=MODELS_DIR / "object_detector.onnx")
  parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
  args = parser.parse_args()
  print(fetch_model(args.output, args.manifest))


if __name__ == "__main__":
  main()
