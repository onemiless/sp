#!/usr/bin/env python3
"""Bounded, metadata-only objectd validation recorder. It never stores camera frames."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

from openpilot.cereal import messaging
from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params


MAX_FILE_BYTES = 50 * 1024 * 1024
MAX_DIRECTORY_BYTES = 500 * 1024 * 1024
PARAM_KEYS = ("VisionObjectDetectionEnabled", "VisionObjectOverlay", "VisionObjectDistanceDisplay",
              "VisionObjectDebugOverlay", "VisionObjectRecordValidation")
DEFAULT_OUTPUT_DIR = Path(Paths.log_root()).parent / "vision_object_validation"


def _sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def _prune(directory: Path, reserve_bytes: int = 0) -> None:
  files = sorted(directory.glob("vision-object-*.jsonl"), key=lambda path: path.stat().st_mtime)
  total = sum(path.stat().st_size for path in files)
  for path in files:
    if total <= MAX_DIRECTORY_BYTES - reserve_bytes:
      break
    size = path.stat().st_size
    path.unlink(missing_ok=True)
    total -= size


def _header(repo: Path) -> dict:
  manifest_path = repo / "openpilot/selfdrive/objectd/models/object_detector_manifest.json"
  params = Params()
  try:
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo, check=True, capture_output=True, text=True).stdout.strip()
  except (OSError, subprocess.SubprocessError):
    commit = "unknown"
  return {
    "type": "header", "formatVersion": 1, "createdMonoTime": time.monotonic_ns(), "commit": commit,
    "manifestSha256": _sha256(manifest_path), "deviceType": HARDWARE.get_device_type(),
    "params": {key: params.get_bool(key) for key in PARAM_KEYS}, "containsCameraFrames": False,
  }


def _record(state) -> dict:
  return {
    "type": "sample", "recordMonoTime": time.monotonic_ns(), "state": str(state.state),
    "streamType": str(state.streamType), "sourceFrameId": int(state.sourceFrameId),
    "sourceTimestampEof": int(state.sourceTimestampEof), "publishMonoTime": int(state.publishMonoTime),
    "inferenceUpdated": bool(state.inferenceUpdated), "inferenceDurationMs": float(state.inferenceDurationMs),
    "resultAgeMs": float(state.resultAgeMs), "resultValid": bool(state.resultValid),
    "errorCode": str(state.errorCode), "inferenceFrequencyHz": float(state.inferenceFrequencyHz),
    "droppedObjectCount": int(state.droppedObjectCount),
    "objects": [{
      "trackId": int(obj.trackId), "classId": int(obj.classId), "confidence": float(obj.confidence),
      "bboxNormalized": [float(obj.bboxNormalized.left), float(obj.bboxNormalized.top),
                         float(obj.bboxNormalized.right), float(obj.bboxNormalized.bottom)],
      "distanceValid": bool(obj.distanceValid), "distance": float(obj.distance) if obj.distanceValid else 0.0,
    } for obj in list(state.objects)[:32]],
  }


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
  args = parser.parse_args()
  args.output_dir.mkdir(parents=True, exist_ok=True)
  _prune(args.output_dir, MAX_FILE_BYTES)

  repo = Path(__file__).resolve().parents[4]
  sm = messaging.SubMaster(["visionObjectStateSP"])
  path = args.output_dir / f"vision-object-{time.monotonic_ns()}.jsonl"
  try:
    with path.open("x", encoding="utf-8") as stream:
      stream.write(json.dumps(_header(repo), separators=(",", ":")) + "\n")
      while stream.tell() < MAX_FILE_BYTES:
        sm.update(1000)
        if sm.updated["visionObjectStateSP"]:
          stream.write(json.dumps(_record(sm["visionObjectStateSP"]), separators=(",", ":")) + "\n")
          stream.flush()
  except KeyboardInterrupt:
    pass
  except (OSError, ValueError) as exc:
    raise SystemExit(f"validation recording stopped without affecting objectd: {exc}") from exc
  finally:
    _prune(args.output_dir)


if __name__ == "__main__":
  main()
