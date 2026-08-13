#!/usr/bin/env python3
"""Calibrate YOLO ground distance against conservatively matched SP lead distances."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.objectd.distance_calibration import (PARAM_KEY, CalibrationSample, fit_profile, invert_profile,
                                                              match_lead_to_object, profiles_from_json,
                                                              profiles_to_payload)


DEFAULT_OUTPUT_DIR = Path("/data/vision_object_distance_calibration")
MAX_SAMPLES = 2000
MAX_OUTPUT_BYTES = 50 * 1024 * 1024
MIN_TRACK_STREAK = 3
MAX_RADAR_AGE_NS = int(1e9)


def _lead_payload(lead) -> dict:
  return {
    "dRel": float(lead.dRel), "yRel": float(lead.yRel), "present": bool(lead.present),
    "radar": bool(lead.radar), "modelProb": float(lead.modelProb),
  }


def _object_payload(obj, profile) -> dict:
  position = (float(obj.x), float(obj.y), float(obj.z))
  if profile is not None:
    position = invert_profile(position, profile)
  return {
    "trackId": int(obj.trackId), "classId": int(obj.classId), "corridorState": str(obj.corridorState),
    "x": position[0], "y": position[1], "distanceValid": bool(obj.distanceValid),
  }


def _write_samples(path: Path, samples: list[CalibrationSample]) -> None:
  with path.open("x", encoding="utf-8") as stream:
    stream.write(json.dumps({"type": "header", "formatVersion": 1, "containsCameraFrames": False}) + "\n")
    for sample in samples:
      stream.write(json.dumps({"type": "sample", **asdict(sample)}, separators=(",", ":")) + "\n")


def _prune(directory: Path) -> None:
  files = sorted(directory.glob("distance-calibration-*.jsonl"), key=lambda path: path.stat().st_mtime)
  total = sum(path.stat().st_size for path in files)
  for path in files:
    if total <= MAX_OUTPUT_BYTES:
      break
    total -= path.stat().st_size
    path.unlink(missing_ok=True)


def collect(duration_s: float, output_dir: Path, min_interval_s: float) -> int:
  output_dir.mkdir(parents=True, exist_ok=True)
  _prune(output_dir)
  params = Params()
  profiles = profiles_from_json(params.get(PARAM_KEY))
  sm = messaging.SubMaster(["visionObjectStateSP", "radarState"])
  samples: list[CalibrationSample] = []
  streaks: dict[tuple[str, int], tuple[int, int]] = {}
  last_accepted_ns: dict[tuple[str, int], int] = {}
  deadline = time.monotonic() + duration_s
  interrupted = False

  try:
    while time.monotonic() < deadline and len(samples) < MAX_SAMPLES:
      sm.update(1000)
      if not (sm.updated["visionObjectStateSP"] and sm.seen["radarState"] and sm.valid["radarState"] and
              sm.alive["radarState"]):
        continue
      state = sm["visionObjectStateSP"]
      if not state.resultValid or str(state.distanceMode) == "invalid":
        continue
      if abs(int(state.publishMonoTime) - int(sm.logMonoTime["radarState"])) > MAX_RADAR_AGE_NS:
        continue

      stream_name = str(state.streamType)
      objects = [_object_payload(obj, profiles.get(stream_name)) for obj in state.objects]
      used_tracks: set[int] = set()
      for lead_index, lead in enumerate((sm["radarState"].leadOne, sm["radarState"].leadTwo)):
        streak_key = (stream_name, lead_index)
        match = match_lead_to_object(_lead_payload(lead), objects, stream_name, int(state.publishMonoTime))
        if match is None or match.track_id in used_tracks:
          streaks.pop(streak_key, None)
          continue
        previous_track, previous_count = streaks.get(streak_key, (-1, 0))
        count = previous_count + 1 if previous_track == match.track_id else 1
        streaks[streak_key] = (match.track_id, count)
        if count < MIN_TRACK_STREAK:
          continue
        key = (stream_name, match.track_id)
        if match.timestamp_ns - last_accepted_ns.get(key, 0) < int(min_interval_s * 1e9):
          continue
        samples.append(match)
        last_accepted_ns[key] = match.timestamp_ns
        used_tracks.add(match.track_id)
  except KeyboardInterrupt:
    interrupted = True

  output_path = output_dir / f"distance-calibration-{time.time_ns()}.jsonl"
  _write_samples(output_path, samples)
  _prune(output_dir)
  fitted = {stream: fit_profile(samples, stream) for stream in ("road", "wide")}
  accepted = {stream: profile for stream, profile in fitted.items() if profile is not None}
  if accepted:
    params.put(PARAM_KEY, profiles_to_payload({**profiles, **accepted}), block=True)
  summary = {
    "interrupted": interrupted, "samples": len(samples), "sampleFile": str(output_path),
    "radarSamples": sum(sample.radar for sample in samples),
    "profilesWritten": {stream: asdict(profile) for stream, profile in accepted.items()},
    "profilesRejected": [stream for stream, profile in fitted.items()
                         if profile is None and any(sample.stream == stream for sample in samples)],
    "activation": "restart objectd or make an offroad-to-onroad transition",
  }
  print(json.dumps(summary, indent=2, sort_keys=True))
  return 0 if accepted else 2


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  commands = parser.add_subparsers(dest="command", required=True)
  collect_parser = commands.add_parser("collect", help="collect matched samples, robustly fit, and save valid profiles")
  collect_parser.add_argument("--duration", type=float, default=600.0)
  collect_parser.add_argument("--min-interval", type=float, default=0.5)
  collect_parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
  commands.add_parser("status", help="print the active profiles")
  commands.add_parser("reset", help="remove all distance calibration profiles")
  args = parser.parse_args()
  params = Params()
  if args.command == "collect":
    if args.duration <= 0.0 or args.min_interval < 0.1:
      parser.error("duration must be positive and min-interval must be at least 0.1 seconds")
    raise SystemExit(collect(args.duration, args.output_dir, args.min_interval))
  if args.command == "reset":
    params.remove(PARAM_KEY)
    print("distance calibration profiles removed; restart objectd to use raw geometry")
  else:
    print(json.dumps(params.get(PARAM_KEY) or {"schemaVersion": 1, "profiles": {}}, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
