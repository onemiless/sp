"""Pure helpers for calibrating camera-ground object ranges against SP leads."""
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

import numpy as np


PARAM_KEY = "VisionObjectDistanceCalibration"
SCHEMA_VERSION = 1
MIN_SAMPLES = 30
MIN_DISTANCE_SPAN_M = 15.0
SUPPORTED_STREAMS = ("road", "wide")
MOTOR_VEHICLE_CLASSES = frozenset((2, 4, 5))  # car, bus, truck


@dataclass(frozen=True)
class CalibrationSample:
  stream: str
  track_id: int
  raw_forward_m: float
  raw_lateral_m: float
  sp_distance_m: float
  sp_lateral_m: float
  radar: bool
  timestamp_ns: int = 0


@dataclass(frozen=True)
class CalibrationProfile:
  stream: str
  scale: float
  offset_m: float
  sample_count: int
  rmse_m: float
  created_at: int


def apply_profile(position: Sequence[float], profile: CalibrationProfile) -> tuple[tuple[float, float, float], float]:
  """Apply longitudinal offset and common camera scale, then return ground-plane slant range."""
  x, y, z = (float(value) for value in position)
  corrected = (profile.scale * x + profile.offset_m, profile.scale * y, z)
  return corrected, math.hypot(corrected[0], corrected[1])


def invert_profile(position: Sequence[float], profile: CalibrationProfile) -> tuple[float, float, float]:
  """Recover raw camera-ground coordinates so a profile can be recalibrated without compounding it."""
  x, y, z = (float(value) for value in position)
  return ((x - profile.offset_m) / profile.scale, y / profile.scale, z)


def _finite(value: Any) -> bool:
  try:
    return math.isfinite(float(value))
  except (TypeError, ValueError):
    return False


def match_lead_to_object(lead: Mapping[str, Any], objects: Sequence[Mapping[str, Any]], stream: str,
                         timestamp_ns: int) -> CalibrationSample | None:
  """Conservatively associate one SP ego-lane lead with one YOLO motor vehicle."""
  if stream not in SUPPORTED_STREAMS or not lead.get("present", False):
    return None
  if not _finite(lead.get("dRel")) or not _finite(lead.get("yRel")):
    return None
  sp_x, sp_y = float(lead["dRel"]), float(lead["yRel"])
  if not 5.0 <= sp_x <= 100.0:
    return None

  candidates: list[tuple[float, Mapping[str, Any]]] = []
  longitudinal_gate = max(6.0, 0.25 * sp_x)
  for obj in objects:
    if (int(obj.get("classId", -1)) not in MOTOR_VEHICLE_CLASSES or
        obj.get("corridorState") != "inside" or not obj.get("distanceValid", False) or
        not _finite(obj.get("x")) or not _finite(obj.get("y"))):
      continue
    raw_x, raw_y = float(obj["x"]), float(obj["y"])
    dx, dy = abs(raw_x - sp_x), abs(raw_y - sp_y)
    if not 3.0 <= raw_x <= 120.0 or dx > longitudinal_gate or dy > 2.0:
      continue
    score = dx / longitudinal_gate + dy / 1.5
    candidates.append((score, obj))

  if not candidates:
    return None
  candidates.sort(key=lambda item: item[0])
  if candidates[0][0] > 0.8:
    return None
  if len(candidates) > 1:
    best, second = candidates[0][0], candidates[1][0]
    if second - best < 0.15 or second < max(0.20, best * 1.25):
      return None

  obj = candidates[0][1]
  return CalibrationSample(stream, int(obj["trackId"]), float(obj["x"]), float(obj["y"]), sp_x, sp_y,
                           bool(lead.get("radar", False)), int(timestamp_ns))


def fit_profile(samples: Iterable[CalibrationSample], stream: str,
                created_at: int | None = None) -> CalibrationProfile | None:
  """Fit a bounded robust affine map from raw forward range to SP front-bumper range."""
  valid = [sample for sample in samples if sample.stream == stream and
           _finite(sample.raw_forward_m) and _finite(sample.sp_distance_m) and
           5.0 <= sample.raw_forward_m <= 120.0 and 5.0 <= sample.sp_distance_m <= 100.0]
  if len(valid) < MIN_SAMPLES:
    return None
  x = np.asarray([sample.raw_forward_m for sample in valid], dtype=np.float64)
  y = np.asarray([sample.sp_distance_m for sample in valid], dtype=np.float64)
  if np.ptp(x) < MIN_DISTANCE_SPAN_M or np.ptp(y) < MIN_DISTANCE_SPAN_M:
    return None

  x10, x90 = np.percentile(x, (10, 90))
  y10, y90 = np.percentile(y, (10, 90))
  if x90 - x10 < 1e-6:
    return None
  scale = float((y90 - y10) / (x90 - x10))
  offset = float(np.median(y - scale * x))
  mask = np.ones(len(x), dtype=bool)
  for _ in range(4):
    residual = y - (scale * x + offset)
    center = float(np.median(residual[mask]))
    mad = float(np.median(np.abs(residual[mask] - center)))
    new_mask = np.abs(residual - center) <= max(0.75, 3.0 * 1.4826 * mad)
    if int(np.count_nonzero(new_mask)) < MIN_SAMPLES:
      return None
    design = np.column_stack((x[new_mask], np.ones(int(np.count_nonzero(new_mask)))))
    scale, offset = (float(value) for value in np.linalg.lstsq(design, y[new_mask], rcond=None)[0])
    mask = new_mask

  if not 0.5 <= scale <= 1.5 or not -10.0 <= offset <= 10.0:
    return None
  residual = y[mask] - (scale * x[mask] + offset)
  rmse = float(np.sqrt(np.mean(np.square(residual))))
  if not math.isfinite(rmse) or rmse > 3.0:
    return None
  return CalibrationProfile(stream, scale, offset, int(np.count_nonzero(mask)), rmse,
                            int(time.time_ns() // int(1e9) if created_at is None else created_at))


def profiles_to_payload(profiles: Mapping[str, CalibrationProfile]) -> dict[str, Any]:
  return {
    "schemaVersion": SCHEMA_VERSION,
    "updatedAt": int(time.time_ns() // int(1e9)),
    "profiles": {stream: asdict(profile) for stream, profile in profiles.items() if stream in SUPPORTED_STREAMS},
  }


def profiles_to_json(profiles: Mapping[str, CalibrationProfile]) -> str:
  return json.dumps(profiles_to_payload(profiles), separators=(",", ":"), sort_keys=True)


def profiles_from_json(raw: bytes | str | Mapping[str, Any] | None) -> dict[str, CalibrationProfile]:
  if not raw:
    return {}
  try:
    payload = raw if isinstance(raw, Mapping) else json.loads(raw.decode() if isinstance(raw, bytes) else raw)
    if payload.get("schemaVersion") != SCHEMA_VERSION:
      return {}
    profiles = {}
    for stream, values in payload.get("profiles", {}).items():
      profile = CalibrationProfile(**values)
      if (stream in SUPPORTED_STREAMS and profile.stream == stream and 0.5 <= profile.scale <= 1.5 and
          -10.0 <= profile.offset_m <= 10.0 and profile.sample_count >= MIN_SAMPLES and profile.rmse_m <= 3.0):
        profiles[stream] = profile
    return profiles
  except (AttributeError, json.JSONDecodeError, TypeError, ValueError):
    return {}
