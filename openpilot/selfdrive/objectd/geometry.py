from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np


class DistanceInvalidReason(IntEnum):
  NONE = 0
  EXTRINSICS = 1
  CALIBRATION = 2
  CONTACT_POINT = 3
  ROAD_PLANE = 4
  DYNAMIC_MOTION = 5
  RANGE = 6
  UNCERTAINTY = 7
  STALE = 8


@dataclass(frozen=True)
class DistanceGateInput:
  vehicle_extrinsics_valid: bool = False
  calibration_valid: bool = False
  contact_point_valid: bool = False
  road_plane_stable: bool = False
  acceleration_mps2: float = math.inf
  controls_stable_for_s: float = 0.0
  distance_m: float = math.nan
  distance_std_m: float = math.inf
  result_age_ms: float = math.inf


@dataclass(frozen=True)
class DistanceResult:
  valid: bool
  distance_m: float = 0.0
  distance_std_m: float = 0.0
  reason: DistanceInvalidReason = DistanceInvalidReason.NONE


def gate_metric_distance(value: DistanceGateInput) -> DistanceResult:
  if value.result_age_ms > 300.0:
    return DistanceResult(False, reason=DistanceInvalidReason.STALE)
  if not value.vehicle_extrinsics_valid:
    return DistanceResult(False, reason=DistanceInvalidReason.EXTRINSICS)
  if not value.calibration_valid:
    return DistanceResult(False, reason=DistanceInvalidReason.CALIBRATION)
  if not value.contact_point_valid:
    return DistanceResult(False, reason=DistanceInvalidReason.CONTACT_POINT)
  if not value.road_plane_stable:
    return DistanceResult(False, reason=DistanceInvalidReason.ROAD_PLANE)
  if abs(value.acceleration_mps2) >= 0.5 or value.controls_stable_for_s < 1.0:
    return DistanceResult(False, reason=DistanceInvalidReason.DYNAMIC_MOTION)
  if not math.isfinite(value.distance_m) or not 5.0 <= value.distance_m <= 30.0:
    return DistanceResult(False, reason=DistanceInvalidReason.RANGE)
  if not math.isfinite(value.distance_std_m) or value.distance_std_m <= 0.0 or value.distance_std_m > 3.0:
    return DistanceResult(False, reason=DistanceInvalidReason.UNCERTAINTY)
  return DistanceResult(True, value.distance_m, value.distance_std_m)


def intersect_local_ground(ray_origin_camera_road: np.ndarray, ray_direction_camera_road: np.ndarray,
                           t_front_in_camera_road: np.ndarray) -> np.ndarray | None:
  """Intersect a camera-road ray with z=0 and translate to front-bumper road frame."""
  origin = np.asarray(ray_origin_camera_road, dtype=np.float64)
  direction = np.asarray(ray_direction_camera_road, dtype=np.float64)
  translation = np.asarray(t_front_in_camera_road, dtype=np.float64)
  if origin.shape != (3,) or direction.shape != (3,) or translation.shape != (3,):
    raise ValueError("origin, direction and translation must be length-three vectors")
  if not np.all(np.isfinite([origin, direction, translation])) or abs(direction[2]) < 1e-9:
    return None
  scale = -origin[2] / direction[2]
  if scale <= 0.0:
    return None
  point_camera_road = origin + scale * direction
  point_vehicle_road = point_camera_road - translation
  point_vehicle_road[2] = 0.0
  return point_vehicle_road
