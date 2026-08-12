from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np

from openpilot.common.transformations.camera import (device_frame_from_view_frame, get_view_frame_from_road_frame,
                                                     view_frame_from_device_frame)
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.objectd.constants import MAX_DISPLAY_DISTANCE_M, MIN_DISPLAY_DISTANCE_M


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


@dataclass(frozen=True)
class CoarseDistanceResult:
  valid: bool
  position: tuple[float, float, float] = (0.0, 0.0, 0.0)
  distance_m: float = 0.0
  distance_std_m: float = 0.0


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
  if not math.isfinite(value.distance_m) or not MIN_DISPLAY_DISTANCE_M <= value.distance_m <= MAX_DISPLAY_DISTANCE_M:
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


def camera_view_from_road(rpy_calib: tuple[float, float, float], camera_height_m: float,
                          camera_from_device_euler: tuple[float, float, float] | None = None) -> np.ndarray:
  view_from_road = get_view_frame_from_road_frame(*rpy_calib, camera_height_m)
  if camera_from_device_euler is None:
    return view_from_road
  camera_view_from_standard_view = (view_frame_from_device_frame @ rot_from_euler(camera_from_device_euler)
                                    @ device_frame_from_view_frame)
  transform = np.eye(4, dtype=np.float64)
  transform[:3, :3] = camera_view_from_standard_view
  return (transform @ np.vstack((view_from_road, [0.0, 0.0, 0.0, 1.0])))[:3]


def _camera_ground_point(contact: tuple[float, float], image_size: tuple[int, int], intrinsics: np.ndarray,
                         rpy_calib: tuple[float, float, float], camera_height_m: float,
                         camera_from_device_euler: tuple[float, float, float] | None = None) -> np.ndarray | None:

  width, height = image_size
  calibration = np.asarray(rpy_calib, dtype=np.float64)
  camera_matrix = np.asarray(intrinsics, dtype=np.float64)
  if width <= 0 or height <= 0 or calibration.shape != (3,) or camera_matrix.shape != (3, 3):
    return None
  if not np.all(np.isfinite([*contact, *calibration, camera_height_m, *camera_matrix.flat])):
    return None
  if not (0.0 < contact[0] < 1.0 and 0.0 < contact[1] < 1.0 and 0.5 <= camera_height_m <= 2.5):
    return None

  pixel = np.array([contact[0] * width, contact[1] * height, 1.0], dtype=np.float64)
  try:
    ray_view = np.linalg.solve(camera_matrix, pixel)
  except np.linalg.LinAlgError:
    return None
  view_from_road = camera_view_from_road(tuple(calibration), camera_height_m, camera_from_device_euler)
  rotation = view_from_road[:, :3]
  translation = view_from_road[:, 3]
  ray_origin_road = -rotation.T @ translation
  ray_direction_road = rotation.T @ ray_view
  point = intersect_local_ground(ray_origin_road, ray_direction_road, np.zeros(3))
  if point is None or point[0] <= 0.0:
    return None
  return point


def estimate_camera_ground_distance(contact: tuple[float, float], image_size: tuple[int, int], intrinsics: np.ndarray,
                                    rpy_calib: tuple[float, float, float], rpy_spread: tuple[float, float, float],
                                    camera_height_m: float, contact_std_normalized: float,
                                    camera_from_device_euler: tuple[float, float, float] | None = None) -> CoarseDistanceResult:
  """Estimate display-only range from SP calibration, relative to the camera ground point."""
  spread = np.asarray(rpy_spread, dtype=np.float64)
  if spread.shape != (3,) or not np.all(np.isfinite(spread)) or np.any(spread < 0.0):
    return CoarseDistanceResult(False)
  if not math.isfinite(contact_std_normalized) or contact_std_normalized <= 0.0:
    return CoarseDistanceResult(False)

  point = _camera_ground_point(contact, image_size, intrinsics, rpy_calib, camera_height_m, camera_from_device_euler)
  if point is None:
    return CoarseDistanceResult(False)
  distance = float(np.hypot(point[0], point[1]))
  if distance < MIN_DISPLAY_DISTANCE_M - 1e-6 or distance > MAX_DISPLAY_DISTANCE_M + 1e-6:
    return CoarseDistanceResult(False)
  distance = min(MAX_DISPLAY_DISTANCE_M, max(MIN_DISPLAY_DISTANCE_M, distance))

  variants = []
  for sign in (-1.0, 1.0):
    varied_contact = (contact[0], contact[1] + sign * contact_std_normalized)
    variants.append(_camera_ground_point(varied_contact, image_size, intrinsics, rpy_calib, camera_height_m,
                                         camera_from_device_euler))
    for axis in range(3):
      varied_rpy = np.asarray(rpy_calib, dtype=np.float64).copy()
      varied_rpy[axis] += sign * spread[axis]
      variants.append(_camera_ground_point(contact, image_size, intrinsics, tuple(varied_rpy), camera_height_m,
                                           camera_from_device_euler))
  variant_distances = [float(np.hypot(value[0], value[1])) for value in variants if value is not None]
  distance_std = max(0.5, max((abs(value - distance) for value in variant_distances), default=0.0))
  if not math.isfinite(distance_std):
    distance_std = 0.0
  return CoarseDistanceResult(True, tuple(float(value) for value in point), distance, distance_std)
