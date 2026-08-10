from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RoadPlaneEstimate:
  stable: bool
  coefficients: tuple[float, float, float] = (0.0, 0.0, 0.0)  # z = ax + by + c
  residual_rms_m: float = float("inf")
  curve_count: int = 0
  point_count: int = 0


def fit_road_plane(curves: list[np.ndarray], max_abs_slope: float = 0.02,
                   max_residual_rms_m: float = 0.10) -> RoadPlaneEstimate:
  usable = []
  for curve in curves:
    points = np.asarray(curve, dtype=np.float64)
    if points.ndim == 2 and points.shape[1] == 3 and len(points) >= 3 and np.all(np.isfinite(points)):
      usable.append(points)
  if len(usable) < 2:
    return RoadPlaneEstimate(False, curve_count=len(usable), point_count=sum(len(c) for c in usable))

  points = np.concatenate(usable)
  design = np.column_stack((points[:, 0], points[:, 1], np.ones(len(points))))
  coefficients, *_ = np.linalg.lstsq(design, points[:, 2], rcond=None)
  residuals = points[:, 2] - design @ coefficients
  median = float(np.median(residuals))
  mad = float(np.median(np.abs(residuals - median)))
  if mad > 1e-9:
    mask = np.abs(residuals - median) <= 3.5 * 1.4826 * mad
    if int(np.count_nonzero(mask)) >= 6:
      coefficients, *_ = np.linalg.lstsq(design[mask], points[mask, 2], rcond=None)
      residuals = points[mask, 2] - design[mask] @ coefficients
  rms = float(np.sqrt(np.mean(np.square(residuals))))
  a, b, c = (float(v) for v in coefficients)
  stable = abs(a) <= max_abs_slope and abs(b) <= max_abs_slope and rms <= max_residual_rms_m
  return RoadPlaneEstimate(stable, (a, b, c), rms, len(usable), len(points))
