from collections.abc import Sequence
import math

import numpy as np


LANE_LINE_MIN_PROBABILITY = 0.5
LANE_BOUNDARY_OVERLAP_M = 0.35


def _line_y_at_x(line: tuple[Sequence[float], Sequence[float]], x_m: float) -> float | None:
  x_values = np.asarray(line[0], dtype=np.float64)
  y_values = np.asarray(line[1], dtype=np.float64)
  if x_values.ndim != 1 or y_values.ndim != 1 or len(x_values) < 2 or len(x_values) != len(y_values):
    return None
  finite = np.isfinite(x_values) & np.isfinite(y_values)
  x_values, y_values = x_values[finite], y_values[finite]
  if len(x_values) < 2 or x_m < x_values[0] or x_m > x_values[-1]:
    return None
  return float(np.interp(x_m, x_values, y_values))


def classify_lane_corridor(position: tuple[float, float], lane_lines: Sequence[tuple[Sequence[float], Sequence[float]]],
                           lane_line_probs: Sequence[float]) -> str:
  """Classify a ground point against SP's outer-left, ego-left, ego-right, outer-right lane lines."""
  x_m, y_m = position
  if not (math.isfinite(x_m) and math.isfinite(y_m)) or x_m <= 0.0:
    return "unknown"
  if len(lane_lines) < 4 or len(lane_line_probs) < 4:
    return "unknown"
  try:
    probs = [float(value) for value in lane_line_probs[:4]]
  except (TypeError, ValueError):
    return "unknown"
  if not all(math.isfinite(value) and value >= LANE_LINE_MIN_PROBABILITY for value in probs):
    return "unknown"

  boundaries = [_line_y_at_x(line, x_m) for line in lane_lines[:4]]
  if any(value is None for value in boundaries):
    return "unknown"
  # modelV2 uses image/model convention: negative y is left, positive y is right.
  outer_left, ego_left, ego_right, outer_right = (float(value) for value in boundaries)
  if not outer_left < ego_left < ego_right < outer_right:
    return "unknown"
  if min(abs(y_m - boundary) for boundary in boundaries) <= LANE_BOUNDARY_OVERLAP_M:
    return "overlap"
  if ego_left < y_m < ego_right:
    return "inside"
  if outer_left < y_m < ego_left or ego_right < y_m < outer_right:
    return "outside"
  return "unknown"
