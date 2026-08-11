from dataclasses import dataclass
import math

from openpilot.selfdrive.objectd.constants import MAX_RESULT_AGE_MS, MAX_UI_FRAME_DELTA


@dataclass(frozen=True)
class OverlayGateInput:
  current_stream_road: bool
  message_stream_road: bool
  service_alive: bool
  service_valid: bool
  state: str
  result_valid: bool
  debug: bool
  inference_frequency_hz: float
  current_frame_id: int
  source_frame_id: int
  current_timestamp_eof: int
  source_timestamp_eof: int
  publish_age_ms: float


def should_render_overlay(value: OverlayGateInput) -> bool:
  if not value.current_stream_road or not value.message_stream_road:
    return False
  if not value.service_alive or not value.service_valid:
    return False
  if value.debug:
    if value.state not in ("running", "degraded"):
      return False
  elif not value.result_valid or value.state != "running" or value.inference_frequency_hz < 4.0:
    return False
  if value.current_timestamp_eof <= 0 or value.current_frame_id < value.source_frame_id:
    return False
  if value.current_frame_id - value.source_frame_id > MAX_UI_FRAME_DELTA:
    return False
  age_ms = (value.current_timestamp_eof - value.source_timestamp_eof) / 1e6
  return 0.0 <= age_ms <= MAX_RESULT_AGE_MS and 0.0 <= value.publish_age_ms <= MAX_RESULT_AGE_MS


def map_normalized_bbox(bbox: tuple[float, float, float, float], rect: tuple[float, float, float, float],
                        scale: tuple[float, float], translation: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float, float, float] | None:
  """Apply the same centered frame transform used by CameraView._render."""
  left, top, right, bottom = bbox
  if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
    return None
  rect_x, rect_y, rect_width, rect_height = rect
  scale_x, scale_y = scale
  translate_x, translate_y = translation
  transform_values = (rect_x, rect_y, rect_width, rect_height, scale_x, scale_y, translate_x, translate_y)
  if not all(math.isfinite(value) for value in transform_values):
    return None
  if rect_width <= 0.0 or rect_height <= 0.0 or scale_x <= 0.0 or scale_y <= 0.0:
    return None
  frame_width = rect_width * scale_x
  frame_height = rect_height * scale_y
  x_offset = rect_x + (rect_width - frame_width) / 2.0 + translate_x * rect_width / 2.0
  y_offset = rect_y + (rect_height - frame_height) / 2.0 + translate_y * rect_height / 2.0
  return (x_offset + left * frame_width, y_offset + top * frame_height,
          (right - left) * frame_width, (bottom - top) * frame_height)
