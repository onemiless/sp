import time

import pyray as rl
from msgq.visionipc import VisionStreamType

from openpilot.selfdrive.objectd.constants import CLASS_NAMES, TRACK_MAX_PREDICTION_S
from openpilot.selfdrive.objectd.ui_contract import (OverlayGateInput, StatusBadgeInput, object_status_badge,
                                                     should_render_overlay)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached


ROAD_STREAM = VisionStreamType.VISION_STREAM_ROAD
BOX_COLOR = rl.Color(60, 210, 255, 220)
DEBUG_COLOR = rl.Color(255, 190, 60, 220)
TEXT_COLOR = rl.Color(255, 255, 255, 255)
BADGE_COLORS = {
  "running": rl.Color(60, 210, 110, 255),
  "switching": rl.Color(255, 196, 60, 255),
  "degraded": rl.Color(255, 140, 45, 255),
  "error": rl.Color(245, 65, 65, 255),
  "waiting": rl.Color(150, 160, 170, 255),
  "disabled": rl.Color(100, 105, 110, 255),
}
BADGE_FONT_SIZE = 34
BADGE_HEIGHT = 62


class VisionObjectRenderer:
  def __init__(self, camera_view):
    self.camera_view = camera_view
    self.font = gui_app.font(FontWeight.SEMI_BOLD)

  @staticmethod
  def _predicted_bbox(obj, dt_s: float) -> tuple[float, float, float, float] | None:
    bbox_value = obj.bboxNormalized
    velocity_value = obj.bboxVelocityNormalized
    bbox = [float(bbox_value.left), float(bbox_value.top), float(bbox_value.right), float(bbox_value.bottom)]
    velocity = [float(velocity_value.leftPerSecond), float(velocity_value.topPerSecond),
                float(velocity_value.rightPerSecond), float(velocity_value.bottomPerSecond)]
    if dt_s > 0.0:
      if not obj.bboxPredictionValid or dt_s > TRACK_MAX_PREDICTION_S:
        return None
      bbox = [value + rate * dt_s for value, rate in zip(bbox, velocity, strict=True)]
    bbox = [min(1.0, max(0.0, value)) for value in bbox]
    if not (bbox[0] < bbox[2] and bbox[1] < bbox[3]):
      return None
    return tuple(bbox)

  def render(self, rect: rl.Rectangle) -> None:
    self._render_status_badge(rect)
    if not (ui_state.vision_object_overlay or ui_state.vision_object_debug_overlay):
      return
    sm = ui_state.sm
    state = sm["visionObjectStateSP"]
    debug = ui_state.vision_object_debug_overlay
    if state.sourceWidth <= 0 or state.sourceHeight <= 0:
      return

    current_frame_id = self.camera_view.display_frame_id
    current_timestamp = self.camera_view.display_timestamp_eof
    age_ms = (current_timestamp - state.sourceTimestampEof) / 1e6
    publish_age_ms = state.resultAgeMs + max(0.0, (time.monotonic_ns() - state.publishMonoTime) / 1e6)
    if not should_render_overlay(OverlayGateInput(
      self.camera_view.stream_type in (VisionStreamType.VISION_STREAM_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD),
      self._stream_name() == str(state.streamType), sm.alive["visionObjectStateSP"], sm.valid["visionObjectStateSP"],
      str(state.state), bool(state.resultValid), debug, float(state.inferenceFrequencyHz), current_frame_id,
      int(state.sourceFrameId), current_timestamp, int(state.sourceTimestampEof), publish_age_ms,
    )):
      return

    dt_s = min(age_ms / 1000.0, TRACK_MAX_PREDICTION_S)
    color = DEBUG_COLOR if debug and not state.resultValid else BOX_COLOR
    for obj in list(state.objects)[:32]:
      bbox = self._predicted_bbox(obj, dt_s)
      if bbox is None:
        continue
      screen = self.camera_view.normalized_bbox_to_screen(rect, bbox)
      if screen is None or screen.width < 2 or screen.height < 2:
        continue
      rl.draw_rectangle_lines_ex(screen, 4.0, color)
      class_id = int(obj.classId)
      class_name = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else f"class-{class_id}"
      label = f"{class_name} {float(obj.confidence):.2f}"
      if ui_state.vision_object_distance_display and obj.distanceValid:
        label += f" ~{float(obj.distance):.1f}m"
      if debug:
        label += f" #{int(obj.trackId)} {age_ms:.0f}ms"
      rl.draw_text_ex(self.font, label, rl.Vector2(screen.x, max(rect.y, screen.y - 32)), 28, 0, TEXT_COLOR)

  def _render_status_badge(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    state = sm["visionObjectStateSP"]
    process_running = any(p.name == "objectd" and p.running for p in sm["managerState"].processes)
    badge = object_status_badge(StatusBadgeInput(
      enabled=ui_state.vision_object_detection_enabled,
      process_running=process_running,
      service_alive=sm.alive["visionObjectStateSP"],
      service_valid=sm.valid["visionObjectStateSP"],
      state=str(state.state),
      result_valid=bool(state.resultValid),
      error_code=str(state.errorCode),
      message_stream_matches=self._stream_name() == str(state.streamType),
      inference_frequency_hz=float(state.inferenceFrequencyHz),
    ))
    text_size = measure_text_cached(self.font, badge.text, BADGE_FONT_SIZE)
    badge_width = text_size.x + 70
    badge_rect = rl.Rectangle(rect.x + rect.width - badge_width - 245, rect.y + 48, badge_width, BADGE_HEIGHT)
    rl.draw_rectangle_rounded(badge_rect, 0.45, 10, rl.Color(0, 0, 0, 205))
    rl.draw_rectangle_rounded_lines_ex(badge_rect, 0.45, 10, 3, rl.Color(255, 255, 255, 70))
    color = BADGE_COLORS[badge.level]
    rl.draw_circle(int(badge_rect.x + 27), int(badge_rect.y + BADGE_HEIGHT / 2), 10, color)
    rl.draw_text_ex(self.font, badge.text,
                    rl.Vector2(badge_rect.x + 48, badge_rect.y + (BADGE_HEIGHT - text_size.y) / 2),
                    BADGE_FONT_SIZE, 0, TEXT_COLOR)

  def _stream_name(self) -> str:
    return "wide" if self.camera_view.stream_type == VisionStreamType.VISION_STREAM_WIDE_ROAD else "road"
