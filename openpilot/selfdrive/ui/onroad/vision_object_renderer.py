import time

import pyray as rl
from msgq.visionipc import VisionStreamType

from openpilot.selfdrive.objectd.constants import CLASS_NAMES, TRACK_MAX_PREDICTION_S
from openpilot.selfdrive.objectd.ui_contract import OverlayGateInput, should_render_overlay
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app


ROAD_STREAM = VisionStreamType.VISION_STREAM_ROAD
BOX_COLOR = rl.Color(60, 210, 255, 220)
DEBUG_COLOR = rl.Color(255, 190, 60, 220)
TEXT_COLOR = rl.Color(255, 255, 255, 255)


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
    if not (ui_state.vision_object_overlay or ui_state.vision_object_debug_overlay):
      return
    if self.camera_view.stream_type != ROAD_STREAM:
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
      True, str(state.streamType) == "road", sm.alive["visionObjectStateSP"], sm.valid["visionObjectStateSP"],
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
