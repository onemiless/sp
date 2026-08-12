import unittest

from openpilot.selfdrive.objectd.constants import (DEGRADED_INFERENCE_HZ, MAX_RESULT_AGE_MS, MAX_UI_FRAME_DELTA,
                                                   NORMAL_INFERENCE_HZ, TRACK_MAX_PREDICTION_S)
from openpilot.selfdrive.objectd.ui_contract import (OverlayGateInput, StatusBadgeInput, map_normalized_bbox,
                                                     object_status_badge, should_render_overlay)


def valid_input(**overrides):
  values = {
    "current_stream_road": True, "message_stream_road": True, "service_alive": True, "service_valid": True,
    "state": "running", "result_valid": True, "debug": False, "inference_frequency_hz": 3.0,
    "current_frame_id": 105, "source_frame_id": 100, "current_timestamp_eof": 1_200_000_000,
    "source_timestamp_eof": 1_000_000_000, "publish_age_ms": 220.0,
  }
  values.update(overrides)
  return OverlayGateInput(**values)


class TestOverlayGate(unittest.TestCase):
  def test_valid_road_message(self):
    self.assertTrue(should_render_overlay(valid_input()))

  def test_wide_always_hidden(self):
    self.assertFalse(should_render_overlay(valid_input(current_stream_road=False)))
    self.assertFalse(should_render_overlay(valid_input(message_stream_road=False)))

  def test_stale_or_future_frame_hidden(self):
    self.assertFalse(should_render_overlay(valid_input(publish_age_ms=501.0)))
    self.assertFalse(should_render_overlay(valid_input(current_frame_id=99)))
    self.assertFalse(should_render_overlay(valid_input(current_frame_id=111)))

  def test_low_frequency_only_debug(self):
    self.assertFalse(should_render_overlay(valid_input(state="degraded", result_valid=False, inference_frequency_hz=2.0)))
    self.assertTrue(should_render_overlay(valid_input(state="degraded", result_valid=False, debug=True, inference_frequency_hz=2.0)))

  def test_three_hz_display_contract(self):
    self.assertEqual(NORMAL_INFERENCE_HZ, 3.0)
    self.assertEqual(DEGRADED_INFERENCE_HZ, 2.0)
    self.assertEqual(MAX_RESULT_AGE_MS, 500.0)
    self.assertEqual(MAX_UI_FRAME_DELTA, 10)
    self.assertEqual(TRACK_MAX_PREDICTION_S, 0.5)
    self.assertFalse(should_render_overlay(valid_input(inference_frequency_hz=2.9)))


class TestStatusBadge(unittest.TestCase):
  def badge(self, **overrides):
    values = {
      "enabled": True, "process_running": True, "service_alive": True, "service_valid": True,
      "state": "running", "result_valid": True, "error_code": "none", "current_stream_road": True,
      "inference_frequency_hz": 3.0,
    }
    values.update(overrides)
    return object_status_badge(StatusBadgeInput(**values))

  def test_running_is_green_even_with_no_objects(self):
    badge = self.badge()
    self.assertEqual(badge.level, "running")
    self.assertIn("3.0", badge.text)

  def test_wide_explains_why_boxes_are_hidden(self):
    badge = self.badge(current_stream_road=False)
    self.assertEqual(badge.level, "wide")
    self.assertIn("WIDE", badge.text)

  def test_waiting_and_disabled_are_distinct(self):
    self.assertEqual(self.badge(process_running=False, service_alive=False).level, "waiting")
    self.assertEqual(self.badge(enabled=False, process_running=False, service_alive=False).level, "disabled")

  def test_fused_is_error_and_stale_is_degraded(self):
    self.assertEqual(self.badge(state="sessionFused", result_valid=False, error_code="circuitBreaker").level, "error")
    self.assertEqual(self.badge(state="degraded", result_valid=False, error_code="stale").level, "degraded")


class TestBboxMapping(unittest.TestCase):
  def test_road_1928x1208_letterbox_corners_and_center(self):
    # CameraView renders a 1928x1208 ROAD frame centered inside this square viewport.
    scale = (1.0, (1208 / 1928))
    full = map_normalized_bbox((0.0, 0.0, 1.0, 1.0), (10.0, 20.0, 1000.0, 1000.0), scale)
    self.assertIsNotNone(full)
    self.assertAlmostEqual(full[0], 10.0)
    self.assertAlmostEqual(full[1], 20.0 + (1000.0 - 1000.0 * scale[1]) / 2.0)
    self.assertAlmostEqual(full[2], 1000.0)
    self.assertAlmostEqual(full[3], 1000.0 * scale[1])
    center = map_normalized_bbox((0.49, 0.49, 0.51, 0.51), (10.0, 20.0, 1000.0, 1000.0), scale)
    self.assertAlmostEqual(center[0] + center[2] / 2.0, 510.0)
    self.assertAlmostEqual(center[1] + center[3] / 2.0, 520.0)

  def test_c3x_augmented_road_crop_scale(self):
    # 2160x1080 display, 30 px UI border, AR0231 ROAD intrinsics and 1.1x camera zoom.
    rect = (30.0, 30.0, 2100.0, 1020.0)
    scale = (1.1 * 1928 / rect[2], 1.1 * 1208 / rect[3])
    center = map_normalized_bbox((0.45, 0.45, 0.55, 0.55), rect, scale)
    self.assertIsNotNone(center)
    self.assertAlmostEqual(center[0] + center[2] / 2.0, 1080.0)
    self.assertAlmostEqual(center[1] + center[3] / 2.0, 540.0)

  def test_invalid_bbox_rejected(self):
    self.assertIsNone(map_normalized_bbox((0.8, 0.1, 0.2, 0.5), (0.0, 0.0, 100.0, 100.0), (1.0, 1.0)))


if __name__ == "__main__":
  unittest.main()
