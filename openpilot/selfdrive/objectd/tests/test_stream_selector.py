import unittest

from openpilot.selfdrive.objectd.stream_selector import ROAD_STREAM_NAME, WIDE_STREAM_NAME, select_object_stream


class TestObjectStreamSelector(unittest.TestCase):
  def test_experimental_low_speed_uses_wide(self):
    self.assertEqual(select_object_stream(ROAD_STREAM_NAME, True, 0.0), WIDE_STREAM_NAME)
    self.assertEqual(select_object_stream(ROAD_STREAM_NAME, True, 9.9), WIDE_STREAM_NAME)

  def test_experimental_high_speed_uses_road(self):
    self.assertEqual(select_object_stream(WIDE_STREAM_NAME, True, 15.1), ROAD_STREAM_NAME)

  def test_hysteresis_keeps_current_stream(self):
    self.assertEqual(select_object_stream(WIDE_STREAM_NAME, True, 12.0), WIDE_STREAM_NAME)
    self.assertEqual(select_object_stream(ROAD_STREAM_NAME, True, 12.0), ROAD_STREAM_NAME)

  def test_non_experimental_always_uses_road(self):
    self.assertEqual(select_object_stream(WIDE_STREAM_NAME, False, 0.0), ROAD_STREAM_NAME)

  def test_invalid_current_stream_fails_safe_to_road(self):
    self.assertEqual(select_object_stream("invalid", True, 12.0), ROAD_STREAM_NAME)


if __name__ == "__main__":
  unittest.main()
