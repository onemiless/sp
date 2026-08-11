import unittest

from openpilot.system.manager.vision_object_latch import VisionObjectSessionLatch


class FakeParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get_bool(self, key):
    assert key == "VisionObjectDetectionEnabled"
    return self.enabled


class FakeCP:
  def __init__(self, brand):
    self.brand = brand


class TestVisionObjectSessionLatch(unittest.TestCase):
  def test_tesla_enabled_on_rising_edge(self):
    latch = VisionObjectSessionLatch()
    params = FakeParams(True)
    self.assertFalse(latch(False, params, FakeCP("tesla")))
    self.assertTrue(latch(True, params, FakeCP("tesla")))

  def test_onroad_param_changes_do_not_change_session(self):
    latch = VisionObjectSessionLatch()
    params = FakeParams(True)
    self.assertTrue(latch(True, params, FakeCP("tesla")))
    params.enabled = False
    self.assertTrue(latch(True, params, FakeCP("tesla")))
    self.assertFalse(latch(False, params, FakeCP("tesla")))
    params.enabled = True
    self.assertTrue(latch(True, params, FakeCP("tesla")))

  def test_waits_for_tesla_car_params_after_rising_edge(self):
    latch = VisionObjectSessionLatch()
    params = FakeParams(True)
    self.assertFalse(latch(True, params, FakeCP("")))
    self.assertTrue(latch(True, params, FakeCP("tesla")))

  def test_falls_back_to_persistent_car_brand(self):
    latch = VisionObjectSessionLatch(persistent_brand_getter=lambda params: "tesla")
    self.assertTrue(latch(True, FakeParams(True), FakeCP("")))

  def test_disabled_at_rising_edge_cannot_enable_onroad(self):
    latch = VisionObjectSessionLatch()
    params = FakeParams(False)
    self.assertFalse(latch(True, params, FakeCP("")))
    params.enabled = True
    self.assertFalse(latch(True, params, FakeCP("tesla")))

  def test_disabled_wrong_brand_or_param(self):
    self.assertFalse(VisionObjectSessionLatch()(True, FakeParams(True), FakeCP("toyota")))
    self.assertFalse(VisionObjectSessionLatch()(True, FakeParams(False), FakeCP("tesla")))

  def test_missing_model_artifact_fails_closed(self):
    latch = VisionObjectSessionLatch(lambda: False)
    self.assertFalse(latch(True, FakeParams(True), FakeCP("tesla")))


if __name__ == "__main__":
  unittest.main()
