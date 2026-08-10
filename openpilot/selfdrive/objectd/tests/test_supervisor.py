import unittest

from openpilot.selfdrive.objectd.supervisor import RetryController


class TestRetryController(unittest.TestCase):
  def test_backoff_and_session_fuse(self):
    retry = RetryController()
    expected = [1.0, 2.0, 4.0, 8.0]
    for index, delay in enumerate(expected):
      retry.record_failure(float(index * 10))
      self.assertAlmostEqual(retry.next_start_s, index * 10 + delay)
      self.assertFalse(retry.fused)
    retry.record_failure(40.0)
    self.assertTrue(retry.fused)
    self.assertFalse(retry.can_start(1000.0))

  def test_failure_window_expires(self):
    retry = RetryController()
    for t in (0.0, 1.0, 2.0, 3.0):
      retry.record_failure(t)
    retry.record_failure(700.0)
    self.assertFalse(retry.fused)


if __name__ == "__main__":
  unittest.main()
