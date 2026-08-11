import unittest

from openpilot.selfdrive.objectd.supervisor import (RetryController, model_service_available,
                                                    stream_connect_timed_out, worker_timed_out)


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

  def test_worker_uses_longer_deadline_until_first_message(self):
    self.assertFalse(worker_timed_out(0.0, False, 29.9, 2.0, 30.0))
    self.assertTrue(worker_timed_out(0.0, False, 30.1, 2.0, 30.0))
    self.assertFalse(worker_timed_out(10.0, True, 11.9, 2.0, 30.0))
    self.assertTrue(worker_timed_out(10.0, True, 12.1, 2.0, 30.0))

  def test_model_service_waits_for_first_message_then_fails_closed(self):
    self.assertTrue(model_service_available(False, False))
    self.assertTrue(model_service_available(True, True))
    self.assertFalse(model_service_available(True, False))

  def test_failure_window_expires(self):
    retry = RetryController()
    for t in (0.0, 1.0, 2.0, 3.0):
      retry.record_failure(t)
    retry.record_failure(700.0)
    self.assertFalse(retry.fused)

  def test_stream_connect_deadline(self):
    self.assertFalse(stream_connect_timed_out(10.0, 14.9, 5.0))
    self.assertTrue(stream_connect_timed_out(10.0, 15.0, 5.0))


if __name__ == "__main__":
  unittest.main()
