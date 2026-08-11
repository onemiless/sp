from collections import deque


BACKOFF_SECONDS = (1.0, 2.0, 4.0, 8.0, 30.0)
FAILURE_WINDOW_S = 600.0
FAILURE_FUSE_COUNT = 5


def stream_connect_timed_out(started_s: float, now_s: float, timeout_s: float) -> bool:
  return timeout_s >= 0.0 and now_s - started_s >= timeout_s


class RetryController:
  def __init__(self):
    self.failures: deque[float] = deque()
    self.failure_count = 0
    self.next_start_s = 0.0
    self.fused = False

  def record_failure(self, now_s: float) -> None:
    self.failures.append(now_s)
    while self.failures and now_s - self.failures[0] > FAILURE_WINDOW_S:
      self.failures.popleft()
    self.failure_count += 1
    self.fused = len(self.failures) >= FAILURE_FUSE_COUNT
    delay = BACKOFF_SECONDS[min(self.failure_count - 1, len(BACKOFF_SECONDS) - 1)]
    self.next_start_s = now_s + delay

  def can_start(self, now_s: float) -> bool:
    return not self.fused and now_s >= self.next_start_s
