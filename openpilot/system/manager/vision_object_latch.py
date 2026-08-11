from collections.abc import Callable


class VisionObjectSessionLatch:
  """Latch the ROAD detector decision once at the offroad-to-onroad edge."""

  def __init__(self, artifact_available_getter: Callable[[], bool] = lambda: True):
    self._artifact_available_getter = artifact_available_getter
    self._started = False
    self._session_eligible = False
    self._latched = False

  def __call__(self, started, params, CP) -> bool:
    if not started:
      self._started = False
      self._session_eligible = False
      self._latched = False
      return False

    if not self._started:
      self._session_eligible = bool(
        params is not None and params.get_bool("VisionObjectDetectionEnabled") and
        self._artifact_available_getter()
      )
    if self._session_eligible and getattr(CP, "brand", "") == "tesla":
      self._latched = True
    self._started = True
    return self._latched
