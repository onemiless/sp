from collections.abc import Callable


class VisionObjectSessionLatch:
  """Latch the ROAD detector decision once at the offroad-to-onroad edge."""

  def __init__(self, device_type_getter: Callable[[], str]):
    self._device_type_getter = device_type_getter
    self._started = False
    self._latched = False

  def __call__(self, started, params, CP) -> bool:
    if not started:
      self._started = False
      self._latched = False
      return False

    if not self._started:
      self._latched = bool(
        self._device_type_getter() == "tizi" and
        getattr(CP, "brand", "") == "tesla" and
        params is not None and params.get_bool("VisionObjectDetectionEnabled")
      )
    self._started = True
    return self._latched
