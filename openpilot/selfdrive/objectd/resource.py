from dataclasses import dataclass
from enum import IntEnum

from openpilot.selfdrive.objectd.constants import ResourceThresholds


class ResourceMode(IntEnum):
  NORMAL = 0
  DEGRADED = 1
  PAUSED = 2


@dataclass(frozen=True)
class ResourceSample:
  model_mean_ms: float = 0.0
  model_p95_ms: float = 0.0
  frame_drop_percent: float = 0.0
  object_p95_ms: float = 0.0
  memory_percent: float = 0.0
  thermal_ok: bool = True
  model_alive: bool = True


class ResourceGovernor:
  def __init__(self, thresholds: ResourceThresholds | None = None):
    self.thresholds = thresholds or ResourceThresholds()
    self.mode = ResourceMode.NORMAL
    self._pressure_since: float | None = None
    self._healthy_since: float | None = None

  def update(self, sample: ResourceSample, now_s: float) -> ResourceMode:
    immediate_pause = (not sample.thermal_ok or not sample.model_alive or
                       sample.memory_percent >= self.thresholds.memory_pause_percent)
    pressure = (sample.model_mean_ms > self.thresholds.model_mean_ms or
                sample.model_p95_ms > self.thresholds.model_p95_ms or
                sample.frame_drop_percent > self.thresholds.model_frame_drop_percent or
                sample.object_p95_ms > self.thresholds.object_p95_ms)

    if immediate_pause:
      self.mode = ResourceMode.PAUSED
      self._pressure_since = now_s
      self._healthy_since = None
      return self.mode

    if pressure:
      self._healthy_since = None
      if self._pressure_since is None:
        self._pressure_since = now_s
      if self.mode == ResourceMode.NORMAL:
        self.mode = ResourceMode.DEGRADED
      elif self.mode == ResourceMode.DEGRADED and now_s - self._pressure_since >= self.thresholds.degrade_hold_s:
        self.mode = ResourceMode.PAUSED
      return self.mode

    self._pressure_since = None
    if self.mode != ResourceMode.NORMAL:
      if self._healthy_since is None:
        self._healthy_since = now_s
      if now_s - self._healthy_since >= self.thresholds.recovery_hold_s:
        self.mode = ResourceMode.NORMAL
    return self.mode
