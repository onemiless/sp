from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.objectd.constants import TRACK_MAX_MISSES, TRACK_MAX_PREDICTION_S
from openpilot.selfdrive.objectd.postprocess import Detection, bbox_iou


@dataclass
class TrackedDetection:
  track_id: int
  class_id: int
  confidence: float
  bbox: tuple[float, float, float, float]
  velocity: tuple[float, float, float, float]
  prediction_valid: bool
  bbox_clipped: bool
  misses: int = 0
  timestamp_ns: int = 0


class ShortTermTracker:
  def __init__(self, iou_threshold: float = 0.25):
    self.iou_threshold = iou_threshold
    self._tracks: dict[int, TrackedDetection] = {}
    self._next_track_id = 1
    self._last_timestamp_ns: int | None = None

  def reset(self) -> None:
    self._tracks.clear()
    self._last_timestamp_ns = None

  def update(self, detections: list[Detection], timestamp_ns: int) -> list[TrackedDetection]:
    if timestamp_ns <= 0 or (self._last_timestamp_ns is not None and timestamp_ns <= self._last_timestamp_ns):
      self.reset()
    self._last_timestamp_ns = timestamp_ns

    unmatched_tracks = set(self._tracks)
    results: list[TrackedDetection] = []
    for detection in sorted(detections, key=lambda d: -d.confidence):
      best_id, best_iou = None, self.iou_threshold
      for track_id in unmatched_tracks:
        track = self._tracks[track_id]
        if track.class_id != detection.class_id:
          continue
        overlap = float(bbox_iou(np.asarray(detection.bbox), np.asarray(track.bbox)))
        if overlap >= best_iou:
          best_id, best_iou = track_id, overlap

      if best_id is None:
        track = TrackedDetection(self._next_track_id, detection.class_id, detection.confidence,
                                 detection.bbox, (0.0, 0.0, 0.0, 0.0), False,
                                 detection.bbox_clipped, timestamp_ns=timestamp_ns)
        self._next_track_id += 1
      else:
        previous = self._tracks[best_id]
        unmatched_tracks.remove(best_id)
        dt = (timestamp_ns - previous.timestamp_ns) / 1e9
        prediction_valid = 0.02 <= dt <= TRACK_MAX_PREDICTION_S
        velocity = tuple((np.asarray(detection.bbox) - np.asarray(previous.bbox)) / dt) if prediction_valid else (0.0,) * 4
        prediction_valid = prediction_valid and all(np.isfinite(velocity)) and max(abs(v) for v in velocity) <= 2.0
        track = TrackedDetection(best_id, detection.class_id, detection.confidence, detection.bbox,
                                 tuple(float(v) for v in velocity) if prediction_valid else (0.0,) * 4,
                                 prediction_valid, detection.bbox_clipped, timestamp_ns=timestamp_ns)
      self._tracks[track.track_id] = track
      results.append(track)

    for track_id in unmatched_tracks:
      track = self._tracks[track_id]
      track.misses += 1
      if track.misses > TRACK_MAX_MISSES:
        del self._tracks[track_id]

    return sorted(results, key=lambda t: (-t.confidence, t.track_id))
