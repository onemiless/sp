from dataclasses import dataclass


SCHEMA_VERSION = 2
PUBLISH_HZ = 5.0
NORMAL_INFERENCE_HZ = 5.0
DEGRADED_INFERENCE_HZ = 2.0
MAX_OBJECTS = 32
MAX_RESULT_AGE_MS = 300.0
MAX_UI_FRAME_DELTA = 6
TRACK_MAX_MISSES = 2
TRACK_MAX_PREDICTION_S = 0.3
MODEL_INPUT_SIZE = (416, 416)

# YOLOX COCO indices mapped to stable schema class IDs.
COCO_CLASS_MAP = {
  0: 0,  # person
  1: 1,  # bicycle
  2: 2,  # car
  3: 3,  # motorcycle
  5: 4,  # bus
  7: 5,  # truck
}

CLASS_NAMES = ("person", "bicycle", "car", "motorcycle", "bus", "truck")


@dataclass(frozen=True)
class DetectorThresholds:
  confidence: float = 0.35
  nms_iou: float = 0.45
  max_objects: int = MAX_OBJECTS


@dataclass(frozen=True)
class ResourceThresholds:
  model_mean_ms: float = 40.0
  model_p95_ms: float = 60.0
  model_frame_drop_percent: float = 1.0
  object_p95_ms: float = 200.0
  memory_pause_percent: float = 90.0
  degrade_hold_s: float = 10.0
  recovery_hold_s: float = 15.0
