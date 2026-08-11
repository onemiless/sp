from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.objectd.constants import COCO_CLASS_MAP, DetectorThresholds
from openpilot.selfdrive.objectd.preprocess import LetterboxTransform, invert_letterbox_xyxy


@dataclass(frozen=True)
class Detection:
  class_id: int
  confidence: float
  bbox: tuple[float, float, float, float]
  bbox_clipped: bool = False


def decode_yolox_grid(output: np.ndarray, input_size: tuple[int, int],
                      strides: tuple[int, ...] = (8, 16, 32)) -> np.ndarray:
  """Decode the raw YOLOX release head into input-image xywh coordinates."""
  predictions = np.asarray(output, dtype=np.float32)
  if predictions.ndim not in (2, 3) or predictions.shape[-1] < 4:
    raise ValueError("YOLOX output must have shape [N, values] or [batch, N, values]")

  input_width, input_height = input_size
  if input_width <= 0 or input_height <= 0:
    raise ValueError("input dimensions must be positive")

  grids = []
  expanded_strides = []
  for stride in strides:
    if input_width % stride or input_height % stride:
      raise ValueError(f"input size {input_size} is not divisible by stride {stride}")
    width, height = input_width // stride, input_height // stride
    grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
    grid = np.stack((grid_x, grid_y), axis=2).reshape(-1, 2)
    grids.append(grid)
    expanded_strides.append(np.full((len(grid), 1), stride, dtype=np.float32))

  grid = np.concatenate(grids).astype(np.float32, copy=False)
  expanded_stride = np.concatenate(expanded_strides)
  if predictions.shape[-2] != len(grid):
    raise ValueError(f"YOLOX output has {predictions.shape[-2]} anchors, expected {len(grid)} for {input_size}")

  decoded = predictions.copy()
  decoded[..., :2] = (decoded[..., :2] + grid) * expanded_stride
  decoded[..., 2:4] = np.exp(decoded[..., 2:4]) * expanded_stride
  return decoded


def bbox_iou(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  a = np.asarray(a, dtype=np.float32)
  b = np.asarray(b, dtype=np.float32)
  top_left = np.maximum(a[..., :2], b[..., :2])
  bottom_right = np.minimum(a[..., 2:], b[..., 2:])
  intersection = np.prod(np.maximum(0.0, bottom_right - top_left), axis=-1)
  area_a = np.prod(np.maximum(0.0, a[..., 2:] - a[..., :2]), axis=-1)
  area_b = np.prod(np.maximum(0.0, b[..., 2:] - b[..., :2]), axis=-1)
  return intersection / np.maximum(area_a + area_b - intersection, 1e-9)


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
  order = np.argsort(-scores, kind="stable")
  keep: list[int] = []
  while order.size:
    current = int(order[0])
    keep.append(current)
    if order.size == 1:
      break
    overlaps = bbox_iou(boxes[order[1:]], boxes[current])
    order = order[1:][overlaps <= iou_threshold]
  return keep


def decode_yolox(output: np.ndarray, transform: LetterboxTransform,
                 thresholds: DetectorThresholds | None = None) -> tuple[list[Detection], int]:
  thresholds = thresholds or DetectorThresholds()
  predictions = decode_yolox_grid(output, (transform.input_width, transform.input_height))
  if predictions.ndim == 3 and predictions.shape[0] == 1:
    predictions = predictions[0]
  if predictions.ndim != 2 or predictions.shape[1] < 6:
    raise ValueError("YOLOX output must have shape [N, 5 + classes]")

  class_scores = predictions[:, 5:]
  original_classes = np.argmax(class_scores, axis=1)
  confidence = predictions[:, 4] * class_scores[np.arange(len(predictions)), original_classes]
  allowed = np.array([int(c) in COCO_CLASS_MAP for c in original_classes], dtype=bool)
  selected = allowed & np.isfinite(confidence) & (confidence >= thresholds.confidence)
  if not np.any(selected):
    return [], 0

  rows = predictions[selected]
  classes = original_classes[selected]
  scores = confidence[selected]
  xyxy = np.empty((len(rows), 4), dtype=np.float32)
  xyxy[:, 0] = rows[:, 0] - rows[:, 2] / 2
  xyxy[:, 1] = rows[:, 1] - rows[:, 3] / 2
  xyxy[:, 2] = rows[:, 0] + rows[:, 2] / 2
  xyxy[:, 3] = rows[:, 1] + rows[:, 3] / 2
  normalized = invert_letterbox_xyxy(xyxy, transform)

  valid_shape = (normalized[:, 2] > normalized[:, 0]) & (normalized[:, 3] > normalized[:, 1])
  normalized, classes, scores, xyxy = normalized[valid_shape], classes[valid_shape], scores[valid_shape], xyxy[valid_shape]
  keep: list[int] = []
  for class_index in np.unique(classes):
    indices = np.flatnonzero(classes == class_index)
    keep.extend(indices[i] for i in _nms(normalized[indices], scores[indices], thresholds.nms_iou))
  keep.sort(key=lambda i: (-float(scores[i]), int(classes[i]), i))

  dropped = max(0, len(keep) - thresholds.max_objects)
  keep = keep[:thresholds.max_objects]
  detections = []
  for i in keep:
    clipped = bool(np.any(xyxy[i] < 0) or xyxy[i, 2] > transform.input_width or xyxy[i, 3] > transform.input_height)
    detections.append(Detection(COCO_CLASS_MAP[int(classes[i])], float(scores[i]),
                                tuple(float(v) for v in normalized[i]), clipped))
  return detections, dropped
