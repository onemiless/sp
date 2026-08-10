from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.objectd.constants import MODEL_INPUT_SIZE


@dataclass(frozen=True)
class LetterboxTransform:
  source_width: int
  source_height: int
  input_width: int
  input_height: int
  scale: float
  pad_x: float
  pad_y: float


def _resize_nearest(image: np.ndarray, width: int, height: int) -> np.ndarray:
  ys = np.minimum((np.arange(height) * image.shape[0] / height).astype(np.int64), image.shape[0] - 1)
  xs = np.minimum((np.arange(width) * image.shape[1] / width).astype(np.int64), image.shape[1] - 1)
  return image[ys[:, None], xs[None, :]]


def letterbox_rgb(image: np.ndarray, input_size: tuple[int, int] = MODEL_INPUT_SIZE,
                  pad_value: int = 114) -> tuple[np.ndarray, LetterboxTransform]:
  if image.ndim != 3 or image.shape[2] != 3 or image.shape[0] <= 0 or image.shape[1] <= 0:
    raise ValueError("image must be a non-empty HxWx3 RGB array")

  input_width, input_height = input_size
  if input_width <= 0 or input_height <= 0:
    raise ValueError("input dimensions must be positive")

  source_height, source_width = image.shape[:2]
  scale = min(input_width / source_width, input_height / source_height)
  resized_width = max(1, min(input_width, int(round(source_width * scale))))
  resized_height = max(1, min(input_height, int(round(source_height * scale))))
  resized = _resize_nearest(image, resized_width, resized_height)

  pad_x = (input_width - resized_width) // 2
  pad_y = (input_height - resized_height) // 2
  padded = np.full((input_height, input_width, 3), pad_value, dtype=np.uint8)
  padded[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized

  # Official YOLOX ONNX weights consume unnormalized BGR CHW float32 input.
  tensor = padded[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32, copy=False)
  transform = LetterboxTransform(source_width, source_height, input_width, input_height,
                                 scale, float(pad_x), float(pad_y))
  return np.ascontiguousarray(tensor), transform


def invert_letterbox_xyxy(boxes: np.ndarray, transform: LetterboxTransform) -> np.ndarray:
  boxes = np.asarray(boxes, dtype=np.float32)
  if boxes.ndim != 2 or boxes.shape[1] != 4:
    raise ValueError("boxes must have shape Nx4")

  restored = boxes.copy()
  restored[:, [0, 2]] = (restored[:, [0, 2]] - transform.pad_x) / transform.scale
  restored[:, [1, 3]] = (restored[:, [1, 3]] - transform.pad_y) / transform.scale
  restored[:, [0, 2]] = np.clip(restored[:, [0, 2]], 0, transform.source_width)
  restored[:, [1, 3]] = np.clip(restored[:, [1, 3]], 0, transform.source_height)
  restored[:, [0, 2]] /= transform.source_width
  restored[:, [1, 3]] /= transform.source_height
  return restored
