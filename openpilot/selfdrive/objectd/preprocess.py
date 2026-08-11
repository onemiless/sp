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


def _letterbox_geometry(source_width: int, source_height: int, input_size: tuple[int, int]) -> tuple[int, int, int, int, float]:
  input_width, input_height = input_size
  if source_width <= 0 or source_height <= 0:
    raise ValueError("source dimensions must be positive")
  if input_width <= 0 or input_height <= 0:
    raise ValueError("input dimensions must be positive")

  scale = min(input_width / source_width, input_height / source_height)
  resized_width = max(1, min(input_width, int(round(source_width * scale))))
  resized_height = max(1, min(input_height, int(round(source_height * scale))))
  pad_x = (input_width - resized_width) // 2
  pad_y = (input_height - resized_height) // 2
  return resized_width, resized_height, pad_x, pad_y, scale


def _rgb_to_model_tensor(rgb: np.ndarray, input_size: tuple[int, int], pad_x: int, pad_y: int,
                         pad_value: int) -> np.ndarray:
  input_width, input_height = input_size
  padded = np.full((input_height, input_width, 3), pad_value, dtype=np.uint8)
  padded[pad_y:pad_y + rgb.shape[0], pad_x:pad_x + rgb.shape[1]] = rgb
  # Official YOLOX ONNX weights consume unnormalized BGR CHW float32 input.
  tensor = padded[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32, copy=False)
  return np.ascontiguousarray(tensor)


def letterbox_rgb(image: np.ndarray, input_size: tuple[int, int] = MODEL_INPUT_SIZE,
                  pad_value: int = 114) -> tuple[np.ndarray, LetterboxTransform]:
  if image.ndim != 3 or image.shape[2] != 3 or image.shape[0] <= 0 or image.shape[1] <= 0:
    raise ValueError("image must be a non-empty HxWx3 RGB array")

  input_width, input_height = input_size
  if input_width <= 0 or input_height <= 0:
    raise ValueError("input dimensions must be positive")

  source_height, source_width = image.shape[:2]
  resized_width, resized_height, pad_x, pad_y, scale = _letterbox_geometry(source_width, source_height, input_size)
  resized = _resize_nearest(image, resized_width, resized_height)

  tensor = _rgb_to_model_tensor(resized, input_size, pad_x, pad_y, pad_value)
  transform = LetterboxTransform(source_width, source_height, input_width, input_height,
                                 scale, float(pad_x), float(pad_y))
  return tensor, transform


def letterbox_nv12(buf, input_size: tuple[int, int] = MODEL_INPUT_SIZE,
                   pad_value: int = 114) -> tuple[np.ndarray, LetterboxTransform]:
  """Sample a VisionIPC NV12 frame at model resolution before RGB conversion."""
  source_width, source_height = int(buf.width), int(buf.height)
  stride, uv_offset = int(buf.stride), int(buf.uv_offset)
  input_width, input_height = input_size
  if source_width % 2 or source_height % 2 or stride < source_width or uv_offset < stride * source_height:
    raise ValueError("invalid NV12 VisionIPC buffer geometry")

  resized_width, resized_height, pad_x, pad_y, scale = _letterbox_geometry(source_width, source_height, input_size)
  raw = np.frombuffer(buf.data, dtype=np.uint8)
  if raw.size < uv_offset + stride * (source_height // 2):
    raise ValueError("NV12 VisionIPC buffer is truncated")
  y_plane = raw[:uv_offset].reshape((-1, stride))
  uv_plane = raw[uv_offset:].reshape((-1, stride))

  ys = np.minimum((np.arange(resized_height) * source_height / resized_height).astype(np.int64), source_height - 1)
  xs = np.minimum((np.arange(resized_width) * source_width / resized_width).astype(np.int64), source_width - 1)
  y = y_plane[ys[:, None], xs[None, :]]
  uv_rows = (ys // 2)[:, None]
  uv_cols = ((xs // 2) * 2)[None, :]
  u = uv_plane[uv_rows, uv_cols]
  v = uv_plane[uv_rows, uv_cols + 1]

  yuv = np.stack((y, u, v), axis=2).astype(np.int16)
  yuv[:, :, 1:] -= 128
  conversion = np.array([
    [1.00000, 1.00000, 1.00000],
    [0.00000, -0.39465, 2.03211],
    [1.13983, -0.58060, 0.00000],
  ])
  rgb = np.dot(yuv, conversion).clip(0, 255).astype(np.uint8)
  tensor = _rgb_to_model_tensor(rgb, input_size, pad_x, pad_y, pad_value)
  transform = LetterboxTransform(source_width, source_height, input_width, input_height,
                                 scale, float(pad_x), float(pad_y))
  return tensor, transform


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
