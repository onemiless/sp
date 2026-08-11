import json
import pickle
from pathlib import Path

import numpy as np

from openpilot.common.file_chunker import get_existing_chunks, open_file_chunked


OBJECTD_DIR = Path(__file__).resolve().parent
MODELS_DIR = OBJECTD_DIR / "models"
MANIFEST_PATH = MODELS_DIR / "object_detector_manifest.json"
MODEL_PKL_PATH = MODELS_DIR / "object_detector_tinygrad.pkl"
MODEL_COMPILE_STAMP_PATH = MODELS_DIR / "object_detector_tinygrad.sha256"


def model_artifact_available(model_path: Path = MODEL_PKL_PATH, manifest_path: Path = MANIFEST_PATH,
                             stamp_path: Path = MODEL_COMPILE_STAMP_PATH) -> bool:
  try:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_hash = str(manifest["model"]["onnxSha256"]).lower()
    compiled_hash = stamp_path.read_text(encoding="utf-8").strip().lower()
    chunks_exist = all(Path(path).is_file() for path in get_existing_chunks(str(model_path)))
    return len(expected_hash) == 64 and compiled_hash == expected_hash and chunks_exist
  except (FileNotFoundError, KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
    return False


def model_hash() -> str:
  manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
  return manifest["model"]["onnxSha256"][:12]


class ObjectModelRunner:
  def __init__(self):
    from tinygrad import Device, Tensor

    self._device = Device.DEFAULT
    self._tensor = Tensor
    with open_file_chunked(str(MODEL_PKL_PATH)) as stream:
      self._run = pickle.load(stream)
    self._manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

  def run(self, input_tensor: np.ndarray) -> np.ndarray:
    expected = tuple(self._manifest["input"]["shape"])
    if input_tensor.shape != expected or input_tensor.dtype != np.float32:
      raise ValueError(f"expected float32 input {expected}, got {input_tensor.dtype} {input_tensor.shape}")
    tensor = self._tensor(input_tensor, device=self._device).realize()
    output = self._run(images=tensor)
    result = output.numpy()
    expected_output = tuple(self._manifest["output"]["shape"])
    if result.shape != expected_output or not np.isfinite(result).all():
      raise RuntimeError(f"invalid model output: expected finite {expected_output}, got {result.shape}")
    return result
