from collections.abc import Mapping
import os
from pathlib import Path


SOURCE_MODELS_DIR = Path(__file__).resolve().parent / "models"
DEVICE_MODELS_DIR = Path("/data/models/objectd")


def object_model_root(device: bool | None = None, environ: Mapping[str, str] | None = None) -> Path:
  """Return a model artifact directory that survives updater's checkout clean."""
  environment = os.environ if environ is None else environ
  if override := environment.get("OBJECTD_MODEL_ROOT"):
    return Path(override)
  if device is None:
    device = Path("/TICI").is_file()
  return DEVICE_MODELS_DIR if device else SOURCE_MODELS_DIR
