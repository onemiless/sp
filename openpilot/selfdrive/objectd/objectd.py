#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass
import multiprocessing
import queue
import time

import numpy as np

from openpilot.cereal import log, messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.objectd.constants import (DEGRADED_INFERENCE_HZ, MAX_RESULT_AGE_MS,
                                                   NORMAL_INFERENCE_HZ, PUBLISH_HZ, SCHEMA_VERSION)
from openpilot.selfdrive.objectd.geometry import estimate_camera_ground_distance
from openpilot.selfdrive.objectd.model_runner import model_hash
from openpilot.selfdrive.objectd.resource import ResourceGovernor, ResourceMode, ResourceSample, object_duration_sample
from openpilot.selfdrive.objectd.supervisor import RetryController, model_service_available, worker_timed_out
from openpilot.selfdrive.objectd.worker import worker_main


WORKER_DEADLINE_S = 2.0
WORKER_STARTUP_DEADLINE_S = 30.0


@dataclass(frozen=True)
class CoarseCalibrationContext:
  intrinsics: np.ndarray
  rpy_calib: tuple[float, float, float]
  rpy_spread: tuple[float, float, float]
  camera_height_m: float


class WorkerController:
  def __init__(self):
    self.context = multiprocessing.get_context("spawn")
    self.output_queue = None
    self.control_queue = None
    self.process = None
    self.last_heartbeat_s = 0.0
    self.received_message = False

  def start(self) -> None:
    if self.process is not None:
      return
    # Queues are per-worker so a forced stop cannot leak a stale stop command or result into the replacement.
    self.output_queue = self.context.Queue(maxsize=2)
    self.control_queue = self.context.Queue(maxsize=2)
    self.process = self.context.Process(target=worker_main, args=(self.output_queue, self.control_queue), daemon=True)
    self.process.start()
    self.last_heartbeat_s = time.monotonic()
    self.received_message = False

  def stop(self) -> None:
    if self.process is None:
      return
    try:
      self.control_queue.put_nowait({"stop": True})
    except queue.Full:
      pass
    self.process.join(0.5)
    if self.process.is_alive():
      self.process.terminate()
      self.process.join(1.0)
    self.process = None

  def set_frequency(self, frequency_hz: float) -> None:
    if self.control_queue is None:
      return
    try:
      self.control_queue.put_nowait({"frequency_hz": frequency_hz})
    except queue.Full:
      pass

  def drain(self) -> list[dict]:
    messages = []
    if self.output_queue is None:
      return messages
    try:
      while True:
        message = self.output_queue.get_nowait()
        messages.append(message)
        self.last_heartbeat_s = time.monotonic()
        self.received_message = True
    except queue.Empty:
      return messages

  @property
  def alive(self) -> bool:
    return self.process is not None and self.process.is_alive()


def _percentile(values: deque[float], percentile: float) -> float:
  return float(np.percentile(values, percentile)) if values else 0.0


def _coarse_calibration_context(sm, source_width: int, source_height: int) -> CoarseCalibrationContext | None:
  required = ("deviceState", "liveCalibration", "roadCameraState")
  if any(not sm.seen[name] or not sm.valid[name] or not sm.alive[name] for name in required):
    return None
  calibration = sm["liveCalibration"]
  if str(calibration.calStatus) != "calibrated":
    return None
  try:
    rpy_calib = tuple(float(value) for value in calibration.rpyCalib)
    rpy_spread = tuple(float(value) for value in calibration.rpyCalibSpread)
    camera_height = tuple(float(value) for value in calibration.height)
    camera_config = DEVICE_CAMERAS[(str(sm["deviceState"].deviceType), str(sm["roadCameraState"].sensor))].fcam
  except (KeyError, TypeError, ValueError):
    return None
  if len(rpy_calib) != 3 or len(rpy_spread) != 3 or len(camera_height) != 1:
    return None
  if camera_config.size != (source_width, source_height):
    return None
  values = (*rpy_calib, *rpy_spread, camera_height[0], *camera_config.intrinsics.flat)
  if not np.all(np.isfinite(values)):
    return None
  return CoarseCalibrationContext(camera_config.intrinsics, rpy_calib, rpy_spread, camera_height[0])


def _object_payload(track: dict, source_size: tuple[int, int],
                    calibration: CoarseCalibrationContext | None) -> dict:
  bbox = [float(v) for v in track["bbox"]]
  velocity = [float(v) for v in track["velocity"]]
  contact = ((bbox[0] + bbox[2]) / 2.0, bbox[3])
  contact_valid = bool(not track["bbox_clipped"] and 0.0 < contact[0] < 1.0 and 0.0 < contact[1] < 1.0)
  estimate = None
  if calibration is not None and contact_valid:
    contact_std = max(2.0 / source_size[1], 0.05 * (bbox[3] - bbox[1]))
    estimate = estimate_camera_ground_distance(contact, source_size, calibration.intrinsics,
                                               calibration.rpy_calib, calibration.rpy_spread,
                                               calibration.camera_height_m, contact_std)
  distance_valid = bool(estimate is not None and estimate.valid)
  position = estimate.position if distance_valid else (0.0, 0.0, 0.0)
  distance = estimate.distance_m if distance_valid else 0.0
  distance_std = estimate.distance_std_m if distance_valid else 0.0
  range_band = "unknown" if not distance_valid else ("near" if distance < 10.0 else "medium" if distance < 20.0 else "far")
  return {
    "trackId": int(track["track_id"]),
    "classId": int(track["class_id"]),
    "confidence": float(track["confidence"]),
    "bboxNormalized": {"left": bbox[0], "top": bbox[1], "right": bbox[2], "bottom": bbox[3]},
    "bboxVelocityNormalized": {
      "leftPerSecond": velocity[0], "topPerSecond": velocity[1],
      "rightPerSecond": velocity[2], "bottomPerSecond": velocity[3],
    },
    "bboxPredictionValid": bool(track["prediction_valid"]),
    "bboxClipped": bool(track["bbox_clipped"]),
    "contactU": contact[0],
    "contactV": contact[1],
    "contactPointValid": contact_valid,
    "x": position[0], "y": position[1], "z": position[2],
    "positionStd": {"x": distance_std, "y": distance_std, "z": 0.0},
    "distance": distance, "distanceStd": distance_std, "distanceValid": distance_valid,
    "rangeBand": range_band,
    "relativeSpeed": 0.0, "relativeSpeedValid": False,
    "ttc": 0.0, "ttcValid": False,
    "corridorState": "unknown",
  }


def main() -> None:
  cloudlog.warning("objectd supervisor init: ROAD-only, display-only")
  pm = messaging.PubMaster(["visionObjectStateSP"])
  sm = messaging.SubMaster(["deviceState", "liveCalibration", "modelV2", "roadCameraState"])
  rk = Ratekeeper(PUBLISH_HZ, print_delay_threshold=None)
  worker = WorkerController()
  retry = RetryController()
  governor = ResourceGovernor()
  model_times_ms: deque[float] = deque(maxlen=150)
  object_times_ms: deque[float] = deque(maxlen=150)
  last_result = None
  error_code = "none"
  inference_updated = False
  worker.start()

  try:
    while True:
      now_s = time.monotonic()
      now_ns = time.monotonic_ns()
      sm.update(0)
      inference_updated = False

      for worker_message in worker.drain():
        if worker_message["kind"] == "result":
          last_result = worker_message
          duration_sample = object_duration_sample(worker_message)
          if duration_sample is not None:
            object_times_ms.append(duration_sample)
          error_code = "none"
          inference_updated = True
        elif worker_message["kind"] == "error":
          error_code = worker_message.get("error", "timeout")
          cloudlog.error(f"objectd worker error: {worker_message.get('detail', '')}")

      if sm.updated["modelV2"]:
        model_times_ms.append(float(sm["modelV2"].modelExecutionTime) * 1000.0)
      device = sm["deviceState"]
      resource_sample = ResourceSample(
        model_mean_ms=float(np.mean(model_times_ms)) if model_times_ms else 0.0,
        model_p95_ms=_percentile(model_times_ms, 95),
        frame_drop_percent=float(sm["modelV2"].frameDropPerc) if sm.seen["modelV2"] else 0.0,
        object_p95_ms=_percentile(object_times_ms, 95),
        memory_percent=float(device.memoryUsagePercent) if sm.seen["deviceState"] else 0.0,
        thermal_ok=(not sm.seen["deviceState"] or device.thermalStatus < log.DeviceState.ThermalStatus.overheated),
        model_alive=model_service_available(sm.seen["modelV2"], sm.alive["modelV2"]),
      )
      mode = governor.update(resource_sample, now_s)

      worker_failed = worker.process is not None and not worker.alive
      deadline_exceeded = worker.alive and worker_timed_out(
        worker.last_heartbeat_s, worker.received_message, now_s, WORKER_DEADLINE_S, WORKER_STARTUP_DEADLINE_S,
      )
      if worker_failed or deadline_exceeded:
        worker.stop()
        retry.record_failure(now_s)
        last_result = None
        error_code = "circuitBreaker" if retry.fused else (error_code if error_code != "none" else "timeout")

      if mode == ResourceMode.PAUSED:
        worker.stop()
        last_result = None
        if not resource_sample.thermal_ok:
          error_code = "thermal"
        elif resource_sample.memory_percent >= 90.0:
          error_code = "memory"
        elif not resource_sample.model_alive:
          error_code = "timeout"
      elif not worker.alive and retry.can_start(now_s):
        worker.start()
      elif worker.alive:
        worker.set_frequency(NORMAL_INFERENCE_HZ if mode == ResourceMode.NORMAL else DEGRADED_INFERENCE_HZ)

      result_age_ms = (now_ns - int(last_result["source_timestamp_eof"])) / 1e6 if last_result else float("inf")
      normal_overlay_rate = mode == ResourceMode.NORMAL
      result_valid = bool(last_result and normal_overlay_rate and result_age_ms <= MAX_RESULT_AGE_MS and error_code == "none")
      if last_result and result_age_ms > MAX_RESULT_AGE_MS:
        error_code = "stale"
      debug_degraded_objects = bool(last_result and mode == ResourceMode.DEGRADED and result_age_ms <= MAX_RESULT_AGE_MS)
      source_size = ((int(last_result["source_width"]), int(last_result["source_height"])) if last_result else (0, 0))
      calibration = _coarse_calibration_context(sm, *source_size) if result_valid else None
      objects = ([_object_payload(obj, source_size, calibration) for obj in last_result["objects"]]
                 if (result_valid or debug_degraded_objects) else [])
      coarse_distance_available = bool(calibration is not None and any(obj["distanceValid"] for obj in objects))

      msg = messaging.new_message("visionObjectStateSP", valid=True)
      state = msg.visionObjectStateSP
      state.schemaVersion = SCHEMA_VERSION
      state.state = "sessionFused" if retry.fused else ("running" if result_valid else "degraded")
      state.streamType = "road"
      state.sourceWidth = int(last_result["source_width"]) if last_result else 0
      state.sourceHeight = int(last_result["source_height"]) if last_result else 0
      state.sourceFrameId = int(last_result["source_frame_id"]) if last_result else 0
      state.sourceTimestampEof = int(last_result["source_timestamp_eof"]) if last_result else 0
      state.publishMonoTime = now_ns
      state.inferenceUpdated = inference_updated
      state.inferenceDurationMs = float(last_result["duration_ms"]) if last_result else 0.0
      state.resultAgeMs = float(result_age_ms) if np.isfinite(result_age_ms) else 0.0
      state.resultValid = result_valid
      state.modelHash = model_hash()
      state.positionOrigin = "cameraGroundCenter"
      state.vehicleExtrinsicsValid = False
      state.roadPlaneValid = False
      state.distanceMode = "coarse" if coarse_distance_available else "invalid"
      state.objects = objects
      state.droppedObjectCount = int(last_result["dropped"]) if last_result else 0
      state.errorCode = error_code
      state.inferenceFrequencyHz = NORMAL_INFERENCE_HZ if mode == ResourceMode.NORMAL else (DEGRADED_INFERENCE_HZ if mode == ResourceMode.DEGRADED else 0.0)
      pm.send("visionObjectStateSP", msg)
      rk.keep_time()
  finally:
    worker.stop()


if __name__ == "__main__":
  main()
