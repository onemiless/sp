#!/usr/bin/env python3
import queue
import time
import traceback

from msgq.visionipc import VisionIpcClient, VisionStreamType

from openpilot.selfdrive.objectd.constants import NORMAL_INFERENCE_HZ
from openpilot.selfdrive.objectd.model_runner import ObjectModelRunner
from openpilot.selfdrive.objectd.postprocess import decode_yolox
from openpilot.selfdrive.objectd.preprocess import letterbox_rgb
from openpilot.selfdrive.objectd.tracker import ShortTermTracker
from openpilot.system.camerad.snapshot import extract_image


ROAD_STREAM = VisionStreamType.VISION_STREAM_ROAD


def _put_latest(output_queue, message: dict) -> None:
  try:
    output_queue.put_nowait(message)
  except queue.Full:
    try:
      output_queue.get_nowait()
    except queue.Empty:
      pass
    output_queue.put_nowait(message)


def worker_main(output_queue, control_queue) -> None:
  try:
    runner = ObjectModelRunner()
  except Exception as exc:
    _put_latest(output_queue, {"kind": "error", "error": "modelLoad", "detail": str(exc)[:256]})
    return

  tracker = ShortTermTracker()
  client = VisionIpcClient("camerad", ROAD_STREAM, conflate=True)
  while not client.connect(False):
    _put_latest(output_queue, {"kind": "heartbeat", "mono_time": time.monotonic()})
    time.sleep(0.1)

  frequency_hz = NORMAL_INFERENCE_HZ
  last_inference = 0.0
  _put_latest(output_queue, {"kind": "ready", "mono_time": time.monotonic()})
  while True:
    try:
      while True:
        command = control_queue.get_nowait()
        if command.get("stop"):
          return
        frequency_hz = max(0.1, float(command.get("frequency_hz", frequency_hz)))
    except queue.Empty:
      pass

    frame = client.recv(timeout_ms=100)
    now = time.monotonic()
    if frame is None:
      _put_latest(output_queue, {"kind": "heartbeat", "mono_time": now})
      continue
    if now - last_inference < 1.0 / frequency_hz:
      continue

    started = time.perf_counter()
    try:
      rgb = extract_image(frame)
      tensor, transform = letterbox_rgb(rgb)
      output = runner.run(tensor)
      detections, dropped = decode_yolox(output, transform)
      source_timestamp_eof = int(client.timestamp_eof)
      tracks = tracker.update(detections, source_timestamp_eof)
      objects = [{
        "track_id": track.track_id,
        "class_id": track.class_id,
        "confidence": track.confidence,
        "bbox": track.bbox,
        "velocity": track.velocity,
        "prediction_valid": track.prediction_valid,
        "bbox_clipped": track.bbox_clipped,
      } for track in tracks]
      duration_ms = (time.perf_counter() - started) * 1000.0
      _put_latest(output_queue, {
        "kind": "result",
        "source_width": int(frame.width),
        "source_height": int(frame.height),
        "source_frame_id": int(client.frame_id),
        "source_timestamp_eof": source_timestamp_eof,
        "duration_ms": duration_ms,
        "objects": objects,
        "dropped": dropped,
        "mono_time": time.monotonic(),
      })
      last_inference = now
    except Exception as exc:
      _put_latest(output_queue, {
        "kind": "error", "error": "timeout", "detail": str(exc)[:256],
        "traceback": traceback.format_exc(limit=3)[:1024], "mono_time": time.monotonic(),
      })
      return


if __name__ == "__main__":
  raise SystemExit("objectd worker must be started by the supervisor using multiprocessing spawn")
