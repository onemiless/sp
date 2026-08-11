import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from openpilot.selfdrive.objectd.constants import DetectorThresholds
from openpilot.selfdrive.objectd.geometry import (DistanceGateInput, DistanceInvalidReason, estimate_camera_ground_distance,
                                                 gate_metric_distance, intersect_local_ground)
from openpilot.selfdrive.objectd.model_runner import model_artifact_available
from openpilot.selfdrive.objectd.postprocess import Detection, decode_yolox, decode_yolox_grid
from openpilot.selfdrive.objectd.preprocess import invert_letterbox_xyxy, letterbox_nv12, letterbox_rgb
from openpilot.selfdrive.objectd.resource import ResourceGovernor, ResourceMode, ResourceSample, object_duration_sample
from openpilot.selfdrive.objectd.road_plane import fit_road_plane
from openpilot.selfdrive.objectd.tracker import ShortTermTracker


class TestPreprocess(unittest.TestCase):
  def test_nv12_letterbox_matches_full_rgb_conversion(self):
    class FakeVisionBuf:
      width = 8
      height = 4
      stride = 10
      uv_offset = stride * height

    buf = FakeVisionBuf()
    uv_height = 16
    raw = np.zeros(buf.uv_offset + buf.stride * uv_height, dtype=np.uint8)
    y = raw[:buf.uv_offset].reshape(buf.height, buf.stride)
    y[:, :buf.width] = np.arange(buf.height * buf.width, dtype=np.uint8).reshape(buf.height, buf.width) + 80
    uv = raw[buf.uv_offset:].reshape(uv_height, buf.stride)
    uv[:buf.height // 2, :buf.width:2] = [[90, 100, 110, 120], [130, 140, 150, 160]]
    uv[:buf.height // 2, 1:buf.width:2] = [[170, 160, 150, 140], [130, 120, 110, 100]]
    buf.data = memoryview(raw)

    from openpilot.system.camerad.snapshot import extract_image
    expected, expected_transform = letterbox_rgb(extract_image(buf), (32, 32))
    actual, actual_transform = letterbox_nv12(buf, (32, 32))

    np.testing.assert_array_equal(actual, expected)
    self.assertEqual(actual_transform, expected_transform)

  def test_letterbox_round_trip_non_square(self):
    image = np.zeros((120, 200, 3), dtype=np.uint8)
    tensor, transform = letterbox_rgb(image)
    self.assertEqual(tensor.shape, (1, 3, 416, 416))
    source_box = np.array([[20.0, 10.0, 180.0, 110.0]], dtype=np.float32)
    model_box = source_box.copy()
    model_box[:, [0, 2]] = model_box[:, [0, 2]] * transform.scale + transform.pad_x
    model_box[:, [1, 3]] = model_box[:, [1, 3]] * transform.scale + transform.pad_y
    restored = invert_letterbox_xyxy(model_box, transform)
    np.testing.assert_allclose(restored[0], [0.1, 10 / 120, 0.9, 110 / 120], atol=1e-6)

  def test_invalid_image_rejected(self):
    with self.assertRaises(ValueError):
      letterbox_rgb(np.zeros((0, 1, 3), dtype=np.uint8))


class TestPostprocess(unittest.TestCase):
  def test_release_head_grid_stride_decode(self):
    raw = np.zeros((1, 3549, 85), dtype=np.float32)
    raw[0, 0, :4] = [1.0, 2.0, np.log(2.0), np.log(3.0)]
    decoded = decode_yolox_grid(raw, (416, 416))
    np.testing.assert_allclose(decoded[0, 0, :4], [8.0, 16.0, 16.0, 24.0], atol=1e-6)
    np.testing.assert_allclose(raw[0, 0, :4], [1.0, 2.0, np.log(2.0), np.log(3.0)], atol=1e-6)

  def test_allowed_classes_nms_and_limit(self):
    _, transform = letterbox_rgb(np.zeros((416, 416, 3), dtype=np.uint8))
    output = np.zeros((3549, 85), dtype=np.float32)
    output[0, :5] = [100 / 8, 100 / 8, np.log(50 / 8), np.log(50 / 8), 0.9]
    output[0, 7] = 0.9  # COCO car class index 2
    output[1, :5] = [102 / 8 - 1, 102 / 8, np.log(50 / 8), np.log(50 / 8), 0.8]
    output[1, 7] = 0.9
    output[2, :5] = [300 / 8 - 2, 300 / 8, np.log(40 / 8), np.log(60 / 8), 0.95]
    output[2, 5] = 0.8  # person
    output[3, :5] = [200 / 8 - 3, 200 / 8, np.log(20 / 8), np.log(20 / 8), 0.99]
    output[3, 9] = 0.99  # unsupported COCO class 4
    detections, dropped = decode_yolox(output, transform, DetectorThresholds(0.35, 0.45, 32))
    self.assertEqual(dropped, 0)
    self.assertEqual([d.class_id for d in detections], [2, 0])
    np.testing.assert_allclose(detections[0].bbox, [75 / 416, 75 / 416, 125 / 416, 125 / 416], atol=1e-6)


class TestTracker(unittest.TestCase):
  def test_track_velocity_and_timestamp_reset(self):
    tracker = ShortTermTracker()
    first = tracker.update([Detection(2, 0.9, (0.1, 0.1, 0.3, 0.3))], 1_000_000_000)[0]
    second = tracker.update([Detection(2, 0.9, (0.11, 0.1, 0.31, 0.3))], 1_100_000_000)[0]
    self.assertEqual(first.track_id, second.track_id)
    self.assertTrue(second.prediction_valid)
    self.assertAlmostEqual(second.velocity[0], 0.1, places=5)
    reset = tracker.update([Detection(2, 0.9, (0.11, 0.1, 0.31, 0.3))], 1_000_000_000)[0]
    self.assertNotEqual(second.track_id, reset.track_id)
    self.assertFalse(reset.prediction_valid)

  def test_tracker_accepts_three_hz_interval(self):
    tracker = ShortTermTracker()
    tracker.update([Detection(2, 0.9, (0.1, 0.1, 0.3, 0.3))], 1_000_000_000)
    updated = tracker.update([Detection(2, 0.9, (0.11, 0.1, 0.31, 0.3))], 1_340_000_000)[0]
    self.assertTrue(updated.prediction_valid)


class TestDistanceGate(unittest.TestCase):
  def test_sp_calibration_camera_ground_projection(self):
    from openpilot.common.transformations.camera import get_view_frame_from_road_frame

    width, height = 1928, 1208
    intrinsics = np.array([[2648.0, 0.0, width / 2], [0.0, 2648.0, height / 2], [0.0, 0.0, 1.0]])
    view_from_road = get_view_frame_from_road_frame(0.0, 0.0, 0.0, 1.28)
    expected = np.array([10.0, 2.0, 0.0, 1.0])
    projected = intrinsics @ (view_from_road @ expected)
    contact = (projected[0] / projected[2] / width, projected[1] / projected[2] / height)

    result = estimate_camera_ground_distance(contact, (width, height), intrinsics, (0.0, 0.0, 0.0),
                                             (0.0, 0.0, 0.0), 1.28, 1.0 / height)
    self.assertTrue(result.valid)
    np.testing.assert_allclose(result.position, expected[:3], atol=1e-6)
    self.assertAlmostEqual(result.distance_m, np.hypot(10.0, 2.0), places=5)

    far = np.array([30.0, 0.0, 0.0, 1.0])
    projected_far = intrinsics @ (view_from_road @ far)
    far_contact = (projected_far[0] / projected_far[2] / width, projected_far[1] / projected_far[2] / height)
    far_result = estimate_camera_ground_distance(far_contact, (width, height), intrinsics, (0.0, 0.0, 0.0),
                                                (0.0, 0.0, 0.0), 1.28, 1.0 / height)
    self.assertTrue(far_result.valid)
    self.assertEqual(far_result.distance_m, 30.0)

  def test_coarse_distance_rejects_horizon_and_large_uncertainty(self):
    intrinsics = np.array([[1000.0, 0.0, 500.0], [0.0, 1000.0, 300.0], [0.0, 0.0, 1.0]])
    horizon = estimate_camera_ground_distance((0.5, 0.5), (1000, 600), intrinsics,
                                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.2, 0.001)
    self.assertFalse(horizon.valid)

    uncertain = estimate_camera_ground_distance((0.5, 0.54), (1000, 600), intrinsics,
                                                (0.0, 0.0, 0.0), (0.0, 0.1, 0.0), 1.2, 0.02)
    self.assertFalse(uncertain.valid)

  def test_missing_extrinsics_fails_closed(self):
    result = gate_metric_distance(DistanceGateInput(result_age_ms=0.0))
    self.assertFalse(result.valid)
    self.assertEqual(result.reason, DistanceInvalidReason.EXTRINSICS)
    self.assertEqual(result.distance_m, 0.0)

  def test_valid_input(self):
    result = gate_metric_distance(DistanceGateInput(
      True, True, True, True, 0.1, 2.0, 10.0, 0.5, 100.0,
    ))
    self.assertTrue(result.valid)
    self.assertEqual(result.distance_m, 10.0)

  def test_camera_road_to_front_bumper_numeric_contract(self):
    origin = np.array([0.0, 0.0, 1.4])
    direction = np.array([11.5, 0.0, -1.4])
    point = intersect_local_ground(origin, direction, np.array([1.5, 0.0, 0.0]))
    np.testing.assert_allclose(point, [10.0, 0.0, 0.0], atol=1e-8)

  def test_parallel_or_upward_ray_invalid(self):
    self.assertIsNone(intersect_local_ground(np.array([0.0, 0.0, 1.4]), np.array([1.0, 0.0, 0.0]), np.zeros(3)))


class TestRoadPlane(unittest.TestCase):
  def test_two_consistent_curves_stable(self):
    x = np.linspace(0.0, 30.0, 20)
    left = np.column_stack((x, np.full_like(x, 1.5), 0.01 * x))
    right = np.column_stack((x, np.full_like(x, -1.5), 0.01 * x))
    result = fit_road_plane([left, right])
    self.assertTrue(result.stable)
    self.assertAlmostEqual(result.coefficients[0], 0.01, places=6)

  def test_steep_or_single_curve_invalid(self):
    x = np.linspace(0.0, 30.0, 20)
    steep = np.column_stack((x, np.zeros_like(x), 0.03 * x))
    self.assertFalse(fit_road_plane([steep]).stable)
    self.assertFalse(fit_road_plane([steep, steep.copy()]).stable)


class TestResourceGovernor(unittest.TestCase):
  def test_cold_start_duration_is_not_resource_pressure(self):
    self.assertIsNone(object_duration_sample({"kind": "result", "duration_ms": 2000.0, "warmup": True}))
    self.assertEqual(object_duration_sample({"kind": "result", "duration_ms": 75.0, "warmup": False}), 75.0)

  def test_degrade_then_pause(self):
    governor = ResourceGovernor()
    pressure = ResourceSample(model_p95_ms=70.0)
    self.assertEqual(governor.update(pressure, 0.0), ResourceMode.DEGRADED)
    self.assertEqual(governor.update(pressure, 11.0), ResourceMode.PAUSED)

  def test_immediate_pause(self):
    governor = ResourceGovernor()
    self.assertEqual(governor.update(ResourceSample(memory_percent=90.0), 0.0), ResourceMode.PAUSED)


class TestModelArtifact(unittest.TestCase):
  def test_requires_complete_model_artifact(self):
    with tempfile.TemporaryDirectory() as directory:
      model_path = Path(directory) / "object_detector_tinygrad.pkl"
      manifest_path = Path(directory) / "object_detector_manifest.json"
      stamp_path = Path(directory) / "object_detector_tinygrad.sha256"
      expected_hash = "a" * 64
      manifest_path.write_text(json.dumps({"model": {"onnxSha256": expected_hash}}), encoding="utf-8")
      self.assertFalse(model_artifact_available(model_path, manifest_path, stamp_path))
      model_path.write_bytes(b"compiled")
      self.assertFalse(model_artifact_available(model_path, manifest_path, stamp_path))
      stamp_path.write_text("b" * 64, encoding="utf-8")
      self.assertFalse(model_artifact_available(model_path, manifest_path, stamp_path))
      stamp_path.write_text(expected_hash + "\n", encoding="utf-8")
      self.assertTrue(model_artifact_available(model_path, manifest_path, stamp_path))

      model_path.unlink()
      Path(f"{model_path}.chunkmanifest").write_text("2", encoding="utf-8")
      Path(f"{model_path}.chunk01of02").write_bytes(b"first")
      self.assertFalse(model_artifact_available(model_path, manifest_path, stamp_path))
      Path(f"{model_path}.chunk02of02").write_bytes(b"second")
      self.assertTrue(model_artifact_available(model_path, manifest_path, stamp_path))


if __name__ == "__main__":
  unittest.main()
