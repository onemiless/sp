import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.objectd.distance_calibration import (CalibrationProfile, CalibrationSample, apply_profile,
                                                             fit_profile, match_lead_to_object, profiles_from_json,
                                                             profiles_to_payload)
from openpilot.selfdrive.objectd.objectd import _object_payload


class TestDistanceCalibration(unittest.TestCase):
  def test_robust_affine_fit_rejects_bad_matches(self):
    samples = [CalibrationSample("road", i, 10.0 + i, 0.1, 1.08 * (10.0 + i) - 1.4, 0.1, True)
               for i in range(40)]
    samples += [CalibrationSample("road", 100 + i, 15.0 + i, 0.0, 80.0 - i, 0.0, False) for i in range(4)]
    profile = fit_profile(samples, "road", created_at=123)
    self.assertIsNotNone(profile)
    self.assertAlmostEqual(profile.scale, 1.08, places=2)
    self.assertAlmostEqual(profile.offset_m, -1.4, places=1)
    self.assertGreaterEqual(profile.sample_count, 35)
    self.assertLess(profile.rmse_m, 0.2)

  def test_fit_requires_distance_span_and_enough_samples(self):
    too_few = [CalibrationSample("road", i, 20.0 + i * 0.1, 0.0, 20.0 + i * 0.1, 0.0, True) for i in range(10)]
    self.assertIsNone(fit_profile(too_few, "road", created_at=1))
    no_span = [CalibrationSample("road", i, 20.0 + i * 0.01, 0.0, 20.0 + i * 0.01, 0.0, True) for i in range(40)]
    self.assertIsNone(fit_profile(no_span, "road", created_at=1))

  def test_side_distance_uses_corrected_longitudinal_and_lateral_components(self):
    profile = CalibrationProfile("wide", 1.1, -1.0, 50, 0.3, 1)
    position, distance = apply_profile((20.0, 10.0, 0.0), profile)
    np.testing.assert_allclose(position, (21.0, 11.0, 0.0))
    self.assertAlmostEqual(distance, np.hypot(21.0, 11.0))

  def test_profile_round_trip_uses_typed_params_payload(self):
    profile = CalibrationProfile("road", 1.02, -0.8, 42, 0.4, 123)
    restored = profiles_from_json(profiles_to_payload({"road": profile}))
    self.assertEqual(restored, {"road": profile})

  def test_object_payload_applies_profile_to_published_components(self):
    track = {
      "track_id": 1, "class_id": 2, "confidence": 0.8, "bbox": (0.4, 0.3, 0.6, 0.8),
      "velocity": (0.0, 0.0, 0.0, 0.0), "bbox_clipped": False, "prediction_valid": True,
    }
    estimate = SimpleNamespace(valid=True, position=(20.0, 10.0, 0.0), distance_m=np.hypot(20.0, 10.0),
                               distance_std_m=0.5)
    profile = CalibrationProfile("road", 1.1, -1.0, 50, 0.3, 1)
    calibration = SimpleNamespace(intrinsics=None, rpy_calib=(), rpy_spread=(), camera_height_m=0.0,
                                  camera_from_device_euler=None)
    with patch("openpilot.selfdrive.objectd.objectd.estimate_camera_ground_distance", return_value=estimate):
      payload = _object_payload(track, (1928, 1208), calibration, None, profile)
    self.assertAlmostEqual(payload["x"], 21.0)
    self.assertAlmostEqual(payload["y"], 11.0)
    self.assertAlmostEqual(payload["distance"], np.hypot(21.0, 11.0))
    self.assertAlmostEqual(payload["distanceStd"], 0.55)

  def test_match_uses_same_target_not_just_nearest_distance(self):
    objects = [
      {"trackId": 1, "classId": 2, "corridorState": "inside", "x": 30.0, "y": 3.0, "distanceValid": True},
      {"trackId": 2, "classId": 2, "corridorState": "inside", "x": 33.0, "y": 0.2, "distanceValid": True},
    ]
    match = match_lead_to_object({"dRel": 31.0, "yRel": 0.1, "present": True, "radar": True}, objects, "road", 10)
    self.assertIsNotNone(match)
    self.assertEqual(match.track_id, 2)
    self.assertEqual(match.sp_distance_m, 31.0)

  def test_match_rejects_adjacent_lane_and_ambiguous_pair(self):
    adjacent = [{"trackId": 1, "classId": 2, "corridorState": "outside", "x": 20.0, "y": 0.0, "distanceValid": True}]
    self.assertIsNone(match_lead_to_object({"dRel": 20.0, "yRel": 0.0, "present": True, "radar": True}, adjacent, "road", 1))
    ambiguous = [
      {"trackId": 1, "classId": 2, "corridorState": "inside", "x": 20.0, "y": -0.1, "distanceValid": True},
      {"trackId": 2, "classId": 2, "corridorState": "inside", "x": 20.2, "y": 0.1, "distanceValid": True},
    ]
    self.assertIsNone(match_lead_to_object({"dRel": 20.1, "yRel": 0.0, "present": True, "radar": True}, ambiguous, "road", 1))


if __name__ == "__main__":
  unittest.main()
