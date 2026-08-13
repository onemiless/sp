from pathlib import Path
import re
import unittest


REPO = Path(__file__).resolve().parents[4]


class TestScopeContract(unittest.TestCase):
  def test_experimental_feature_defaults_are_false(self):
    params = (REPO / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
    self.assertIn('{"VisionObjectDetectionEnabled", {PERSISTENT | BACKUP, BOOL, "0"}}', params)
    self.assertIn('{"VisionObjectOverlay", {PERSISTENT | BACKUP, BOOL, "0"}}', params)
    self.assertIn('{"VisionObjectDistanceDisplay", {PERSISTENT | BACKUP, BOOL, "0"}}', params)
    self.assertIn('{"VisionObjectDistanceCalibration", {PERSISTENT | BACKUP, JSON}}', params)

  def test_objectd_build_and_validation_recording_are_explicit(self):
    sconstruct = (REPO / "SConstruct").read_text(encoding="utf-8")
    process_config = (REPO / "openpilot/system/manager/process_config.py").read_text(encoding="utf-8")
    self.assertIn("AddOption('--objectd'", sconstruct)
    self.assertIn("if GetOption('objectd')", sconstruct)
    self.assertIn('PythonProcess("vision_object_recorder"', process_config)

  def test_tesla_settings_do_not_gate_by_device_type(self):
    settings = (REPO / "openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/tesla.py").read_text(encoding="utf-8")
    self.assertNotIn("HARDWARE.get_device_type()", settings)
    self.assertNotIn("vision_object_distance_toggle.action_item.set_enabled(False)", settings)

  def test_camera_view_defers_pyray_type_annotations(self):
    camera_view = (REPO / "openpilot/selfdrive/ui/onroad/cameraview.py").read_text(encoding="utf-8")
    self.assertTrue(camera_view.startswith("from __future__ import annotations\n"))

  def test_display_uses_raw_computed_distance_without_uncertainty_text(self):
    renderer = (REPO / "openpilot/selfdrive/ui/onroad/vision_object_renderer.py").read_text(encoding="utf-8")
    self.assertIn('label += f" {float(obj.distance):.1f}m"', renderer)
    self.assertNotIn("distanceStd", renderer)

  def test_model_runner_matches_compiled_host_input_device(self):
    model_runner = (REPO / "openpilot/selfdrive/objectd/model_runner.py").read_text(encoding="utf-8")
    self.assertIn('self._tensor(input_tensor, device="NPY")', model_runner)

  def test_device_build_and_runtime_share_persistent_model_root(self):
    scons = (REPO / "openpilot/selfdrive/objectd/SConscript").read_text(encoding="utf-8")
    model_runner = (REPO / "openpilot/selfdrive/objectd/model_runner.py").read_text(encoding="utf-8")
    self.assertIn("object_model_root(device=arch == 'larch64')", scons)
    self.assertIn("MODEL_ARTIFACT_DIR = object_model_root()", model_runner)

  def test_feature_does_not_import_control_chain_modules(self):
    sources = list((REPO / "openpilot/selfdrive/objectd").glob("*.py"))
    sources += [REPO / "openpilot/selfdrive/ui/onroad/vision_object_renderer.py",
                REPO / "openpilot/system/manager/vision_object_latch.py"]
    forbidden = re.compile(r"^\s*(?:from|import)\s+.*(?:controlsd|radard|planner|carcontroller|sendcan|panda)", re.MULTILINE)
    for source in sources:
      self.assertIsNone(forbidden.search(source.read_text(encoding="utf-8")), source)


if __name__ == "__main__":
  unittest.main()
