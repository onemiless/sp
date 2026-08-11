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

  def test_objectd_build_and_validation_recording_are_explicit(self):
    sconstruct = (REPO / "SConstruct").read_text(encoding="utf-8")
    process_config = (REPO / "openpilot/system/manager/process_config.py").read_text(encoding="utf-8")
    self.assertIn("AddOption('--objectd'", sconstruct)
    self.assertIn("if GetOption('objectd')", sconstruct)
    self.assertIn('PythonProcess("vision_object_recorder"', process_config)

  def test_feature_does_not_import_control_chain_modules(self):
    sources = list((REPO / "openpilot/selfdrive/objectd").glob("*.py"))
    sources += [REPO / "openpilot/selfdrive/ui/onroad/vision_object_renderer.py",
                REPO / "openpilot/system/manager/vision_object_latch.py"]
    forbidden = re.compile(r"^\s*(?:from|import)\s+.*(?:controlsd|radard|planner|carcontroller|sendcan|panda)", re.MULTILINE)
    for source in sources:
      self.assertIsNone(forbidden.search(source.read_text(encoding="utf-8")), source)


if __name__ == "__main__":
  unittest.main()
