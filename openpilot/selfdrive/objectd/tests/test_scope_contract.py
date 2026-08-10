from pathlib import Path
import re
import unittest


REPO = Path(__file__).resolve().parents[4]


class TestScopeContract(unittest.TestCase):
  def test_enable_default_is_true_and_display_defaults_are_false(self):
    params = (REPO / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
    self.assertIn('{"VisionObjectDetectionEnabled", {PERSISTENT | BACKUP, BOOL, "1"}}', params)
    self.assertIn('{"VisionObjectOverlay", {PERSISTENT | BACKUP, BOOL, "0"}}', params)
    self.assertIn('{"VisionObjectDistanceDisplay", {PERSISTENT | BACKUP, BOOL, "0"}}', params)

  def test_feature_does_not_import_control_chain_modules(self):
    sources = list((REPO / "openpilot/selfdrive/objectd").glob("*.py"))
    sources += [REPO / "openpilot/selfdrive/ui/onroad/vision_object_renderer.py",
                REPO / "openpilot/system/manager/vision_object_latch.py"]
    forbidden = re.compile(r"^\s*(?:from|import)\s+.*(?:controlsd|radard|planner|carcontroller|sendcan|panda)", re.MULTILINE)
    for source in sources:
      self.assertIsNone(forbidden.search(source.read_text(encoding="utf-8")), source)


if __name__ == "__main__":
  unittest.main()
