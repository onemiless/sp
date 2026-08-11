import re
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]


class TestSchemaContract(unittest.TestCase):
  def test_reserved_id_and_union_ordinal_unchanged(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
    log = (ROOT / "openpilot/cereal/log.capnp").read_text(encoding="utf-8")
    self.assertRegex(custom, r"struct VisionObjectStateSP @0xcb9fd56c7057593a\s*\{")
    self.assertRegex(log, r"visionObjectStateSP @136\s*:Custom\.VisionObjectStateSP;")
    self.assertNotRegex(log, r"customReserved10\s+@136")

  def test_service_is_fixed_five_hz(self):
    services = (ROOT / "openpilot/cereal/services.py").read_text(encoding="utf-8")
    self.assertRegex(services, r'"visionObjectStateSP":\s*\(False,\s*5\.\)')

  def test_bbox_and_position_fields_have_named_components(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
    object_body = re.search(r"struct VisionObject \{(?P<body>.*?)\n  \}", custom, re.DOTALL)
    self.assertIsNotNone(object_body)
    self.assertIn("bboxNormalized @3 :NormalizedBoundingBox", object_body.group("body"))
    self.assertIn("bboxVelocityNormalized @4 :NormalizedBoundingBoxVelocity", object_body.group("body"))
    self.assertIn("positionStd @13 :PositionStdDev", object_body.group("body"))
    self.assertRegex(custom, r"struct NormalizedBoundingBox \{\s*left @0 :Float32;\s*top @1 :Float32;\s*right @2 :Float32;\s*bottom @3 :Float32;")
    self.assertRegex(custom, r"struct NormalizedBoundingBoxVelocity \{\s*leftPerSecond @0 :Float32;\s*topPerSecond @1 :Float32;")
    self.assertRegex(custom, r"struct PositionStdDev \{\s*x @0 :Float32;\s*y @1 :Float32;\s*z @2 :Float32;")


if __name__ == "__main__":
  unittest.main()
