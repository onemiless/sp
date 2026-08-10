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

  def test_bbox_and_position_fields_are_bounded_lists_by_producer_contract(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text(encoding="utf-8")
    object_body = re.search(r"struct VisionObject \{(?P<body>.*?)\n  \}", custom, re.DOTALL)
    self.assertIsNotNone(object_body)
    self.assertIn("bboxNormalized @3 :List(Float32)", object_body.group("body"))
    self.assertIn("bboxVelocityNormalized @4 :List(Float32)", object_body.group("body"))
    self.assertIn("positionStd @13 :List(Float32)", object_body.group("body"))


if __name__ == "__main__":
  unittest.main()
