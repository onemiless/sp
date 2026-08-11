from types import SimpleNamespace
import unittest

from openpilot.selfdrive.objectd.compile_object_model import graph_specs


class TestCompileObjectModel(unittest.TestCase):
  def test_graph_outputs_are_mapped_from_parsed_onnx_specs(self):
    input_spec = SimpleNamespace(shape=(1, 3, 416, 416))
    output_spec = SimpleNamespace(shape=(1, 3549, 85))
    graph = {
      "initializer": [{"name": "weights"}],
      "input": [
        {"name": "images", "parsed_type": input_spec},
        {"name": "weights", "parsed_type": SimpleNamespace(shape=(1,))},
      ],
      "output": [{"name": "output", "parsed_type": output_spec}],
    }
    inputs, outputs = graph_specs(graph)
    self.assertEqual(inputs, {"images": input_spec})
    self.assertEqual(outputs, {"output": output_spec})


if __name__ == "__main__":
  unittest.main()
