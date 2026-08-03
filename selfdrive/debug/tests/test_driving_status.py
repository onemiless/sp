from cereal import messaging
from openpilot.selfdrive.debug.driving_status import _is_tesla_model_y, _set_speed_kph
from opendbc.sunnypilot.car.tesla.carstate_ext import publish_tesla_road_context


def test_set_speed_matches_device_hud_units_and_fallback():
  assert _set_speed_kph(110.0, 35.0) == 110.0
  assert _set_speed_kph(0.0, 35.0) == 35.0


def test_hw4_visualization_decoder_is_limited_to_tesla_model_y():
  model_y = messaging.new_message("carParams")
  model_y.carParams.brand = "tesla"
  model_y.carParams.carFingerprint = "TESLA_MODEL_Y"
  assert _is_tesla_model_y(model_y.carParams.to_bytes())

  other_car = messaging.new_message("carParams")
  other_car.carParams.brand = "toyota"
  other_car.carParams.carFingerprint = "TOYOTA PRIUS 2017"
  assert not _is_tesla_model_y(other_car.carParams.to_bytes())
  assert not _is_tesla_model_y(b"not capnp")


def test_tesla_oem_road_context_is_read_only_and_hides_stale_data():
  values = {"DAS_trafficLightColor": 1, "DAS_stopLineDist": 25.5}
  fresh = messaging.new_message("carStateSP")
  publish_tesla_road_context(fresh.carStateSP, values, 1_000_000_000, 1_100_000_000)
  assert fresh.carStateSP.teslaRoadContext.to_dict() == {
    "available": True, "trafficLightColor": 1, "stopLineDistance": 25.5,
  }

  stale = messaging.new_message("carStateSP")
  publish_tesla_road_context(stale.carStateSP, values, 1, 2_000_000_000)
  assert stale.carStateSP.teslaRoadContext.available is False
