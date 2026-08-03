from opendbc.can import CANPacker

from openpilot.selfdrive.debug.tesla_can_visualization import TeslaCanVisualization


def _frame(packer, message, bus, values):
  return packer.make_can_msg(message, bus, values)


def test_tesla_can_visualization_builds_scene_from_multiple_buses():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "UI_driverAssistMapData", 1, {
      "UI_navRouteActive": 1,
      "UI_gpsRoadMatch": 1,
      "UI_mapSpeedUnits": 1,
      "UI_mapSpeedLimit": 13,
      "UI_nextBranchDist": 120,
      "UI_nextBranchRightOffRamp": 1,
      "UI_parallelAutoparkEnabled": 1,
      "UI_inSuperchargerGeofence": 1,
      "UI_rejectNav": 1,
    }),
    _frame(packer, "DAS_lanes", 2, {
      "DAS_leftLaneExists": 1,
      "DAS_rightLaneExists": 1,
      "DAS_virtualLaneWidth": 3.5,
      "DAS_virtualLaneViewRange": 80,
      "DAS_virtualLaneC0": 0,
      "DAS_virtualLaneC1": 0,
      "DAS_virtualLaneC2": 0,
      "DAS_virtualLaneC3": 0,
      "DAS_leftLineUsage": 2,
      "DAS_rightLineUsage": 2,
    }),
    _frame(packer, "APP_trafficControl", 0, {
      "APP_tcFeatureState": 3,
      "APP_tcStateMachine": 4,
      "APP_tcControlSource": 3,
      "APP_tcControlType": 3,
      "APP_tcControlDistance": 42,
      "APP_tcControlLightState": 1,
      "APP_tcVisionLight": 1,
      "APP_tcVisionLine": 1,
    }),
    _frame(packer, "DAS_object", 2, {
      "DAS_objectId": 0,
      "DAS_leadVehType": 2,
      "DAS_leadVehRelevantForControl": 1,
      "DAS_leadVehDx": 25,
      "DAS_leadVehVxRel": -2,
      "DAS_leadVehDy": 0,
      "DAS_leadVehId": 7,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)

  assert scene["available"]
  assert scene["buses"] == ["AP", "PARTY", "VEH"]
  assert scene["navigation"]["route_active"]
  assert scene["navigation"]["next_branch_distance_m"] == 120
  assert scene["navigation"]["speed_limit"] == 60
  assert scene["navigation"]["parallel_autopark_enabled"]
  assert scene["navigation"]["in_supercharger_geofence"]
  assert scene["navigation"]["reject_navigation"]
  assert scene["lanes"]["left_usage"] == "fused"
  assert scene["lanes"]["right_usage"] == "fused"
  assert scene["traffic"]["light_state"] == "red"
  assert scene["traffic"]["control_available"]
  assert not scene["traffic"]["road_sign_available"]
  assert scene["traffic"]["control_distance_m"] == 42
  assert scene["vehicles"] == [{
    "category": "lead", "index": 1, "track_id": 7, "type": "car", "x_m": 25.0, "y_m": -0.0,
    "relative_speed": -2.0, "relevant_for_control": True, "heading_rad": None,
  }]


def test_tesla_can_visualization_hides_stale_optional_data():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "APP_trafficControl", 0, {
    "APP_tcFeatureState": 3,
    "APP_tcControlType": 3,
    "APP_tcControlDistance": 20,
    "APP_tcControlLightState": 2,
  })
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [frame])])

  assert visualization.snapshot(1_100_000_000)["traffic"]["available"]
  assert not visualization.snapshot(4_000_000_000)["traffic"]["available"]
  assert not visualization.snapshot(4_000_000_000)["available"]


def test_tesla_can_visualization_reset_discards_cached_vehicle_data():
  packer = CANPacker("tesla_modely_hw4_perception")
  frame = _frame(packer, "DAS_object", 2, {
    "DAS_objectId": 0,
    "DAS_leadVehType": 2,
    "DAS_leadVehDx": 15,
    "DAS_leadVehId": 4,
  })
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [frame])])
  assert visualization.snapshot(1_100_000_000)["vehicles"]

  visualization.reset()
  assert not visualization.snapshot(1_100_000_000)["available"]
