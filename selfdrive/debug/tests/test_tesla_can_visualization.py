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


def test_tesla_can_visualization_hides_idle_traffic_control_without_light():
  """A fresh control/sign frame is not proof of a traffic light: idle/SNA
  values must not render as a green light a few meters ahead."""
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "APP_trafficControl", 0, {
      "APP_tcFeatureState": 0,
      "APP_tcStateMachine": 0,
      "APP_tcControlType": 1,
      "APP_tcControlDistance": 1,
      "APP_tcControlLightState": 2,
    }),
    _frame(packer, "DAS_object", 2, {
      "DAS_objectId": 4,
      "DAS_roadSignId": 255,
      "DAS_roadSignStopLineDist": 1.0,
      "DAS_roadSignControlActive": 0,
      "DAS_roadSignSource": 0,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  traffic = visualization.snapshot(1_100_000_000)["traffic"]
  assert traffic["control_frame_fresh"]
  assert traffic["sign_frame_fresh"]
  assert not traffic["available"]
  assert not traffic["control_available"]
  assert not traffic["road_sign_available"]
  assert traffic["light_state"] == "unknown"
  assert traffic["control_distance_m"] is None
  assert traffic["stop_line_distance_m"] is None


def test_tesla_can_visualization_traffic_light_sign_gates_stop_line_and_arrow():
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()

  def sign_traffic(sign_values):
    visualization.reset()
    visualization.update([(1_000_000_000, [_frame(packer, "DAS_object", 2, {"DAS_objectId": 4, **sign_values})])])
    return visualization.snapshot(1_100_000_000)["traffic"]

  invalid = sign_traffic({
    "DAS_roadSignId": 255,
    "DAS_roadSignStopLineDist": 30,
    "DAS_roadSignSource": 0,
  })
  assert not invalid["road_sign_available"]
  assert invalid["stop_line_distance_m"] is None

  valid = sign_traffic({
    "DAS_roadSignId": 1,
    "DAS_roadSignStopLineDist": 30,
    "DAS_roadSignColor": 1,
    "DAS_roadSignArrow": 1,
    "DAS_roadSignSource": 2,
    "DAS_roadSignControlActive": 1,
  })
  assert valid["road_sign_available"]
  assert valid["stop_line_distance_m"] == 30
  assert valid["road_sign_arrow"] == "left"
  assert valid["road_sign_color"] == "red"


def test_tesla_can_visualization_rear_uses_live_flags_not_trip_latches():
  packer = CANPacker("tesla_modely_hw4_perception")

  def rear_snapshot(**flags):
    visualization = TeslaCanVisualization()
    visualization.update([(1_000_000_000, [_frame(packer, "DAS_visualDebug", 2, flags)])])
    return visualization.snapshot(1_100_000_000)["rear_vehicles"]

  only_trip = rear_snapshot(DAS_rearLeftVehDetectedTrip=1, DAS_rearRightVehDetectedTrip=1)
  assert not only_trip["left_live"]
  assert not only_trip["right_live"]

  left_now = rear_snapshot(DAS_rearLeftVehDetectedCurrent=1)
  assert left_now["left_live"]
  assert not left_now["right_live"]

  right_now = rear_snapshot(DAS_rearVehDetectedThisCycle=1, DAS_rearRightVehDetectedTrip=1)
  assert right_now["right_live"]
  assert not right_now["left_live"]


def test_tesla_can_visualization_decodes_road_sign_pedestrian_blind_spot_and_front_safety():
  packer = CANPacker("tesla_modely_hw4_perception")
  frames = [
    _frame(packer, "UI_driverAssistRoadSign", 2, {
      "UI_roadSign": 1,
      "UI_stopSignStopLineDist": 12.0,
      "UI_stopSignStopLineConf": 100,
    }),
    _frame(packer, "UI_driverAssistRoadSign", 2, {
      "UI_roadSign": 2,
      "UI_trafficLightStopLineDist": 30.0,
      "UI_trafficLightStopLineConf": 90,
    }),
    _frame(packer, "APP_pedestrianDetection", 1, {
      "APP_pedestrianDetectedFrontMain": 1,
      "APP_pedestrianDetectedBackup": 1,
      "APP_closestPedestrian1dX": 3.2,
      "APP_closestPedestrian1dY": -1.6,
      "APP_closestPedestrian2dX": 5.0,
    }),
    _frame(packer, "DAS_status", 0, {
      "DAS_blindSpotRearLeft": 2,
      "DAS_blindSpotRearRight": 1,
      "DAS_sideCollisionWarning": 1,
      "DAS_forwardCollisionWarning": 1,
    }),
    _frame(packer, "DAS_integratedSafetyFront", 0, {
      "DAS_targetDistanceFront": 12.0,
      "DAS_relativeVelocityFront": -4.0,
      "DAS_timeToImpactFront": 30,
      "DAS_predictedImpactOvrlapFront": 62.5,
    }),
  ]
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, frames)])

  scene = visualization.snapshot(1_100_000_000)
  road_sign = scene["road_sign"]
  assert road_sign["available"]
  assert road_sign["stop_sign_stop_line_distance_m"] == 12.0
  assert road_sign["stop_sign_stop_line_confidence"] == 100
  assert road_sign["traffic_light_stop_line_distance_m"] == 30.0
  assert road_sign["traffic_light_stop_line_confidence"] == 90

  pedestrians = scene["pedestrian_detection"]
  assert pedestrians["available"]
  assert pedestrians["front_main"]
  assert pedestrians["backup"]
  assert pedestrians["closest"][0]["x_m"] == 3.2
  assert pedestrians["closest"][0]["y_m"] == -1.6

  blind_spot = scene["blind_spot"]
  assert blind_spot["available"]
  assert blind_spot["left_level"] == 2
  assert blind_spot["right_level"] == 1
  assert blind_spot["left_live"]
  assert blind_spot["right_live"]
  assert blind_spot["side_collision_warning_level"] == 1
  assert blind_spot["forward_collision_warning_level"] == 1

  front_safety = scene["front_safety"]
  assert front_safety["available"]
  assert front_safety["target_distance_m"] == 12.0
  assert front_safety["relative_velocity_mps"] == -4.0
  assert front_safety["time_to_impact_s"] == 30.0
  assert front_safety["predicted_impact_overlap_pct"] == 62.5


def test_tesla_can_visualization_gates_road_sign_stop_line_sna():
  """Idle/SNA road sign frames must not produce a bogus stop line distance."""
  packer = CANPacker("tesla_modely_hw4_perception")
  visualization = TeslaCanVisualization()
  visualization.update([(1_000_000_000, [_frame(packer, "UI_driverAssistRoadSign", 2, {
    "UI_roadSign": 2,
    "UI_trafficLightStopLineDist": -8.0,
    "UI_trafficLightStopLineConf": 0,
  })])])

  road_sign = visualization.snapshot(1_100_000_000)["road_sign"]
  assert road_sign["available"]
  assert road_sign["traffic_light_stop_line_distance_m"] is None
  assert road_sign["traffic_light_stop_line_confidence"] is None
