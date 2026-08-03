"""Read-only Tesla Model Y HW4 CAN context for the local web visualization.

This parser is deliberately owned by the debug web process. It is not part of
CarInterface, CAN validity, safety, or any control decision.
"""
from __future__ import annotations

import math
import time
from typing import Any

from opendbc.can import CANParser


DBC_NAME = "tesla_modely_hw4_perception"
MESSAGE_NAMES = (
  "UI_driverAssistMapData",
  "DAS_visualDebug",
  "DAS_lanes",
  "APP_trafficControl",
  "DAS_object",
)
BUS_NAMES = {0: "PARTY", 1: "VEH", 2: "AP"}
FRAME_STALE_NS = 2_000_000_000
NAV_STALE_NS = 5_000_000_000

USAGE = {0: "rejected", 1: "available", 2: "fused", 3: "blacklisted"}
VEHICLE_TYPES = {0: "unknown", 1: "truck", 2: "car", 3: "motorcycle", 4: "bicycle", 5: "pedestrian", 6: "ipso"}
ROAD_CLASSES = {0: "unknown", 1: "class_1_major", 2: "class_2", 3: "class_3", 4: "class_4", 5: "class_5", 6: "class_6_minor"}
SPEED_LIMIT_TYPES = {1: "regular", 2: "advisory", 3: "dependent", 4: "bumps", 7: "unknown"}
SPEED_LIMIT_VALUES = {
  1: 5, 2: 7, 3: 10, 4: 15, 5: 20, 6: 25, 7: 30, 8: 35, 9: 40, 10: 45,
  11: 50, 12: 55, 13: 60, 14: 65, 15: 70, 16: 75, 17: 80, 18: 85, 19: 90,
  20: 95, 21: 100, 22: 105, 23: 110, 24: 115, 25: 120, 26: 130, 27: 140,
  28: 150, 29: 160,
}
TRAFFIC_FEATURE_STATES = {0: "disabled", 1: "unavailable", 2: "available", 3: "active"}
TRAFFIC_MACHINE_STATES = {0: "disabled", 1: "standby", 2: "aware", 3: "warning", 4: "stopping", 5: "stopped", 6: "continuing"}
TRAFFIC_SOURCES = {0: "none", 1: "map", 2: "vision", 3: "map_and_vision"}
TRAFFIC_TYPES = {
  0: "invalid", 1: "unknown", 2: "stop_sign", 3: "traffic_light", 4: "yield", 5: "crosswalk",
  6: "keep_clear_enter", 7: "keep_clear_exit", 8: "suicide_left", 9: "pedestrian_crossing",
  10: "ramp_meter", 11: "speed_bump", 12: "speed_hump", 13: "traffic_rule",
  14: "all_way_stop_sign", 15: "bike_merge_from_left", 16: "bike_merge_from_right",
  17: "no_stop", 18: "t_implicit", 19: "t_implicit_by_name", 20: "t_implicit_by_geometry",
  21: "bev_junction", 22: "t_arm",
}
LIGHT_STATES = {0: "none", 1: "red", 2: "green", 3: "yellow", 4: "off", 5: "white", 6: "other"}
ROAD_SIGN_COLORS = {0: "none", 1: "red", 2: "yellow", 3: "green", 4: "red_yellow"}
ROAD_SIGN_TYPES = {0: "stop_sign", 1: "traffic_light", 255: "unavailable"}
ROAD_SIGN_SOURCES = {0: "none", 1: "navigation", 2: "vision"}
ROAD_SIGN_ARROWS = {0: "circle", 1: "left", 2: "right", 3: "straight", 4: "unknown"}
PLANNER_STATES = {
  0: "disabled", 1: "virtual_lane", 2: "follow", 3: "lane_change_requested", 4: "lane_change_in_progress",
  5: "waiting_side_obstacle", 6: "waiting_forward_obstacle", 7: "lane_change_abort",
}
BEHAVIOR_TYPES = {0: "invalid", 1: "in_lane", 2: "lane_change_left", 3: "lane_change_right"}
HEALTH_STATES = {0: "unavailable", 1: "nominal", 2: "degraded", 3: "severely_degraded", 4: "aborting", 5: "fault"}
ROAD_SURFACES = {0: "unknown", 1: "normal", 2: "enhanced"}


def _round(value: float, digits: int = 2) -> float:
  return round(float(value), digits)


def _int(values: dict[str, float], name: str) -> int:
  return int(values.get(name, 0.0))


def _bool(values: dict[str, float], name: str) -> bool:
  return bool(_int(values, name))


class TeslaCanVisualization:
  """Decode optional OEM context without participating in vehicle control."""

  def __init__(self, buses: tuple[int, ...] = (0, 1, 2)) -> None:
    optional_messages = [(name, math.nan) for name in MESSAGE_NAMES]
    self.parsers = {bus: CANParser(DBC_NAME, optional_messages, bus) for bus in buses}
    self.frames: dict[str, tuple[dict[str, float], int, int]] = {}
    self.object_frames: dict[int, tuple[dict[str, float], int, int]] = {}

  def reset(self) -> None:
    self.frames.clear()
    self.object_frames.clear()

  def update(self, can_packets: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    if not can_packets:
      return
    for bus, parser in self.parsers.items():
      updated = parser.update(can_packets)
      for name in MESSAGE_NAMES[:-1]:
        message = parser.dbc.name_to_msg[name]
        if message.address not in updated:
          continue
        timestamp = max(parser.ts_nanos[name].values(), default=0)
        previous = self.frames.get(name)
        if timestamp and (previous is None or timestamp >= previous[1]):
          self.frames[name] = (dict(parser.vl[name]), timestamp, bus)

      object_address = parser.dbc.name_to_msg["DAS_object"].address
      if object_address not in updated:
        continue
      all_values = parser.vl_all["DAS_object"]
      object_ids = all_values.get("DAS_objectId", [])
      timestamp = max(parser.ts_nanos["DAS_object"].values(), default=0)
      for index, object_id_value in enumerate(object_ids):
        object_id = int(object_id_value)
        if not 0 <= object_id <= 5:
          continue
        values = {name: samples[index] for name, samples in all_values.items() if index < len(samples)}
        previous = self.object_frames.get(object_id)
        if previous is None or timestamp >= previous[1]:
          self.object_frames[object_id] = (values, timestamp, bus)

  @staticmethod
  def _fresh(frame: tuple[dict[str, float], int, int] | None, now_ns: int, stale_ns: int = FRAME_STALE_NS) -> bool:
    return frame is not None and frame[1] > 0 and 0 <= now_ns - frame[1] <= stale_ns

  def _frame(self, name: str, now_ns: int, stale_ns: int = FRAME_STALE_NS) -> tuple[dict[str, float], int, int] | None:
    frame = self.frames.get(name)
    return frame if self._fresh(frame, now_ns, stale_ns) else None

  def _object_frame(self, mux: int, now_ns: int) -> tuple[dict[str, float], int, int] | None:
    frame = self.object_frames.get(mux)
    return frame if self._fresh(frame, now_ns) else None

  @staticmethod
  def _bus(frame: tuple[dict[str, float], int, int] | None) -> str | None:
    return BUS_NAMES.get(frame[2], str(frame[2])) if frame else None

  def _navigation(self, now_ns: int) -> dict[str, Any]:
    map_frame = self._frame("UI_driverAssistMapData", now_ns, NAV_STALE_NS)
    debug_frame = self._frame("DAS_visualDebug", now_ns, NAV_STALE_NS)
    map_values = map_frame[0] if map_frame else {}
    debug_values = debug_frame[0] if debug_frame else {}
    speed_code = _int(map_values, "UI_mapSpeedLimit")
    speed_unit = "kph" if _int(map_values, "UI_mapSpeedUnits") else "mph"
    branch_distance = float(map_values.get("UI_nextBranchDist", 310.0))
    nav_distance = float(debug_values.get("DAS_navDistance", 25500.0))
    return {
      "available": bool(map_frame or debug_frame),
      "route_active": _bool(map_values, "UI_navRouteActive"),
      "gps_road_match": _bool(map_values, "UI_gpsRoadMatch"),
      "parallel_autopark_enabled": _bool(map_values, "UI_parallelAutoparkEnabled"),
      "perpendicular_autopark_enabled": _bool(map_values, "UI_perpendicularAutoparkEnabled"),
      "nav_available": _bool(debug_values, "DAS_navAvailable"),
      "nav_distance_m": _round(nav_distance, 0) if nav_distance < 25500.0 else None,
      "next_branch_distance_m": _round(branch_distance, 0) if branch_distance < 310.0 else None,
      "next_branch_left_off_ramp": _bool(map_values, "UI_nextBranchLeftOffRamp"),
      "next_branch_right_off_ramp": _bool(map_values, "UI_nextBranchRightOffRamp"),
      "controlled_access": _bool(map_values, "UI_controlledAccess"),
      "road_class": ROAD_CLASSES.get(_int(map_values, "UI_roadClass"), "unknown"),
      "country_code": _int(map_values, "UI_countryCode"),
      "street_count": _int(map_values, "UI_streetCount"),
      "speed_limit": SPEED_LIMIT_VALUES.get(speed_code),
      "speed_limit_unlimited": speed_code == 30,
      "speed_limit_unit": speed_unit,
      "speed_limit_type": SPEED_LIMIT_TYPES.get(_int(map_values, "UI_mapSpeedLimitType"), "unknown"),
      "speed_limit_dependency": _int(map_values, "UI_mapSpeedLimitDependency"),
      "in_supercharger_geofence": _bool(map_values, "UI_inSuperchargerGeofence"),
      "autosteer_navigation_usage": USAGE.get(_int(debug_values, "DAS_autosteerNavigationUsage"), "unknown") if debug_frame else "unknown",
      "reject_navigation": _bool(map_values, "UI_rejectNav"),
      "reject_hpp": _bool(map_values, "UI_rejectHPP"),
      "reject_left_lane": _bool(map_values, "UI_rejectLeftLane"),
      "reject_right_lane": _bool(map_values, "UI_rejectRightLane"),
      "reject_left_free_space": _bool(map_values, "UI_rejectLeftFreeSpace"),
      "reject_right_free_space": _bool(map_values, "UI_rejectRightFreeSpace"),
      "reject_autosteer": _bool(map_values, "UI_rejectAutosteer"),
      "reject_hands_on": _bool(map_values, "UI_rejectHandsOn"),
      "accept_botts_dots": _bool(map_values, "UI_acceptBottsDots"),
      "autosteer_restricted": _bool(map_values, "UI_autosteerRestricted"),
      "pmm_enabled": _bool(map_values, "UI_pmmEnabled"),
      "sca_enabled": _bool(map_values, "UI_scaEnabled"),
      "sources": sorted(filter(None, (self._bus(map_frame), self._bus(debug_frame)))),
    }

  def _lanes(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_lanes", now_ns)
    if not frame:
      return {"available": False, "center": [], "left": [], "right": []}
    values = frame[0]
    view_range = min(max(float(values["DAS_virtualLaneViewRange"]), 0.0), 100.0)
    width = float(values["DAS_virtualLaneWidth"])
    coefficients = [float(values[f"DAS_virtualLaneC{i}"]) for i in range(4)]
    center = []
    left = []
    right = []
    for x in range(0, int(view_range) + 1, 4):
      y = sum(coefficient * x ** power for power, coefficient in enumerate(coefficients))
      center.append([x, _round(y)])
      if _bool(values, "DAS_leftLaneExists"):
        left.append([x, _round(y + width / 2.0)])
      if _bool(values, "DAS_rightLaneExists"):
        right.append([x, _round(y - width / 2.0)])
    return {
      "available": True,
      "bus": self._bus(frame),
      "left_exists": _bool(values, "DAS_leftLaneExists"),
      "right_exists": _bool(values, "DAS_rightLaneExists"),
      "width_m": _round(width),
      "view_range_m": _round(view_range, 0),
      "coefficients": [_round(value, 7) for value in coefficients],
      "left_usage": USAGE.get(_int(values, "DAS_leftLineUsage"), "unknown"),
      "right_usage": USAGE.get(_int(values, "DAS_rightLineUsage"), "unknown"),
      "left_fork": _int(values, "DAS_leftFork"),
      "right_fork": _int(values, "DAS_rightFork"),
      "center": center,
      "left": left,
      "right": right,
    }

  @staticmethod
  def _vehicle(values: dict[str, float], prefix: str, category: str, index: int) -> dict[str, Any] | None:
    suffix = "" if index == 1 else "2"
    dx = float(values.get(f"DAS_{prefix}Veh{suffix}Dx", 127.5))
    track_id = _int(values, f"DAS_{prefix}Veh{suffix}Id")
    id_unavailable = track_id == (127 if index == 1 else 0)
    if dx >= 127.5 or id_unavailable:
      return None
    type_code = _int(values, f"DAS_{prefix}Veh{suffix}Type")
    return {
      "category": category,
      "index": index,
      "track_id": track_id,
      "type": VEHICLE_TYPES.get(type_code, "unknown"),
      "x_m": _round(dx),
      "y_m": _round(values.get(f"DAS_{prefix}Veh{suffix}Dy", 0.0)),
      # T-CAN publishes the scale for VxRel but does not declare a physical unit.
      "relative_speed": _round(values.get(f"DAS_{prefix}Veh{suffix}VxRel", 0.0)),
      "relevant_for_control": _bool(values, f"DAS_{prefix}Veh{suffix}RelevantForControl"),
    }

  def _vehicles(self, now_ns: int) -> tuple[list[dict[str, Any]], dict[str, bool], list[str]]:
    vehicles: list[dict[str, Any]] = []
    sources: set[str] = set()
    headings_frame = self._object_frame(5, now_ns)
    headings = headings_frame[0] if headings_frame else {}
    if headings_frame:
      sources.add(self._bus(headings_frame) or "")
    for mux, prefix, category in ((0, "lead", "lead"), (1, "left", "left"), (2, "right", "right")):
      frame = self._object_frame(mux, now_ns)
      if not frame:
        continue
      sources.add(self._bus(frame) or "")
      for index in (1, 2):
        vehicle = self._vehicle(frame[0], prefix, category, index)
        if vehicle:
          suffix = "" if index == 1 else "2"
          heading_name = f"DAS_{prefix}Veh{suffix}Heading"
          vehicle["heading_rad"] = _round(headings[heading_name], 3) if heading_name in headings else None
          vehicles.append(vehicle)
    cutin_frame = self._object_frame(3, now_ns)
    if cutin_frame:
      sources.add(self._bus(cutin_frame) or "")
      vehicle = self._vehicle(cutin_frame[0], "cutin", "cutin", 1)
      if vehicle:
        vehicle["heading_rad"] = _round(cutin_frame[0].get("DAS_cutinVehHeading", 0.0), 3)
        vehicles.append(vehicle)

    debug_frame = self._frame("DAS_visualDebug", now_ns)
    debug = debug_frame[0] if debug_frame else {}
    rear = {
      "detected_this_cycle": _bool(debug, "DAS_rearVehDetectedThisCycle"),
      "left_current": _bool(debug, "DAS_rearLeftVehDetectedCurrent"),
      "left_trip": _bool(debug, "DAS_rearLeftVehDetectedTrip"),
      "right_trip": _bool(debug, "DAS_rearRightVehDetectedTrip"),
    }
    return vehicles, rear, sorted(source for source in sources if source)

  def _traffic(self, now_ns: int) -> dict[str, Any]:
    control_frame = self._frame("APP_trafficControl", now_ns)
    sign_frame = self._object_frame(4, now_ns)
    control = control_frame[0] if control_frame else {}
    sign = sign_frame[0] if sign_frame else {}
    control_distance = float(control.get("APP_tcControlDistance", 255.0))
    stop_line_distance = float(sign.get("DAS_roadSignStopLineDist", 184.6))
    sign_type_code = _int(sign, "DAS_roadSignId")
    return {
      "available": bool(control_frame or sign_frame),
      "control_available": bool(control_frame),
      "road_sign_available": bool(sign_frame),
      "feature_state": TRAFFIC_FEATURE_STATES.get(_int(control, "APP_tcFeatureState"), "unknown"),
      "state_machine": TRAFFIC_MACHINE_STATES.get(_int(control, "APP_tcStateMachine"), "unknown"),
      "control_source": TRAFFIC_SOURCES.get(_int(control, "APP_tcControlSource"), "unknown"),
      "control_type": TRAFFIC_TYPES.get(_int(control, "APP_tcControlType"), "unknown"),
      "control_distance_m": _round(control_distance) if control_distance < 255.0 else None,
      "light_state": LIGHT_STATES.get(_int(control, "APP_tcControlLightState"), "unknown"),
      "continuation_reason": _int(control, "APP_tcContinuationReason"),
      "confirmation_type": _int(control, "APP_tcConfirmationType"),
      "warning_suppression_reason": _int(control, "APP_tcWarningSuppressionReason"),
      "unavailable_reason": _int(control, "APP_tcUnavailableReason"),
      "vision_light": _bool(control, "APP_tcVisionLight"),
      "vision_sign": _bool(control, "APP_tcVisionSign"),
      "vision_road_marking": _bool(control, "APP_tcVisionRoadMarking"),
      "vision_line": _bool(control, "APP_tcVisionLine"),
      "road_sign_type": ROAD_SIGN_TYPES.get(sign_type_code, "unknown") if sign_frame else "unknown",
      "road_sign_color": ROAD_SIGN_COLORS.get(_int(sign, "DAS_roadSignColor"), "unknown") if sign_frame else "unknown",
      "stop_line_distance_m": _round(stop_line_distance) if stop_line_distance < 184.6 else None,
      "road_sign_active": _bool(sign, "DAS_roadSignControlActive"),
      "road_sign_source": ROAD_SIGN_SOURCES.get(_int(sign, "DAS_roadSignSource"), "unknown") if sign_frame else "unknown",
      "road_sign_arrow": ROAD_SIGN_ARROWS.get(_int(sign, "DAS_roadSignArrow"), "unknown") if sign_frame else "unknown",
      "road_sign_orientation": _int(sign, "DAS_roadSignOrientation"),
      "sources": sorted(filter(None, (self._bus(control_frame), self._bus(sign_frame)))),
    }

  def _driver_assist(self, now_ns: int) -> dict[str, Any]:
    frame = self._frame("DAS_visualDebug", now_ns)
    values = frame[0] if frame else {}
    return {
      "available": bool(frame),
      "bus": self._bus(frame),
      "planner_state": PLANNER_STATES.get(_int(values, "DAS_plannerState"), "unknown"),
      "behavior": BEHAVIOR_TYPES.get(_int(values, "DAS_behaviorType"), "unknown"),
      "health": HEALTH_STATES.get(_int(values, "DAS_autosteerHealthState"), "unknown"),
      "health_anomaly_level": _int(values, "DAS_autosteerHealthAnomalyLevel"),
      "road_surface": ROAD_SURFACES.get(_int(values, "DAS_roadSurfaceType"), "unknown"),
      "vehicles_usage": USAGE.get(_int(values, "DAS_autosteerVehiclesUsage"), "unknown"),
      "hpp_usage": USAGE.get(_int(values, "DAS_autosteerHPPUsage"), "unknown"),
      "model_usage": USAGE.get(_int(values, "DAS_autosteerModelUsage"), "unknown"),
      "botts_dots_usage": USAGE.get(_int(values, "DAS_autosteerBottsDotsUsage"), "unknown"),
      "offset_side": _int(values, "DAS_offsetSide"),
      "last_line_preference_reason": _int(values, "DAS_lastLinePreferenceReason"),
      "last_abort_reason": _int(values, "DAS_lastAutosteerAbortReason"),
      "developer_app_interface_enabled": _bool(values, "DAS_devAppInterfaceEnabled"),
      "smart_speed_active": _bool(values, "DAS_accSmartSpeedActive"),
      "smart_speed_state": _int(values, "DAS_accSmartSpeedState"),
      "traffic_aware_set_speed": _bool(values, "DAS_trafficAwareSetSpeedInUse"),
      "isa_state": _int(values, "DAS_isaSystemState"),
      "ulc_in_progress": _bool(values, "DAS_ulcInProgress"),
      "ulc_type": _int(values, "DAS_ulcType"),
    }

  def snapshot(self, now_ns: int | None = None) -> dict[str, Any]:
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    navigation = self._navigation(now_ns)
    lanes = self._lanes(now_ns)
    vehicles, rear, vehicle_sources = self._vehicles(now_ns)
    traffic = self._traffic(now_ns)
    driver_assist = self._driver_assist(now_ns)
    buses = sorted({
      *(navigation.get("sources") or []),
      *(traffic.get("sources") or []),
      *vehicle_sources,
      *([lanes["bus"]] if lanes.get("bus") else []),
      *([driver_assist["bus"]] if driver_assist.get("bus") else []),
    })
    return {
      "available": bool(navigation["available"] or lanes["available"] or vehicles or traffic["available"] or driver_assist["available"]),
      "dbc": DBC_NAME,
      "buses": buses,
      "navigation": navigation,
      "lanes": lanes,
      "vehicles": vehicles,
      "rear_vehicles": rear,
      "traffic": traffic,
      "driver_assist": driver_assist,
      "pedestrians": [vehicle for vehicle in vehicles if vehicle["type"] == "pedestrian"],
      "cyclists": [vehicle for vehicle in vehicles if vehicle["type"] in ("bicycle", "motorcycle")],
    }
