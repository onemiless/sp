import inspect
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import MPC_SOURCES, PARAM_DIM, LongitudinalMpc, LongitudinalPlanSource
import openpilot.selfdrive.controls.lib.longitudinal_planner as planner_module
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_cruise_accel, get_max_accel
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP


DT = 0.05


def _cp():
  return SimpleNamespace(
    openpilotLongitudinalControl=True,
    steerRatio=12.0,
    wheelbase=2.8,
    longitudinalActuatorDelay=0.2,
  )


@pytest.mark.parametrize("v_ego", [0.0, 10.0, 25.0, 40.0])
def test_normal_cruise_respects_comfort_and_jerk_limits(v_ego):
  accel, should_stop = get_cruise_accel(False, v_ego + 20.0, v_ego, 0.0, 0.0, _cp(), DT, 0.0, True)

  expected_jerk_step = get_max_accel(v_ego) * DT
  assert accel == pytest.approx(expected_jerk_step)
  assert accel <= get_max_accel(v_ego)
  assert not should_stop


def test_normal_cruise_decelerates_when_set_speed_drops():
  accel, should_stop = get_cruise_accel(False, 20.0, 25.0, 0.0, 0.0, _cp(), DT, 0.0, True)

  assert accel < 0.0
  assert not should_stop


def test_normal_cruise_respects_turn_and_coast_limits():
  turn_limited, _ = get_cruise_accel(False, 40.0, 20.0, 0.0, 20.0, _cp(), 1.0, 0.0, True)
  coast_limited, _ = get_cruise_accel(False, 40.0, 20.0, 0.0, 0.0, _cp(), 1.0, -0.4, False)

  assert turn_limited < get_max_accel(20.0)
  assert coast_limited <= -0.4


def test_e2e_cruise_skips_comfort_turn_coast_and_jerk_limits():
  accel, should_stop = get_cruise_accel(True, 30.0, 5.0, -1.0, 30.0, _cp(), DT, -0.5, False)

  assert accel == ACCEL_MAX
  assert not should_stop


def test_zero_cruise_requests_stop():
  _, should_stop = get_cruise_accel(False, 0.0, 10.0, 0.0, 0.0, _cp(), DT, 0.0, True)

  assert should_stop


def test_mpc_contract_is_lead_only():
  assert MPC_SOURCES == (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1)
  assert PARAM_DIM == 8
  assert "v_cruise" not in inspect.signature(LongitudinalMpc.update).parameters


def test_mpc_selects_closer_lead_and_preserves_parameter_order():
  mpc = LongitudinalMpc.__new__(LongitudinalMpc)
  mpc.x0 = np.array([0.0, 10.0, 0.0])
  mpc.params = np.zeros((13, PARAM_DIM))
  mpc.a_prev = np.linspace(0.1, 0.2, 13)
  mpc.x_sol = np.zeros((13, 3))
  mpc.yref = np.zeros((13, 6))
  mpc.crash_cnt = 0
  mpc.tuning = SimpleNamespace(
    comfort_brake=2.5,
    stop_distance=6.0,
    lead_danger_factor=0.75,
    t_follow_relaxed=1.75,
    t_follow_standard=1.45,
    t_follow_aggressive=1.25,
  )
  mpc.update_tuning = lambda: None
  mpc.run = lambda: None
  mpc.solver = SimpleNamespace(set=lambda *args, **kwargs: None)

  lead_one = SimpleNamespace(present=True, modelProb=0.0)
  lead_two = SimpleNamespace(present=True, modelProb=0.0)
  radar_state = SimpleNamespace(leadOne=lead_one, leadTwo=lead_two)
  lead_one_xv = np.column_stack((np.full(13, 50.0), np.full(13, 10.0)))
  lead_two_xv = np.column_stack((np.full(13, 30.0), np.full(13, 8.0)))
  mpc.process_lead = lambda lead: lead_one_xv if lead is lead_one else lead_two_xv

  mpc.update(radar_state)

  assert mpc.source == LongitudinalPlanSource.lead1
  assert np.all(mpc.params[:, 0] == ACCEL_MIN)
  assert np.all(mpc.params[:, 1] == ACCEL_MAX)
  assert np.all(mpc.params[:, 3] == mpc.a_prev)
  assert np.all(mpc.params[:, 5] == mpc.tuning.lead_danger_factor)
  assert np.all(mpc.params[:, 6] == mpc.tuning.comfort_brake)
  assert np.all(mpc.params[:, 7] == mpc.tuning.stop_distance)


class FakeMpc:
  def __init__(self):
    self.v_solution = np.full(13, 10.0)
    self.a_solution = np.full(13, 1.5)
    self.j_solution = np.zeros(12)
    self.crash_cnt = 0
    self.source = LongitudinalPlanSource.lead0
    self.x0 = np.zeros(3)
    self.set_state_history = []

  def set_weights(self, *args, **kwargs):
    pass

  def set_cur_state(self, v, a):
    self.x0[1:] = (v, a)
    self.set_state_history.append((v, a))

  def update(self, *args, **kwargs):
    pass


def _sm(long_control_state=LongCtrlState.pid, a_ego=0.4, experimental_mode=False):
  lead = SimpleNamespace(present=False)
  return {
    "carControl": SimpleNamespace(orientationNED=[]),
    "carState": SimpleNamespace(
      vEgo=10.0,
      vCruise=36.0,
      aEgo=a_ego,
      standstill=False,
      steeringAngleDeg=0.0,
    ),
    "controlsState": SimpleNamespace(longControlState=long_control_state, forceDecel=False),
    "selfdriveState": SimpleNamespace(enabled=True, personality=0, experimentalMode=experimental_mode),
    "liveParameters": SimpleNamespace(angleOffsetDeg=0.0),
    "modelV2": SimpleNamespace(
      meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[0.0, 1.0])),
      action=SimpleNamespace(desiredAcceleration=1.0, shouldStop=False),
      velocity=SimpleNamespace(x=np.full(33, 10.0)),
    ),
    "radarState": SimpleNamespace(leadOne=lead, leadTwo=lead),
  }


def _planner():
  planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
  planner.CP = _cp()
  planner.frame = -1
  planner.dt = DT
  planner.allow_throttle = True
  planner.stop_line_extra_decel = 0.5
  planner.a_desired = 0.4
  planner.v_desired_filter = FirstOrderFilter(10.0, 2.0, DT)
  planner.a_cruise = 0.0
  planner.output_a_target = 0.0
  planner.output_should_stop = False
  planner.fcw = False
  planner.mpc = FakeMpc()
  planner.v_desired_trajectory = np.zeros(17)
  planner.a_desired_trajectory = np.zeros(17)
  planner.j_desired_trajectory = np.zeros(17)
  planner._update_params = lambda: None
  return planner


def test_next_cycle_mpc_state_uses_previous_final_output(monkeypatch):
  planner = _planner()
  sm = _sm()

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: False)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (1.5, False))

  sp_targets = iter(((10.0, 0.6), (10.0, -0.7)))
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: next(sp_targets))

  planner.update(sm)
  assert planner.output_a_target == pytest.approx(0.0)
  assert planner.a_desired == pytest.approx(0.0)

  planner.update(sm)
  assert planner.mpc.set_state_history[1][1] == pytest.approx(0.0)
  assert planner.mpc.x0[2] == pytest.approx(0.0)


def test_disengage_reengage_resets_stale_cruise_limiter_state(monkeypatch):
  planner = _planner()
  planner.a_cruise = -1.2
  disengaged_sm = _sm(long_control_state=LongCtrlState.off, a_ego=0.3)

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (10.0, -0.7))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: False)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (1.5, False))

  planner.update(disengaged_sm)

  assert planner.mpc.set_state_history[-1][1] == pytest.approx(0.3)
  assert planner.a_cruise > 0.0
  assert abs(planner.a_cruise - 0.3) <= get_max_accel(disengaged_sm["carState"].vEgo) * DT

  # controlsState remains off on the first re-engage planner cycle, so the MPC is
  # initialized from the current measured acceleration once more before entering PID.
  reengage_sm = _sm(long_control_state=LongCtrlState.off, a_ego=0.2)
  planner.update(reengage_sm)
  assert planner.mpc.set_state_history[-1][1] == pytest.approx(0.2)

  expected_active_state = planner.a_desired
  active_sm = _sm(long_control_state=LongCtrlState.pid, a_ego=0.2)
  planner.update(active_sm)
  assert planner.mpc.set_state_history[-1][1] == pytest.approx(expected_active_state)


def test_dec_acc_mode_does_not_add_e2e_candidate(monkeypatch):
  planner = _planner()
  sm = _sm(experimental_mode=True)
  sm["modelV2"].action.desiredAcceleration = -2.0

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (10.0, -0.7))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: False)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (1.5, False))

  planner.update(sm)

  assert planner.output_a_target == pytest.approx(0.0)
  assert planner.mpc.source == LongitudinalPlanSource.cruise


def test_dec_blended_adds_stopline_adjusted_e2e_candidate(monkeypatch):
  planner = _planner()
  sm = _sm(experimental_mode=True)
  sm["modelV2"].action.desiredAcceleration = 0.2
  sm["modelV2"].velocity.x[-1] = 0.0

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (20.0, -0.7))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: True)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (1.5, False))

  planner.update(sm)

  assert planner.output_a_target == pytest.approx(-0.3)
  assert planner.mpc.source == LongitudinalPlanSource.e2e


def test_should_stop_is_combined_from_all_candidates(monkeypatch):
  planner = _planner()
  sm = _sm(experimental_mode=True)
  sm["modelV2"].action.shouldStop = True

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (20.0, -0.7))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: True)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (0.0, False))

  planner.update(sm)

  assert planner.output_should_stop


def test_force_decel_requests_cruise_stop(monkeypatch):
  planner = _planner()
  sm = _sm()
  sm["controlsState"].forceDecel = True

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (20.0, -0.7))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: False)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (0.0, False))

  planner.update(sm)

  assert planner.output_should_stop
  assert planner.mpc.source == LongitudinalPlanSource.cruise


def test_sp_final_speed_target_drives_cruise_candidate(monkeypatch):
  planner = _planner()
  sm = _sm()

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (8.0, 0.8))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: False)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (1.0, False))

  planner.update(sm)

  assert planner.output_a_target < 0.0
  assert planner.mpc.source == LongitudinalPlanSource.cruise


def test_final_planner_output_never_exceeds_accel_max(monkeypatch):
  planner = _planner()
  sm = _sm(experimental_mode=True)
  sm["modelV2"].action.desiredAcceleration = 3.0

  monkeypatch.setattr(LongitudinalPlannerSP, "update", lambda self, sm: None)
  monkeypatch.setattr(LongitudinalPlannerSP, "update_targets", lambda *args, **kwargs: (30.0, 3.0))
  monkeypatch.setattr(LongitudinalPlanner, "is_e2e", lambda self, sm: True)
  monkeypatch.setattr(planner_module, "get_accel_from_plan", lambda *args, **kwargs: (3.0, False))

  planner.update(sm)

  assert planner.output_a_target == ACCEL_MAX
