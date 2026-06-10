"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.lead_predictive_controller import (
  LeadPredictiveController, _GAP_RATIO_MIN, _LEAD_DECEL_TH, _MIN_LEAD_PROB,
)

LeadPredictiveState = custom.LongitudinalPlanSP.SmartCruiseControl.LeadPredictiveState

# A wide gap relative to the standard-personality safe follow distance, valid for all personalities.
V_EGO = MIN_V + 10.0
V_CRUISE = 25.0


def _wide_gap() -> float:
  # 2x the relaxed safe distance is comfortably "wide" for every personality.
  return 2.0 * _GAP_RATIO_MIN * get_safe_obstacle_distance(V_EGO, get_T_FOLLOW(log.LongitudinalPersonality.relaxed))


def generate_radarState(status=True, d_rel=140.0, v_lead=15.0, a_lead=-1.5, model_prob=0.9):
  rs = messaging.new_message('radarState')
  lead = rs.radarState.leadOne
  lead.status = status
  lead.dRel = float(d_rel)
  lead.vLead = float(v_lead)
  lead.vRel = float(v_lead - V_EGO)
  lead.aLeadK = float(a_lead)
  lead.aLeadTau = 1.5
  lead.modelProb = float(model_prob)
  return rs


class TestLeadPredictiveController:

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartLongitudinalAnticipation", True, block=True)
    self.ctrl = LeadPredictiveController()
    self.sm = {'radarState': generate_radarState().radarState}

  def _run(self, sm=None, long_enabled=True, long_override=False, v_ego=V_EGO, a_ego=0.0,
           v_cruise=V_CRUISE, personality=log.LongitudinalPersonality.standard, steps=int(10. / DT_MDL)):
    sm = sm if sm is not None else self.sm
    for _ in range(steps):
      self.ctrl.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise, personality)

  def test_initial_state(self):
    assert self.ctrl.state == LeadPredictiveState.disabled
    assert not self.ctrl.is_active
    assert self.ctrl.output_v_target == V_CRUISE_UNSET
    assert self.ctrl.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartLongitudinalAnticipation", False, block=True)
    self.ctrl.enabled = False
    self._run()
    assert self.ctrl.state == LeadPredictiveState.disabled
    assert not self.ctrl.is_active
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_long_disabled(self):
    self._run(long_enabled=False)
    assert self.ctrl.state == LeadPredictiveState.disabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_no_lead_stays_enabled(self):
    self.sm = {'radarState': generate_radarState(status=False).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert not self.ctrl.is_active
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_lead_accelerating_stays_enabled(self):
    self.sm = {'radarState': generate_radarState(a_lead=1.0).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_lead_decel_wide_gap_coasts(self):
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap(), a_lead=-1.5).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.coasting
    assert self.ctrl.is_active
    assert MIN_V <= self.ctrl.output_v_target < V_CRUISE
    assert self.ctrl.output_a_target == 0.  # a_ego passthrough

  def test_lead_decel_tight_gap_stays_enabled(self):
    # Gap below the wide-gap ratio: let the MPC handle close following / braking.
    tight = 0.5 * _GAP_RATIO_MIN * get_safe_obstacle_distance(V_EGO, get_T_FOLLOW(log.LongitudinalPersonality.standard))
    self.sm = {'radarState': generate_radarState(d_rel=tight, a_lead=-1.5).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_low_model_prob_stays_enabled(self):
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap(), model_prob=_MIN_LEAD_PROB - 0.1).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_below_min_speed_stays_enabled(self):
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap()).radarState}
    self._run(v_ego=MIN_V - 1.0)
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_override(self):
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap()).radarState}
    self._run(long_override=True)
    assert self.ctrl.state == LeadPredictiveState.overriding
    assert not self.ctrl.is_active
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_decel_just_below_threshold_inactive(self):
    # aLeadK just above the decel threshold (less negative) must not trigger.
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap(), a_lead=_LEAD_DECEL_TH + 0.1).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_hysteresis_relaxes_out_not_snap(self):
    # Enter coasting on a decelerating lead at a wide gap.
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap(), a_lead=-1.5).radarState}
    self._run()
    assert self.ctrl.state == LeadPredictiveState.coasting

    # Lead stops decelerating: one step later we must still be coasting (no abrupt snap to UNSET).
    self.sm = {'radarState': generate_radarState(d_rel=_wide_gap(), a_lead=0.0).radarState}
    self.ctrl.update(self.sm, True, False, V_EGO, 0.0, V_CRUISE)
    assert self.ctrl.state == LeadPredictiveState.coasting
    assert self.ctrl.output_v_target < V_CRUISE

    # After the filter relaxes back toward v_cruise, it returns to enabled.
    self._run()
    assert self.ctrl.state == LeadPredictiveState.enabled
    assert self.ctrl.output_v_target == V_CRUISE_UNSET

  def test_personality_scales_anticipation(self):
    # Relaxed anticipates over a longer horizon -> targets a lower (earlier) speed than aggressive.
    sm = {'radarState': generate_radarState(d_rel=_wide_gap(), a_lead=-1.5).radarState}

    relaxed = LeadPredictiveController()
    aggressive = LeadPredictiveController()
    for _ in range(int(10. / DT_MDL)):
      relaxed.update(sm, True, False, V_EGO, 0.0, V_CRUISE, log.LongitudinalPersonality.relaxed)
      aggressive.update(sm, True, False, V_EGO, 0.0, V_CRUISE, log.LongitudinalPersonality.aggressive)

    assert relaxed.is_active and aggressive.is_active
    assert relaxed.output_v_target < aggressive.output_v_target
