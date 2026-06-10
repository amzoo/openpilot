"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom, log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

LeadPredictiveState = custom.LongitudinalPlanSP.SmartCruiseControl.LeadPredictiveState

ACTIVE_STATES = (LeadPredictiveState.coasting,)
ENABLED_STATES = (LeadPredictiveState.enabled, LeadPredictiveState.overriding, *ACTIVE_STATES)

_MIN_LEAD_PROB = 0.5  # reject low confidence / ghost leads
_LEAD_DECEL_TH = -0.5  # m/s^2. Only act on a meaningfully decelerating lead, ignore accel noise.
_GAP_RATIO_MIN = 1.5  # only coast early when the gap is well beyond the MPC's binding follow distance
_T_AHEAD_BASE = 2.0  # s. Anticipation horizon base, added to t_follow (personality scaled).
_PRED_DECAY = 0.12  # 1/s. Gentler lead-decel decay than the MPC, so we anticipate a sustained slowdown.
_MIN_BENEFIT = 0.5  # m/s. Hysteresis: only emit a target that meaningfully lowers v_cruise.
_FILTER_RC = 1.0  # s. Smoothing time constant on the output target.


class LeadPredictiveController:
  v_ego: float = 0.
  a_ego: float = 0.
  v_cruise: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartLongitudinalAnticipation")

    self.state = LeadPredictiveState.disabled
    self.v_filter = FirstOrderFilter(V_CRUISE_UNSET, _FILTER_RC, DT_MDL)
    self.lead_decel = 0.
    self.v_lead_future = 0.
    self.v_target = V_CRUISE_UNSET

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return self.v_target

    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartLongitudinalAnticipation")

  def _gates_pass(self, lead, t_follow: float) -> bool:
    if not self.enabled or not self.long_enabled or self.long_override:
      return False
    # MPC owns low-speed / stopping behaviour.
    if self.v_ego <= MIN_V:
      return False
    if not lead.status or lead.modelProb < _MIN_LEAD_PROB:
      return False
    # Only anticipate a meaningfully decelerating lead.
    if lead.aLeadK > _LEAD_DECEL_TH:
      return False
    # Only ADD early coasting when the gap is wide. When tight, step aside and let the MPC brake.
    if lead.dRel < _GAP_RATIO_MIN * get_safe_obstacle_distance(self.v_ego, t_follow):
      return False
    # Nothing to anticipate if we are not trying to close on the set speed.
    if self.v_cruise <= self.v_ego - 0.5:
      return False
    return True

  def _update_calculations(self, lead, t_follow: float) -> float:
    self.lead_decel = float(lead.aLeadK)

    # Anticipate the lead's near-future flow speed. We decay the deceleration more gently than the
    # MPC (which forgets it within ~1s), so a sustained slowdown is reflected in the target.
    t_ahead = _T_AHEAD_BASE + t_follow
    a_lead_eff = lead.aLeadK * np.exp(-_PRED_DECAY * t_ahead)
    self.v_lead_future = max(0.0, lead.vLead + a_lead_eff * t_ahead)

    # Purely additive: cap at v_cruise (never accelerate), floor at MIN_V (never command creep/stop).
    return float(np.clip(self.v_lead_future, MIN_V, self.v_cruise))

  def _update_state_machine(self, lead, t_follow: float) -> tuple[bool, bool]:
    gates = self._gates_pass(lead, t_follow)

    if gates:
      v_target_raw = self._update_calculations(lead, t_follow)
    else:
      self.lead_decel = 0.
      self.v_lead_future = 0.
      # Relax the filtered target back toward v_cruise so we never snap (which would feel like a surge).
      v_target_raw = self.v_cruise

    self.v_target = self.v_filter.update(v_target_raw)
    lowers_target = self.v_target < self.v_cruise - _MIN_BENEFIT

    # OVERRIDING / DISABLED have priority in any non-disabled state.
    if self.state != LeadPredictiveState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = LeadPredictiveState.disabled
      elif self.long_override:
        self.state = LeadPredictiveState.overriding

      # ENABLED
      elif self.state == LeadPredictiveState.enabled:
        if gates and lowers_target:
          self.state = LeadPredictiveState.coasting

      # COASTING
      elif self.state == LeadPredictiveState.coasting:
        # Leave only once gates fail AND the filtered target has relaxed back to v_cruise.
        if not gates and not lowers_target:
          self.state = LeadPredictiveState.enabled

      # OVERRIDING
      elif self.state == LeadPredictiveState.overriding:
        if not self.long_override:
          self.state = LeadPredictiveState.enabled

    # DISABLED
    elif self.long_enabled and self.enabled:
      self.state = LeadPredictiveState.overriding if self.long_override else LeadPredictiveState.enabled

    if self.state == LeadPredictiveState.disabled:
      # Keep the filter primed at v_cruise so re-enabling does not produce a stale relax-out.
      self.v_filter.x = self.v_cruise

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise: float, personality=log.LongitudinalPersonality.standard) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise = v_cruise

    self._update_params()

    t_follow = get_T_FOLLOW(personality)
    lead = sm['radarState'].leadOne

    self.is_enabled, self.is_active = self._update_state_machine(lead, t_follow)

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
