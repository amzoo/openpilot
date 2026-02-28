import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.selfdrive.modeld.constants import ModelConstants

# Step 1: Physics-informed feedforward improvements
UNDERSTEER_GRADIENT = 0.003      # vehicle understeer gradient rad/(m/s²), RAV4 Prime estimate
EPS_ASSIST_V_EGO = [5.0, 15.0, 25.0, 35.0]  # m/s: EPS assist compensation breakpoints
EPS_ASSIST_GAIN  = [0.7, 0.85, 1.0, 1.15]   # unitless: higher at speed (less EPS assist)

# Step 2: Preview-based feedforward
PREVIEW_WEIGHT = 0.3   # fraction of feedforward from preview lateral accel (0=off, 1=full preview)
PREVIEW_ALPHA  = 0.15  # low-pass filter coefficient per 100 Hz cycle (~0.67s time constant)

# Step 3: Smooth friction compensation
FRICTION_SMOOTH_EPSILON = 0.05   # rad/s: tanh smoothing width (avoids sign discontinuity at zero-crossing)
FRICTION_VISCOUS        = 0.001  # (lat-accel·s/rad): viscous friction coefficient

# Feedforward clamp: reserve headroom for PI feedback
FF_CLAMP = 0.85  # max normalized feedforward (leaves 15% of authority for PI correction)

# Stiction (static friction breakaway) compensation
STICTION_MULTIPLIER     = 1.3    # stiction coefficient = coulomb × this (stiction > kinetic friction)
STICTION_THRESHOLD_RADS = math.radians(0.5)  # rad/s: below this rate, steering is considered stationary

# Backlash (worm gear dead zone) compensation
BACKLASH_WIDTH_DEG  = 0.2   # degrees: dead zone width — measure empirically, set 0 if not observed
BACKLASH_TORQUE     = 0.02  # normalized torque boost to push through backlash dead zone

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)
    self.preview_lat_accel_filtered = 0.0
    self.prev_steer_rate_rads = 0.0
    self.backlash_in_deadzone = False
    self.backlash_ref_angle_deg = 0.0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    # Preview control: blend current and future lat-accel for feedforward only (feedback error unchanged)
    ff_lat_accel = future_desired_lateral_accel
    if self.extension.model_valid and CS.vEgo > 3.0:
      preview_time = float(np.interp(CS.vEgo, [5.0, 15.0, 30.0], [0.5, 1.0, 2.0]))
      preview_raw = float(np.interp(preview_time, ModelConstants.T_IDXS, self.extension.model_v2.acceleration.y))
      self.preview_lat_accel_filtered = PREVIEW_ALPHA * preview_raw + (1 - PREVIEW_ALPHA) * self.preview_lat_accel_filtered
      # Only apply preview when it agrees in sign with the current request (suppresses on straights/reversals)
      if abs(future_desired_lateral_accel) > 0.05 and preview_raw * future_desired_lateral_accel > 0:
        ff_lat_accel = (1 - PREVIEW_WEIGHT) * future_desired_lateral_accel + PREVIEW_WEIGHT * self.preview_lat_accel_filtered
      else:
        self.preview_lat_accel_filtered = future_desired_lateral_accel
    else:
      self.preview_lat_accel_filtered = future_desired_lateral_accel

    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation  # original, for extension/logging
    ff = ff_lat_accel - roll_compensation  # preview-blended, for feedforward
    # Understeer gradient correction: compensates for growing sideslip at speed on curves
    ff += UNDERSTEER_GRADIENT * ff_lat_accel * CS.vEgo
    # Speed-dependent EPS assist compensation: more torque needed per unit lat-accel at highway speeds
    eps_assist_factor = float(np.interp(CS.vEgo, EPS_ASSIST_V_EGO, EPS_ASSIST_GAIN))
    ff *= eps_assist_factor
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    # Smooth friction model: uses actual steering rate rather than error proxy
    # tanh avoids the sign() discontinuity at zero-crossings; viscous term damps oscillation
    steer_rate_rads = math.radians(CS.steeringRateDeg)
    coulomb_friction = self.torque_params.friction * self.torque_params.latAccelFactor * math.tanh(steer_rate_rads / FRICTION_SMOOTH_EPSILON)
    viscous_friction = FRICTION_VISCOUS * steer_rate_rads
    # Stiction: extra breakaway torque when steering is nearly stationary and needs to overcome static friction
    stiction_extra = 0.0
    if abs(steer_rate_rads) < STICTION_THRESHOLD_RADS and abs(ff_lat_accel) > 0.05:
      stiction_extra = STICTION_MULTIPLIER * self.torque_params.friction * self.torque_params.latAccelFactor * math.copysign(1.0, ff_lat_accel)
    # Backlash: compensate for worm gear dead zone on direction reversals
    backlash_extra = 0.0
    direction_reversed = (steer_rate_rads * self.prev_steer_rate_rads < 0) and (abs(self.prev_steer_rate_rads) > math.radians(0.3))
    if direction_reversed:
      self.backlash_in_deadzone = True
      self.backlash_ref_angle_deg = CS.steeringAngleDeg
    if self.backlash_in_deadzone:
      if abs(CS.steeringAngleDeg - self.backlash_ref_angle_deg) >= BACKLASH_WIDTH_DEG:
        self.backlash_in_deadzone = False
      elif abs(ff_lat_accel) > 0.05:
        backlash_extra = BACKLASH_TORQUE * math.copysign(1.0, ff_lat_accel)
    self.prev_steer_rate_rads = steer_rate_rads
    friction_speed_scale = float(np.interp(CS.vEgo, [5.0, 15.0, 30.0], [0.6, 0.85, 1.0]))
    friction_total = (coulomb_friction + viscous_friction + stiction_extra + backlash_extra) * friction_speed_scale
    ff += friction_total

    # Clamp feedforward to leave headroom for PI feedback
    ff = float(np.clip(ff, -FF_CLAMP, FF_CLAMP))

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))
      pid_log.feedforwardOutput = float(ff)
      pid_log.previewLateralAccel = float(ff_lat_accel)
      pid_log.frictionCompensation = float(friction_total)
      pid_log.epsAssistFactor = float(eps_assist_factor)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
