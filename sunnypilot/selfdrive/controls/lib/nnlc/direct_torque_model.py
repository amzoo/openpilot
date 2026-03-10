"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

On-device inference for v3 DirectTorqueModel (direct_torque_v3).

Mirrors scripts/sim_model.py::V3SimController._predict().
No output scaling — model outputs real-car torque convention directly.
"""
import numpy as np


class DirectTorqueModel:
  """
  V3 NNLC inference: direct torque with FiLM conditioning + preview features.

  Contract:
  - predict() returns float in [-1, 1]
  - reset() clears temporal state (call on disengage)
  - No file I/O after __init__
  - NaN-safe: returns 0.0 on any NaN input
  """

  def __init__(self, model_data: dict) -> None:
    assert model_data.get("version") == 3
    assert model_data.get("architecture") == "direct_torque_v3"

    self.s_mean = np.array(model_data["state_normalization"]["mean"],        dtype=np.float32)
    self.s_std  = np.array(model_data["state_normalization"]["std"],         dtype=np.float32)
    self.p_mean = np.array(model_data["preview_normalization"]["mean"],      dtype=np.float32)
    self.p_std  = np.array(model_data["preview_normalization"]["std"],       dtype=np.float32)
    self.c_mean = np.array(model_data["conditioning_normalization"]["mean"], dtype=np.float32)
    self.c_std  = np.array(model_data["conditioning_normalization"]["std"],  dtype=np.float32)

    self.weights = [np.array(l["weight"], dtype=np.float32) for l in model_data["layers"]]
    self.biases  = [np.array(l["bias"],   dtype=np.float32) for l in model_data["layers"]]

    film = model_data["film_layers"]
    self.film_gw = np.array(film["gamma_weight"], dtype=np.float32)
    self.film_gb = np.array(film["gamma_bias"],   dtype=np.float32)
    self.film_bw = np.array(film["beta_weight"],  dtype=np.float32)
    self.film_bb = np.array(film["beta_bias"],    dtype=np.float32)

    self.speed_gate_eps = float(model_data.get("speed_gate_eps", 0.0))

    # Fallback conditioning values (model trained with laf=1.7, friction=0.14)
    baseline = model_data.get("physics_baseline") or {}
    self.default_laf      = float(baseline.get("lat_accel_factor_default", 1.7))
    self.default_friction = float(baseline.get("friction_coeff_default",   0.14))

    # Temporal state (lag-1 feature)
    self.prev_lat_accel = 0.0

  def _forward(self, state: np.ndarray, preview: np.ndarray, cond: np.ndarray) -> float:
    """Numpy forward pass — matches DirectTorqueModel.forward() in PyTorch."""
    sn = (state   - self.s_mean) / (self.s_std   + 1e-8)
    pn = (preview - self.p_mean) / (self.p_std   + 1e-8)
    cn = (cond    - self.c_mean) / (self.c_std   + 1e-8)

    x = np.concatenate([sn, pn])
    h = np.maximum(0, x @ self.weights[0].T + self.biases[0])      # fc1 + ReLU
    gamma = cn @ self.film_gw.T + self.film_gb + 1.0               # FiLM γ
    beta  = cn @ self.film_bw.T + self.film_bb                     # FiLM β
    h = gamma * h + beta
    h = np.maximum(0, h @ self.weights[1].T + self.biases[1])      # fc2 + ReLU
    h = np.maximum(0, h @ self.weights[2].T + self.biases[2])      # fc3 + ReLU
    torque = float((h @ self.weights[3].T + self.biases[3])[0])    # fc4 linear
    return float(np.clip(torque, -1.0, 1.0))

  def predict(self,
              desired_lat_accel: float,       # physics convention (positive = right)
              v_ego: float,
              steer_angle_deg: float,
              steer_rate_deg: float,
              roll: float,
              a_ego: float,
              lat_accel_corrected: float,     # from livePose, physics convention
              yaw_rate: float,                # -livePose.angularVelocityDevice.z
              preview_features: list,         # 12 values from _get_preview_features()
              lat_accel_factor: float | None = None,
              friction_coeff: float | None = None,
              ) -> float:
    """Returns normalized torque in [-1, 1]."""
    if any(np.isnan(x) for x in [desired_lat_accel, v_ego, steer_angle_deg,
                                  steer_rate_deg, roll, a_ego,
                                  lat_accel_corrected, yaw_rate]):
      return 0.0

    state = np.array([
      desired_lat_accel,
      v_ego,
      steer_angle_deg,
      steer_rate_deg,
      roll,
      a_ego,
      lat_accel_corrected,
      yaw_rate,
      self.prev_lat_accel,                            # lag-1
      steer_angle_deg * steer_rate_deg / 1000.0,      # rate product
    ], dtype=np.float32)

    self.prev_lat_accel = lat_accel_corrected

    preview = np.array(preview_features, dtype=np.float32)  # 12 values

    laf  = lat_accel_factor if lat_accel_factor is not None else self.default_laf
    fric = friction_coeff   if friction_coeff   is not None else self.default_friction
    cond = np.array([laf, fric, 0.0, v_ego], dtype=np.float32)  # lat_accel_offset=0.0

    torque = self._forward(state, preview, cond)

    # Fixed linear speed ramp: torque → 0 as v → 0, full at v ≥ 5 m/s.
    # The learned speed_gate_eps collapsed to 0.160 in v3.5 training
    # (gate ≈ 0.997 at v=3 m/s — effectively disabled). The linear ramp
    # provides the correct physics-motivated attenuation without relying
    # on the learned gate value.
    torque *= min(1.0, v_ego / 5.0)

    return torque

  def reset(self) -> None:
    self.prev_lat_accel = 0.0
