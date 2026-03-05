"""
ResidualFFModel — NNLC v2 on-device inference class.

Physics baseline + neural residual feedforward torque predictor.
Designed to run on the comma 3X (Snapdragon 845).

Integration contract:
- predict() returns a single float (total torque, normalized)
- reset() clears temporal state (call on disengage)
- No external dependencies beyond numpy
- No file I/O after __init__
- NaN-safe: returns baseline if any input is NaN
- Deterministic: same inputs always produce same output
"""

import numpy as np


class ResidualFFModel:
  """V2 NNLC inference: physics baseline + neural residual."""

  def __init__(self, model_data: dict):
    assert model_data.get("version") == 2, "Model version must be 2"
    valid_archs = {"residual_ff_v2", "preview_residual_ff_v2"}
    assert model_data.get("architecture") in valid_archs, \
      f"Unknown architecture: {model_data.get('architecture')}"

    self.input_features = model_data["input_features"]

    norm = model_data["input_normalization"]
    self.input_mean = np.array(norm["mean"], dtype=np.float32)
    self.input_std = np.array(norm["std"], dtype=np.float32)

    self.weights = []
    self.biases = []
    self.activations = []

    pending_act = None
    for layer in model_data["layers"]:
      ltype = layer["type"]
      if ltype == "linear":
        self.weights.append(np.array(layer["weight"], dtype=np.float32))
        self.biases.append(np.array(layer["bias"], dtype=np.float32))
        self.activations.append(pending_act)
        pending_act = None
      elif ltype == "relu":
        pending_act = "relu"

    baseline = model_data["physics_baseline"]
    self.default_laf = float(baseline["lat_accel_factor_default"])
    self.default_friction = float(baseline["friction_coeff_default"])
    self.use_torqued = bool(baseline.get("use_torqued_params", True))

    self.residual_clamp = float(model_data.get("residual_clamp", 0.15))

    self.prev_lat_accel = 0.0
    self.prev_steer_angle = 0.0

  def predict(self, lat_accel, v_ego, steer_angle, steer_rate, roll, a_ego,
              lat_accel_factor=None, friction_coeff=None, extra_inputs=None):
    """
    Return total torque (float, normalized [-1, 1]).

    extra_inputs: optional list of preview features (future curvature/roll/yaw_rate).
    """
    inputs_raw = [lat_accel, v_ego, steer_angle, steer_rate, roll, a_ego]
    laf = lat_accel_factor if (self.use_torqued and lat_accel_factor is not None) else self.default_laf
    fric = friction_coeff if (self.use_torqued and friction_coeff is not None) else self.default_friction

    if any(not np.isfinite(x) for x in inputs_raw):
      lat_safe = lat_accel - 9.81 * np.sin(roll) if np.isfinite(lat_accel) and np.isfinite(roll) else 0.0
      return float(laf * lat_safe + fric * np.sign(lat_safe))

    lat_accel_corrected = lat_accel - 9.81 * np.sin(roll)
    baseline = laf * lat_accel_corrected + fric * float(np.sign(lat_accel_corrected))

    base_x = [
      lat_accel_corrected, v_ego, steer_angle, steer_rate,
      roll, a_ego, self.prev_lat_accel, self.prev_steer_angle,
    ]
    if extra_inputs:
      x = np.array(base_x + list(extra_inputs), dtype=np.float32)
    else:
      x = np.array(base_x, dtype=np.float32)
      n_expected = len(self.input_features)
      if n_expected > len(x):
        x = np.concatenate([x, np.zeros(n_expected - len(x), dtype=np.float32)])

    self.prev_lat_accel = float(lat_accel_corrected)
    self.prev_steer_angle = float(steer_angle)

    mean = self.input_mean[:len(x)]
    std = self.input_std[:len(x)]
    x = (x - mean) / (std + 1e-8)

    hidden = x
    for w, b, act in zip(self.weights, self.biases, self.activations):
      hidden = hidden @ w.T + b
      if act == "relu":
        hidden = np.maximum(hidden, 0.0)

    residual = float(np.clip(hidden[0], -self.residual_clamp, self.residual_clamp))
    return baseline + residual

  def reset(self):
    """Reset temporal state. Call on disengage -> re-engage."""
    self.prev_lat_accel = 0.0
    self.prev_steer_angle = 0.0
