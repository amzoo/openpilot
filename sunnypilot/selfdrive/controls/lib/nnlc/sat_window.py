"""
SATCenteredWindow — Volvo patent US20180001927A1 torque window centering.

Instead of clipping output to [-1, 1] centered on zero, shift the window
center to the estimated Self-Aligning Torque (SAT).  This gives the
controller more authority for curve-holding without increasing the total
authority budget.

Safety design:
- max_sat_shift caps how far the window can shift (start at 0.0 = disabled)
- Low-pass filter prevents rapid SAT shifts from causing jerky steering
- Final CAN clamp [-1, 1] is always applied AFTER this (in the caller)
- If SAT estimate is garbage, max_sat_shift = 0.0 makes this a no-op
"""

from __future__ import annotations

import numpy as np


class SATCenteredWindow:
    """
    Shift the torque authority window toward the estimated SAT.

    Parameters
    ----------
    max_sat_shift : float
        Maximum allowed window shift.  0.0 = disabled (pass-through).
    filter_alpha  : float
        Low-pass smoothing factor per control step (~0.05 @ 100 Hz gives
        ~0.5 s time constant).
    authority     : float
        Half-width of the torque authority window (default 1.0 → [-1, 1]).
    """

    _HARD_CAP = 0.4  # absolute maximum shift regardless of input

    def __init__(
        self,
        max_sat_shift: float = 0.0,
        filter_alpha: float = 0.05,
        authority: float = 1.0,
    ) -> None:
        self.max_sat_shift: float = max(0.0, min(float(max_sat_shift), self._HARD_CAP))
        self.filter_alpha: float = float(filter_alpha)
        self.authority: float = float(authority)
        self.sat_filtered: float = 0.0
        self.enabled: bool = self.max_sat_shift > 0.0

    def apply(self, raw_output: float, sat_estimate: float) -> float:
        """
        Clip raw_output to the SAT-shifted authority window.

        When disabled (max_sat_shift == 0.0), returns raw_output unchanged.
        """
        if not self.enabled:
            return raw_output

        sat_clamped = float(np.clip(sat_estimate, -self.max_sat_shift, self.max_sat_shift))
        self.sat_filtered += self.filter_alpha * (sat_clamped - self.sat_filtered)

        lower = -self.authority + self.sat_filtered
        upper = self.authority + self.sat_filtered

        return float(np.clip(raw_output, lower, upper))

    def reset(self) -> None:
        """Reset filtered SAT state. Call on disengage → re-engage."""
        self.sat_filtered = 0.0

    def set_max_shift(self, value: float) -> None:
        """Allow runtime adjustment. Hard cap at _HARD_CAP."""
        self.max_sat_shift = max(0.0, min(float(value), self._HARD_CAP))
        self.enabled = self.max_sat_shift > 0.0
