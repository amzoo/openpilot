"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app

MAX_ACCEL = 4.0
BAR_HEIGHT = 135.0


def _lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
  if x <= x0:
    return y0
  if x >= x1:
    return y1
  return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


class RocketFuel:
  def __init__(self, scale: float = 1.0):
    self._scale = scale
    self._accel_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def render(self, rect: rl.Rectangle, sm) -> None:
    if not ui_state.rocket_fuel:
      return

    scale = self._scale

    # normalize [-1, 1]
    accel_norm = max(-1.0, min(1.0, sm['carState'].aEgo / MAX_ACCEL))
    self._accel_filter.update(accel_norm)

    active = ui_state.status != UIStatus.DISENGAGED
    engaged = ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)
    self._alpha_filter.update(active)

    accel = self._accel_filter.x
    alpha = self._alpha_filter.x

    abs_accel = abs(accel)
    bar_x = rect.x + _lerp(abs_accel, 0.5, 1, 20 * scale, 22 * scale)
    bar_w = _lerp(abs_accel, 0.5, 1, 14 * scale, 56 * scale)
    bar_half_h = BAR_HEIGHT * scale / 2
    # Center between speed limit bottom (rect.y + 255) and face icon top (rect.y + rect.height - 222)
    cy = rect.y + (rect.height + 33) / 2

    # background track
    bg_alpha = _lerp(abs_accel, 0.5, 1.0, 0.25, 0.5)
    if engaged:
      bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * alpha))
    else:
      bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * alpha))

    bg_h = bar_half_h * 2 * alpha
    cap_r = 7 * scale
    bg_roundness = min(1.0, cap_r / (min(bar_w, bg_h) / 2)) if bg_h > 0 else 1.0
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, cy - bg_h / 2, bar_w, bg_h), bg_roundness, 8, bg_color)

    # foreground bar: solid color interpolated white → green/red based on accel magnitude
    fg_alpha = int(255 * 0.9 * alpha)
    if not engaged:
      fg_alpha = int(255 * 0.35 * alpha)

    # max height: fg top cap concentric with bg top cap; min height: circle (bar_w)
    fg_h_max = bar_half_h * alpha + bar_w / 2
    fg_h = _lerp(abs_accel, 0, 1, bar_w, fg_h_max)

    fade = int(255 * max(0.0, 1.0 - abs_accel / 0.3))  # white → full color by 0.3
    if accel >= 0:
      fg_color = rl.Color(fade, 245, fade, fg_alpha)
      fg_y = cy - fg_h + bar_w / 2
    else:
      fg_color = rl.Color(245, fade, fade, fg_alpha)
      fg_y = cy - bar_w / 2

    fg_roundness = min(1.0, cap_r / (min(bar_w, fg_h) / 2)) if fg_h > 0 else 1.0
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, fg_y, bar_w, fg_h), fg_roundness, 8, fg_color)

    # center dot (matches torque bar)
    if abs_accel < 0.5:
      dot_alpha = int(255 * 0.9 * alpha)
      rl.draw_circle(int(bar_x + bar_w / 2), int(cy), 5 * scale, rl.Color(182, 182, 182, dot_alpha))

