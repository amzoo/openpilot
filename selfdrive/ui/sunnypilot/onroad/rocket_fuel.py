"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app

MAX_ACCEL = 4.0
BAR_HEIGHT = 125.0


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
    accel_norm = float(np.clip(sm['carState'].aEgo / MAX_ACCEL, -1.0, 1.0))
    self._accel_filter.update(accel_norm)
    self._alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))

    accel = self._accel_filter.x
    alpha = self._alpha_filter.x

    bar_x = rect.x + np.interp(abs(accel), [0.5, 1], [6 * scale, 8 * scale])
    bar_w = np.interp(abs(accel), [0.5, 1], [14 * scale, 56 * scale])
    bar_half_h = BAR_HEIGHT * scale / 2
    cy = rect.y + rect.height / 2

    # background track (white translucent, full height * alpha)
    bg_alpha = np.interp(abs(accel), [0.5, 1.0], [0.25, 0.5])
    if ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * alpha))
    else:
      bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * alpha))

    bg_h = bar_half_h * 2 * alpha
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, cy - bg_h / 2, bar_w, bg_h), 1.0, 8, bg_color)

    # foreground bar (grows from center toward tip)
    base_alpha = int(200 * alpha)
    if accel >= 0:
      fg_color = rl.Color(0, 245, 0, base_alpha)    # green for acceleration
    else:
      fg_color = rl.Color(245, 0, 0, base_alpha)    # red for braking
    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      fg_color = rl.Color(fg_color.r, fg_color.g, fg_color.b, int(255 * 0.35 * alpha))

    fg_h = bar_half_h * abs(accel) * alpha
    fg_y = cy - fg_h if accel >= 0 else cy
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, fg_y, bar_w, fg_h), 1.0, 8, fg_color)

    # center dot when near zero
    if abs(accel) < 0.5:
      dot_color = rl.Color(182, 182, 182, int(255 * 0.9 * alpha))
      rl.draw_circle(int(bar_x + bar_w / 2), int(cy), int(5 * scale), dot_color)
