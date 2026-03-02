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
from openpilot.system.ui.lib.shader_polygon import draw_rounded_rect

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

    abs_accel = abs(accel)
    bar_x = rect.x + np.interp(abs_accel, [0.5, 1], [6 * scale, 8 * scale])
    bar_w = np.interp(abs_accel, [0.5, 1], [14 * scale, 56 * scale])
    bar_half_h = BAR_HEIGHT * scale / 2
    cy = rect.y + rect.height / 2

    # background track: fades out as foreground bar fills in
    bg_fade = 1.0 - abs_accel
    bg_alpha = np.interp(abs_accel, [0.5, 1.0], [0.25, 0.5])
    if ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * bg_fade * alpha))
    else:
      bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * bg_fade * alpha))

    bg_h = bar_half_h * 2 * alpha
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, cy - bg_h / 2, bar_w, bg_h), 1.0, 8, bg_color)

    # foreground bar: solid color interpolated white → green/red based on accel magnitude
    fg_alpha = int(200 * alpha)
    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      fg_alpha = int(255 * 0.35 * alpha)

    # foreground bar: 2*scale gap on each side (matches torque bar dot gap)
    fg_w = bar_w - 4 * scale
    fg_x = bar_x + 2 * scale
    # max height: fg top cap concentric with bg top cap; min height: circle (fg_w)
    fg_h_max = bar_half_h * alpha + fg_w / 2
    fg_h = float(np.interp(abs_accel, [0, 1], [fg_w, fg_h_max]))

    fade = int(255 * (1.0 - abs_accel))  # 255 = white, 0 = full color
    if accel >= 0:
      fg_color = rl.Color(fade, 245, fade, fg_alpha)
      fg_y = cy - fg_h + fg_w / 2
    else:
      fg_color = rl.Color(245, fade, fade, fg_alpha)
      fg_y = cy - fg_w / 2

    draw_rounded_rect(rect, rl.Rectangle(fg_x, fg_y, fg_w, fg_h), 1.0, color=fg_color)


