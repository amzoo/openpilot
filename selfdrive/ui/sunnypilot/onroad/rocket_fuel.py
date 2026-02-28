"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts, TORQUE_ANGLE_SPAN
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon

MAX_ACCEL = 4.0
ACCEL_ANGLE_SPAN = TORQUE_ANGLE_SPAN  # re-use the same 12.7° span


class RocketFuel:
  def __init__(self):
    self._accel_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def render(self, rect: rl.Rectangle, sm) -> None:
    if not ui_state.rocket_fuel:
      return

    # normalize [-1, 1]
    accel_norm = float(np.clip(sm['carState'].aEgo / MAX_ACCEL, -1.0, 1.0))
    self._accel_filter.update(accel_norm)
    self._alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))

    accel = self._accel_filter.x
    alpha = self._alpha_filter.x

    # geometry — same radius/thickness math as TorqueBar
    accel_line_radius = 1200.0 * 3.0  # match sunnypilot scale=3.0
    accel_line_offset = np.interp(abs(accel), [0.5, 1], [22 * 3, 26 * 3])
    accel_line_height = np.interp(abs(accel), [0.5, 1], [14 * 3, 56 * 3])

    cx = rect.x - accel_line_radius + accel_line_offset
    cy = rect.y + rect.height / 2
    mid_r = accel_line_radius + accel_line_height / 2

    center_angle = 0.0
    bg_span = alpha * ACCEL_ANGLE_SPAN
    bg_start = center_angle - bg_span / 2
    bg_end = center_angle + bg_span / 2

    # background track (white translucent)
    bg_alpha = np.interp(abs(accel), [0.5, 1.0], [0.25, 0.5])
    if ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * alpha))
    else:
      bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * alpha))
    bg_pts = arc_bar_pts(cx, cy, mid_r, accel_line_height, bg_start, bg_end, cap_radius=7 * 3)
    draw_polygon(rect, bg_pts, color=bg_color)

    # foreground bar
    a0s = center_angle
    a1s = center_angle - bg_span / 2 * accel  # negative angles = up
    fg_pts = arc_bar_pts(cx, cy, mid_r, accel_line_height, a0s, a1s, cap_radius=7 * 3)

    # vertical gradient (center y → 65% toward bar tip)
    start_grad_y = cy / rect.height
    if accel < 0:
      end_grad_y = (cy * (1 - 0.65) + max(bg_pts[:, 1]) * 0.65) / rect.height
    else:
      end_grad_y = (cy * (1 - 0.65) + min(bg_pts[:, 1]) * 0.65) / rect.height

    base_alpha = int(200 * alpha)
    if accel >= 0:
      fg_color = rl.Color(0, 245, 0, base_alpha)    # green for acceleration
    else:
      fg_color = rl.Color(245, 0, 0, base_alpha)    # red for braking
    if ui_state.status not in (UIStatus.ENGAGED, UIStatus.LAT_ONLY):
      fg_color = rl.Color(fg_color.r, fg_color.g, fg_color.b, int(255 * 0.35 * alpha))

    draw_polygon(rect, fg_pts, color=fg_color)

    # center dot when near zero
    if abs(accel) < 0.5:
      dot_x = rect.x + accel_line_offset + accel_line_height / 2
      rl.draw_circle(int(dot_x), int(cy), 10 // 2 * 3, rl.Color(182, 182, 182, int(255 * 0.9 * alpha)))
