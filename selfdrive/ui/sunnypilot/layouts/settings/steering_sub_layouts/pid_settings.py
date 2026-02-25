"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import option_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller


class PidSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._kp_slider = option_item_sp(
      title=lambda: tr("Proportional Gain (kP)"),
      param="PidKpV",
      description=lambda: tr("Controls how aggressively the steering corrects for lateral error."),
      min_value=1,
      max_value=200,
      value_change_step=1,
      label_callback=lambda x: f"{x / 100:.2f}",
      use_float_scaling=True,
    )

    self._ki_slider = option_item_sp(
      title=lambda: tr("Integral Gain (kI)"),
      param="PidKiV",
      description=lambda: tr("Controls how quickly accumulated steering error is corrected over time."),
      min_value=1,
      max_value=100,
      value_change_step=1,
      label_callback=lambda x: f"{x / 100:.2f}",
      use_float_scaling=True,
    )

    self._kf_slider = option_item_sp(
      title=lambda: tr("Feedforward (kF)"),
      param="PidKf",
      description=lambda: tr("Feedforward gain applied based on desired steering angle."),
      min_value=1,
      max_value=500,
      value_change_step=1,
      label_callback=lambda x: f"{x / 1000000:.6f}",
      use_float_scaling=True,
    )

    return [self._kp_slider, self._ki_slider, self._kf_slider]

  def _update_state(self):
    super()._update_state()
    offroad = ui_state.is_offroad()
    self._kp_slider.action_item.set_enabled(offroad)
    self._ki_slider.action_item.set_enabled(offroad)
    self._kf_slider.action_item.set_enabled(offroad)

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()
