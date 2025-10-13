import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

class KisaButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._kisa_mode: bool = False

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_kisa: rl.Texture = gui_app.texture('addon/img/kisapilot.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    self._kisa_mode = False

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = not self._kisa_mode
      # self._params.put_bool("Test", new_mode)

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

      ui_state.rec_status = not ui_state.rec_status
      if ui_state.rec_status:
        Params().put_bool_nonblocking("RecordingRunning", True)
      else:
        Params().put_bool_nonblocking("RecordingRunning", False)

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed else 255

    if ui_state.rec_status:
      ring_color  = rl.Color(255, 0, 0, 100)
    else:
      ring_color  = self._black_bg

    radius = self._rect.width / 2
    ring_thickness = 25

    rl.draw_circle(center_x, center_y, radius, ring_color)
    rl.draw_circle(center_x, center_y, radius - ring_thickness, self._black_bg)

    texture = self._txt_kisa if self._held_or_actual_mode() else self._txt_kisa
    rl.draw_texture(texture, center_x - texture.width // 2, center_y - texture.height // 2, self._white_color)

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._kisa_mode

  def _is_toggle_allowed(self):
    return True
    # if not self._params.get_bool("Test"):
    #   return False

