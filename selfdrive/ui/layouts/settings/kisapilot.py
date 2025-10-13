from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import toggle_item, button_item, numeric_item, single_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
import os
import pyray as rl
import json
from pathlib import Path
import subprocess

BUTTON_HEIGHT = 90
BUTTON_PADDING = 10
HIGHLIGHT_COLOR = (34, 139, 34, 241)
DEFAULT_COLOR = (128, 128, 128, 255)
TEXT_COLOR = (255, 255, 255, 255)
TEXT_SIZE = 55
CORNER_RADIUS = 5

KISA_DIR = Path(__file__).parents[3] / "kisapilot"

SCHEMA_PATH = KISA_DIR / "param_schema.json"
CMD_SCHEMA_PATH = KISA_DIR / "cmd_schema.json"

with SCHEMA_PATH.open() as f:
  PARAM_SCHEMA = json.load(f)
with CMD_SCHEMA_PATH.open() as f:
  CMD_SCHEMA = {c["cmd"]: c for c in json.load(f)}

LANG = Params().get("LanguageSetting", return_default=True)

def get_text(v):
  if isinstance(v, dict):
    return v.get(LANG) or v.get("en")
  return str(v) if v is not None else ""

def get_special_texts(m):
  out = {}
  if isinstance(m, dict):
    for k, v in m.items():
      if isinstance(v, dict):
        out[k] = v.get(LANG) or v.get("en")
      else:
        out[k] = str(v)
  return out

def detect_ui_type(item):
  if item.get("value_type") == "BOOL":
    return "TOGGLE"
  if "min_value" in item:
    return "NUMERIC"
  return None

def run_command(cmd_key):
  cmd = CMD_SCHEMA.get(cmd_key)
  if not cmd:
    return

  def _exec():
    subprocess.Popen(cmd["exec"])

  if cmd.get("confirm", False):
    gui_app.set_modal_overlay(
      ConfirmDialog(cmd["description"][LANG], "OK"),
      callback=lambda result: _exec() if result == DialogResult.CONFIRM else None
      )
  else:
    _exec()

class KisaPilotLayout(Widget):
  def build_items_by_groups(self, group_names):
    if isinstance(group_names, str):
      group_names = [group_names]

    items = []

    for meta in PARAM_SCHEMA:
      if meta.get("group") not in group_names:
        continue

      ui_type = detect_ui_type(meta)

      # TOGGLE
      if ui_type == "TOGGLE":
        key = meta["param"]
        w = toggle_item(
          get_text(meta.get("title")),
          description=get_text(meta.get("description")),
          initial_state=self._params.get_bool(key),
          callback=lambda state, k=key: self._params.put_bool(k, state)
        )
        self._param_mapping.append((key, w))
        self._meta_mapping[w] = meta
        items.append(w)

      # NUMERIC
      elif ui_type == "NUMERIC":
        key = meta["param"]
        w = numeric_item(
          get_text(meta.get("title")),
          param_key=key,
          description=get_text(meta.get("description")),
          value_type=meta.get("value_type", "INT"),
          min_value=meta.get("min_value"),
          max_value=meta.get("max_value"),
          step=meta.get("step", 1),
          decimal_places=meta.get("decimal_places", 0),
          special_texts=get_special_texts(meta.get("special_texts"))
        )
        self._param_mapping.append((key, w))
        self._meta_mapping[w] = meta
        items.append(w)

    if "DEV" in group_names:
      for cmd_key, cmd in CMD_SCHEMA.items():
        if cmd_key not in ["onroad_refresh", "reboot"]:
          w = button_item(
            cmd["title"],
            "RUN",
            description=get_text(cmd.get("description")),
            callback=lambda c=cmd_key: run_command(c)
          )
          self._meta_mapping[w] = cmd
          items.append(w)

    return items

  def __init__(self):
    super().__init__()
    self._params = Params()

    self._select_car_dialog = None
    self._select_car_btn = single_button_item(
      lambda: self._params.get("CarName") or "Select Your Car",
      callback=self._show_car_selection_dialog
    )

    self._param_mapping = []
    self._meta_mapping = {}

    self.can_type = str(self._params.get("KisaCANType", return_default=True)).strip().upper()
    self.scc_type = str(self._params.get("KisaSCCType", return_default=True)).strip().upper()

    self._menu_titles = ["Str/UI", "Lat", "Lon", "Ga/bt", "Nv/Sf", "Dev"]
    self._menu_items = [
      [self._select_car_btn] + self.build_items_by_groups("UI/Start"),
      self.build_items_by_groups("LAT"),
      self.build_items_by_groups("LONG"),
      self.build_items_by_groups("GAP/BTN"),
      self.build_items_by_groups("Nav/Safety"),
      self.build_items_by_groups("Dev"),
    ]
    self._current_menu = 0

    self._scroller = Scroller(self._menu_items[self._current_menu], line_separator=True, spacing=0)
    self._press_offset_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)

    ui_state.add_offroad_transition_callback(self._update_items)

  def _set_menu(self, menu_index):
    self._current_menu = menu_index
    self._scroller._items = self._menu_items[menu_index]
    for item in self._scroller._items:
      item.set_touch_valid_callback(self._scroller.scroll_panel.is_touch_valid)
      item.show_event()
    self._scroller.scroll_panel.set_offset(0.0)
    self._update_items()

  def _handle_mouse_release(self, mouse_pos):
    total_width = self._rect.width
    button_width = (total_width - BUTTON_PADDING * (len(self._menu_titles) + 1)) / len(self._menu_titles)
    x = self._rect.x + BUTTON_PADDING
    y = self._rect.y + BUTTON_PADDING

    for idx, title in enumerate(self._menu_titles):
      btn_rect = rl.Rectangle(int(x), int(y), int(button_width), BUTTON_HEIGHT)
      if rl.check_collision_point_rec(mouse_pos, btn_rect):
        self._set_menu(idx)
        return True
      x += button_width + BUTTON_PADDING
    return False

  def _render(self, rect):
    self._rect = rect

    total_width = rect.width
    button_width = (total_width - BUTTON_PADDING * (len(self._menu_titles) + 1)) / len(self._menu_titles)
    x = rect.x + BUTTON_PADDING
    y = rect.y + BUTTON_PADDING

    for idx, title in enumerate(self._menu_titles):
      btn_rect = rl.Rectangle(int(x), int(y), int(button_width), BUTTON_HEIGHT)

      is_pressed = rl.is_mouse_button_down(rl.MOUSE_LEFT_BUTTON) and rl.check_collision_point_rec(rl.get_mouse_position(), btn_rect)
      target_offset = 5 if is_pressed else 0
      self._press_offset_filter.update(target_offset)
      btn_rect.y += self._press_offset_filter.x

      shadow_offset = 2 if is_pressed else 4
      shadow_alpha = 180 if is_pressed else 120
      shadow_rect = rl.Rectangle(btn_rect.x + shadow_offset, btn_rect.y + shadow_offset, btn_rect.width, btn_rect.height)
      rl.draw_rectangle_rounded(shadow_rect, CORNER_RADIUS, 8, (0, 0, 0, shadow_alpha))

      base_color = HIGHLIGHT_COLOR if idx == self._current_menu else DEFAULT_COLOR
      if is_pressed:
          base_color = (max(base_color[0]-80,0), max(base_color[1]-80,0), max(base_color[2]-80,0), base_color[3])
      rl.draw_rectangle_rounded(btn_rect, CORNER_RADIUS, 8, base_color)

      text_width = rl.measure_text(title, TEXT_SIZE)
      text_x = int(x + (button_width - text_width) / 2)
      text_y = int(btn_rect.y + (BUTTON_HEIGHT - TEXT_SIZE) / 2)
      rl.draw_text(title, text_x, text_y, TEXT_SIZE, TEXT_COLOR)

      x += button_width + BUTTON_PADDING

    scroll_rect = rl.Rectangle(rect.x, rect.y + BUTTON_HEIGHT + BUTTON_PADDING * 2, rect.width, rect.height - (BUTTON_HEIGHT + BUTTON_PADDING * 2))
    self._scroller.render(scroll_rect)

  def show_event(self):
    self._scroller.show_event()
    self._update_items()

  def _update_items(self):
    ui_state.update_params()
    for key, item in self._param_mapping:
      if hasattr(item, "action_item"):
        action = item.action_item
        if getattr(action, "value_type", None) == "BOOL":
          action.set_state(self._params.get_bool(key))
    for menu_items in self._menu_items:
      for item in menu_items:
        meta = self._meta_mapping.get(item, {})

        visible_condition_can = not meta.get("can_type") or meta["can_type"] == self.can_type
        visible_condition_scc = not meta.get("scc_type") or meta["scc_type"] == self.scc_type

        item.set_visible(visible_condition_can and visible_condition_scc)

  def _show_car_selection_dialog(self):
    car_path = "/data/params/d/CarList"

    try:
      with open(car_path, "r") as f:
        car_files = [line.strip() for line in f.readlines() if line.strip()]
    except Exception:
      car_files = []

    if not car_files:
      gui_app.set_modal_overlay(
        alert_dialog("No car files found.")
      )
      return

    cur = self._params.get("CarName")
    if isinstance(cur, (bytes, bytearray)):
      try:
        cur = cur.decode()
      except Exception:
        cur = str(cur)
    cur = cur or None

    def handle_car_selection(result: int):
      if result == 1 and self._select_car_dialog:
        selected_car = self._select_car_dialog.selection
        self._params.put("CarName", selected_car)
        self._select_car_dialog = None
        return
      else:
        if not cur:
          self._select_car_dialog = None
          return
        else:
          def confirm_delete(res):
            if res == DialogResult.CONFIRM:
              self._params.remove("CarName")
            self._select_car_dialog = None

          gui_app.set_modal_overlay(
            ConfirmDialog(f"Do you want to delete the current selection {cur} ?", "Delete"),
            callback=confirm_delete
          )
          self._select_car_dialog = None
          return

    self._select_car_dialog = MultiOptionDialog("Select Your Car", car_files, cur)
    gui_app.set_modal_overlay(self._select_car_dialog, callback=handle_car_selection)