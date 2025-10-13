import os
import pyray as rl
from collections.abc import Callable
from abc import ABC
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.toggle import Toggle, WIDTH as TOGGLE_WIDTH, HEIGHT as TOGGLE_HEIGHT
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType

import time
from openpilot.common.params import Params

ITEM_BASE_WIDTH = 600
ITEM_BASE_HEIGHT = 170
ITEM_PADDING = 20
ITEM_TEXT_FONT_SIZE = 50
ITEM_TEXT_COLOR = rl.WHITE
ITEM_TEXT_VALUE_COLOR = rl.Color(170, 170, 170, 255)
ITEM_DESC_TEXT_COLOR = rl.Color(128, 128, 128, 255)
ITEM_DESC_FONT_SIZE = 40
ITEM_DESC_V_OFFSET = 140
RIGHT_ITEM_PADDING = 20
ICON_SIZE = 80
BUTTON_WIDTH = 250
BUTTON_HEIGHT = 100
BUTTON_BORDER_RADIUS = 50
BUTTON_FONT_SIZE = 35
BUTTON_FONT_WEIGHT = FontWeight.MEDIUM

TEXT_PADDING = 20


def _resolve_value(value, default=""):
  if callable(value):
    return value()
  return value if value is not None else default


# Abstract base class for right-side items
class ItemAction(Widget, ABC):
  def __init__(self, width: int = BUTTON_HEIGHT, enabled: bool | Callable[[], bool] = True):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, width, 0))
    self._enabled_source = enabled

  def get_width_hint(self) -> float:
    # Return's action ideal width, 0 means use full width
    return self._rect.width

  def set_enabled(self, enabled: bool | Callable[[], bool]):
    self._enabled_source = enabled

  @property
  def enabled(self):
    return _resolve_value(self._enabled_source, False)


class ToggleAction(ItemAction):
  def __init__(self, initial_state: bool = False, width: int = TOGGLE_WIDTH, enabled: bool | Callable[[], bool] = True,
               callback: Callable[[bool], None] | None = None):
    super().__init__(width, enabled)
    self.toggle = Toggle(initial_state=initial_state, callback=callback)

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    self.toggle.set_touch_valid_callback(touch_callback)

  def _render(self, rect: rl.Rectangle) -> bool:
    self.toggle.set_enabled(self.enabled)
    clicked = self.toggle.render(rl.Rectangle(rect.x, rect.y + (rect.height - TOGGLE_HEIGHT) / 2, self._rect.width, TOGGLE_HEIGHT))
    return bool(clicked)

  def set_state(self, state: bool):
    self.toggle.set_state(state)

  def get_state(self) -> bool:
    return self.toggle.get_state()


class ButtonAction(ItemAction):
  def __init__(self, text: str | Callable[[], str], width: int = BUTTON_WIDTH, enabled: bool | Callable[[], bool] = True):
    super().__init__(width, enabled)
    self._text_source = text
    self._value_source: str | Callable[[], str] | None = None
    self._pressed = False
    self._font = gui_app.font(FontWeight.NORMAL)

    def pressed():
      self._pressed = True

    self._button = Button(
      self.text,
      font_size=BUTTON_FONT_SIZE,
      font_weight=BUTTON_FONT_WEIGHT,
      button_style=ButtonStyle.LIST_ACTION,
      border_radius=BUTTON_BORDER_RADIUS,
      click_callback=pressed,
      text_padding=0,
    )
    self.set_enabled(enabled)

  def get_width_hint(self) -> float:
    value_text = self.value
    if value_text:
      text_width = measure_text_cached(self._font, value_text, ITEM_TEXT_FONT_SIZE).x
      return text_width + BUTTON_WIDTH + TEXT_PADDING
    else:
      return BUTTON_WIDTH

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    self._button.set_touch_valid_callback(touch_callback)

  def set_text(self, text: str | Callable[[], str]):
    self._text_source = text

  def set_value(self, value: str | Callable[[], str]):
    self._value_source = value

  @property
  def text(self):
    return _resolve_value(self._text_source, tr("Error"))

  @property
  def value(self):
    return _resolve_value(self._value_source, "")

  def _render(self, rect: rl.Rectangle) -> bool:
    self._button.set_text(self.text)
    self._button.set_enabled(_resolve_value(self.enabled))
    button_rect = rl.Rectangle(rect.x + rect.width - BUTTON_WIDTH, rect.y + (rect.height - BUTTON_HEIGHT) / 2, BUTTON_WIDTH, BUTTON_HEIGHT)
    self._button.render(button_rect)

    value_text = self.value
    if value_text:
      value_rect = rl.Rectangle(rect.x, rect.y, rect.width - BUTTON_WIDTH - TEXT_PADDING, rect.height)
      gui_label(value_rect, value_text, font_size=ITEM_TEXT_FONT_SIZE, color=ITEM_TEXT_VALUE_COLOR,
                font_weight=FontWeight.NORMAL, alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
                alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE)

    # TODO: just use the generic Widget click callbacks everywhere, no returning from render
    pressed = self._pressed
    self._pressed = False
    return pressed


class TextAction(ItemAction):
  def __init__(self, text: str | Callable[[], str], color: rl.Color = ITEM_TEXT_COLOR, enabled: bool | Callable[[], bool] = True):
    self._text_source = text
    self.color = color

    self._font = gui_app.font(FontWeight.NORMAL)
    initial_text = _resolve_value(text, "")
    text_width = measure_text_cached(self._font, initial_text, ITEM_TEXT_FONT_SIZE).x
    super().__init__(int(text_width + TEXT_PADDING), enabled)

  @property
  def text(self):
    return _resolve_value(self._text_source, tr("Error"))

  def get_width_hint(self) -> float:
    text_width = measure_text_cached(self._font, self.text, ITEM_TEXT_FONT_SIZE).x
    return text_width + TEXT_PADDING

  def _render(self, rect: rl.Rectangle) -> bool:
    gui_label(self._rect, self.text, font_size=ITEM_TEXT_FONT_SIZE, color=self.color,
              font_weight=FontWeight.NORMAL, alignment=rl.GuiTextAlignment.TEXT_ALIGN_RIGHT,
              alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE)
    return False

  def set_text(self, text: str | Callable[[], str]):
    self._text_source = text


class DualButtonAction(ItemAction):
  def __init__(self, left_text: str | Callable[[], str], right_text: str | Callable[[], str], left_callback: Callable = None,
               right_callback: Callable = None, enabled: bool | Callable[[], bool] = True):
    super().__init__(width=0, enabled=enabled)  # Width 0 means use full width
    self.left_button = Button(left_text, click_callback=left_callback, button_style=ButtonStyle.NORMAL, text_padding=0)
    self.right_button = Button(right_text, click_callback=right_callback, button_style=ButtonStyle.DANGER, text_padding=0)

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    self.left_button.set_touch_valid_callback(touch_callback)
    self.right_button.set_touch_valid_callback(touch_callback)

  def _render(self, rect: rl.Rectangle):
    button_spacing = 30
    button_height = 120
    button_width = (rect.width - button_spacing) / 2
    button_y = rect.y + (rect.height - button_height) / 2

    left_rect = rl.Rectangle(rect.x, button_y, button_width, button_height)
    right_rect = rl.Rectangle(rect.x + button_width + button_spacing, button_y, button_width, button_height)

    # expand one to full width if other is not visible
    if not self.left_button.is_visible:
      right_rect.x = rect.x
      right_rect.width = rect.width
    elif not self.right_button.is_visible:
      left_rect.width = rect.width

    # Render buttons
    self.left_button.render(left_rect)
    self.right_button.render(right_rect)


class TripleButtonAction(ItemAction):
  def __init__(self, left_text: str, mid_text: str, right_text: str,
               left_callback: Callable = None, mid_callback: Callable = None, right_callback: Callable = None,
               enabled: bool | Callable[[], bool] = True):
    super().__init__(width=0, enabled=enabled)
    self.left_text = left_text
    self.mid_text = mid_text
    self.right_text = right_text

    self.left_button = Button(left_text, click_callback=left_callback, button_style=ButtonStyle.PRIMARY, text_padding=0)
    self.mid_button = Button(mid_text, click_callback=mid_callback, button_style=ButtonStyle.LIST_ACTION, text_padding=0)
    self.right_button = Button(right_text, click_callback=right_callback, button_style=ButtonStyle.DANGER, text_padding=0)

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    self.left_button.set_touch_valid_callback(touch_callback)
    self.mid_button.set_touch_valid_callback(touch_callback)
    self.right_button.set_touch_valid_callback(touch_callback)

  def _render(self, rect: rl.Rectangle):
    # spacing between buttons
    button_spacing = 20
    button_height = 120

    # compute available width and each button's width (default equal)
    # leave small spacing between three buttons: 2 * spacing
    total_spacing = button_spacing * 2
    base_width = (rect.width - total_spacing) / 3
    button_y = rect.y + (rect.height - button_height) / 2

    # initial rects
    left_rect = rl.Rectangle(rect.x, button_y, base_width, button_height)
    mid_rect = rl.Rectangle(rect.x + base_width + button_spacing, button_y, base_width, button_height)
    right_rect = rl.Rectangle(rect.x + 2 * (base_width + button_spacing), button_y, base_width, button_height)

    # Adjust for visibility:
    # If one of the buttons is not visible, expand the remaining buttons to occupy space.
    vis_left = self.left_button.is_visible
    vis_mid = self.mid_button.is_visible
    vis_right = self.right_button.is_visible

    visible_count = int(bool(vis_left)) + int(bool(vis_mid)) + int(bool(vis_right))
    if visible_count == 0:
      return  # nothing to draw

    # If only one visible -> take full width
    if visible_count == 1:
      if vis_left:
        left_rect.x = rect.x
        left_rect.width = rect.width
      elif vis_mid:
        mid_rect.x = rect.x
        mid_rect.width = rect.width
      else:
        right_rect.x = rect.x
        right_rect.width = rect.width
    # If two visible -> split width between them
    elif visible_count == 2:
      # find which two and place them side by side with one spacing
      pair_width = (rect.width - button_spacing) / 2
      if not vis_left:
        # mid + right
        mid_rect.x = rect.x
        mid_rect.width = pair_width
        right_rect.x = rect.x + pair_width + button_spacing
        right_rect.width = pair_width
      elif not vis_mid:
        # left + right
        left_rect.x = rect.x
        left_rect.width = pair_width
        right_rect.x = rect.x + pair_width + button_spacing
        right_rect.width = pair_width
      else:
        # left + mid
        left_rect.x = rect.x
        left_rect.width = pair_width
        mid_rect.x = rect.x + pair_width + button_spacing
        mid_rect.width = pair_width
    else:
      # all three visible: use computed base widths (already set)
      # but fix small rounding issues so they exactly fill rect.width
      left_rect.x = rect.x
      left_rect.width = base_width
      mid_rect.x = rect.x + base_width + button_spacing
      mid_rect.width = base_width
      right_rect.x = rect.x + 2 * (base_width + button_spacing)
      # last one take remaining width to avoid subpixel gaps
      right_rect.width = rect.x + rect.width - right_rect.x

    # set enabled state on buttons (ItemAction.enabled may be a bool or callable)
    enabled_flag = _resolve_value(self.enabled, True)
    try:
      # if enabled is a tuple/list of per-button flags, support that
      if isinstance(enabled_flag, (tuple, list)):
        left_enabled = bool(enabled_flag[0]) if len(enabled_flag) > 0 else True
        mid_enabled = bool(enabled_flag[1]) if len(enabled_flag) > 1 else True
        right_enabled = bool(enabled_flag[2]) if len(enabled_flag) > 2 else True
      else:
        left_enabled = mid_enabled = right_enabled = bool(enabled_flag)
    except Exception:
      left_enabled = mid_enabled = right_enabled = True

    self.left_button.set_enabled(left_enabled)
    self.mid_button.set_enabled(mid_enabled)
    self.right_button.set_enabled(right_enabled)

    # Render (only render visible ones)
    if vis_left:
      self.left_button.render(left_rect)
    if vis_mid:
      self.mid_button.render(mid_rect)
    if vis_right:
      self.right_button.render(right_rect)

class SingleButtonAction(ItemAction):
  def __init__(self, text: str | Callable[[], str], enabled: bool | Callable[[], bool] = True, callback: Callable | None = None):
    super().__init__(width=0, enabled=enabled)
    self._text_source = text
    self._pressed = False
    self._font = gui_app.font(FontWeight.MEDIUM)

    def pressed():
      self._pressed = True
      if callback:
        callback()

    self._button = Button(
      self.text,
      font_size=ITEM_TEXT_FONT_SIZE + 15,
      font_weight=BUTTON_FONT_WEIGHT,
      button_style=ButtonStyle.LIST_ACTION,
      border_radius=30,
      click_callback=pressed,
      text_padding=0,
    )
    self.set_enabled(enabled)

  @property
  def text(self):
    return _resolve_value(self._text_source, tr("Error"))

  def _render(self, rect: rl.Rectangle) -> bool:
    self._button.set_text(self.text)
    self._button.set_enabled(_resolve_value(self.enabled))

    button_height = 120
    button_rect = rl.Rectangle(
      rect.x + ITEM_PADDING,
      rect.y + (rect.height - button_height) / 2,
      rect.width - 2 * ITEM_PADDING,
      button_height
    )
    self._button.render(button_rect)

    pressed = self._pressed
    self._pressed = False
    return pressed

class MultipleButtonAction(ItemAction):
  def __init__(self, buttons: list[str | Callable[[], str]], button_width: int, selected_index: int = 0, callback: Callable = None):
    super().__init__(width=len(buttons) * button_width + (len(buttons) - 1) * RIGHT_ITEM_PADDING, enabled=True)
    self.buttons = buttons
    self.button_width = button_width
    self.selected_button = selected_index
    self.callback = callback
    self._font = gui_app.font(FontWeight.MEDIUM)

  def set_selected_button(self, index: int):
    if 0 <= index < len(self.buttons):
      self.selected_button = index

  def get_selected_button(self) -> int:
    return self.selected_button

  def _render(self, rect: rl.Rectangle):
    spacing = RIGHT_ITEM_PADDING
    button_y = rect.y + (rect.height - BUTTON_HEIGHT) / 2

    for i, _text in enumerate(self.buttons):
      button_x = rect.x + i * (self.button_width + spacing)
      button_rect = rl.Rectangle(button_x, button_y, self.button_width, BUTTON_HEIGHT)

      # Check button state
      mouse_pos = rl.get_mouse_position()
      is_pressed = rl.check_collision_point_rec(mouse_pos, button_rect) and self.enabled and self.is_pressed
      is_selected = i == self.selected_button

      # Button colors
      if is_selected:
        bg_color = rl.Color(51, 171, 76, 255)  # Green
      elif is_pressed:
        bg_color = rl.Color(74, 74, 74, 255)  # Dark gray
      else:
        bg_color = rl.Color(57, 57, 57, 255)  # Gray

      if not self.enabled:
        bg_color = rl.Color(bg_color.r, bg_color.g, bg_color.b, 150)  # Dim

      # Draw button
      rl.draw_rectangle_rounded(button_rect, 1.0, 20, bg_color)

      # Draw text
      text = _resolve_value(_text, "")
      text_size = measure_text_cached(self._font, text, 40)
      text_x = button_x + (self.button_width - text_size.x) / 2
      text_y = button_y + (BUTTON_HEIGHT - text_size.y) / 2
      text_color = rl.Color(228, 228, 228, 255) if self.enabled else rl.Color(150, 150, 150, 255)
      rl.draw_text_ex(self._font, text, rl.Vector2(text_x, text_y), 40, 0, text_color)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    spacing = RIGHT_ITEM_PADDING
    button_y = self._rect.y + (self._rect.height - BUTTON_HEIGHT) / 2
    for i, _ in enumerate(self.buttons):
      button_x = self._rect.x + i * (self.button_width + spacing)
      button_rect = rl.Rectangle(button_x, button_y, self.button_width, BUTTON_HEIGHT)
      if rl.check_collision_point_rec(mouse_pos, button_rect):
        self.selected_button = i
        if self.callback:
          self.callback(i)


class ListItem(Widget):
  def __init__(self, title: str | Callable[[], str] = "", icon: str | None = None, description: str | Callable[[], str] | None = None,
               description_visible: bool = False, callback: Callable | None = None,
               action_item: ItemAction | None = None):
    super().__init__()
    self._title = title
    self.set_icon(icon)
    self._description = description
    self.description_visible = description_visible
    self.callback = callback
    self.description_opened_callback: Callable | None = None
    self.action_item = action_item

    self.set_rect(rl.Rectangle(0, 0, ITEM_BASE_WIDTH, ITEM_BASE_HEIGHT))
    self._font = gui_app.font(FontWeight.NORMAL)

    self._html_renderer = HtmlRenderer(text="", text_size={ElementType.P: ITEM_DESC_FONT_SIZE},
                                       text_color=ITEM_DESC_TEXT_COLOR)
    self._parse_description(self.description)

    # Cached properties for performance
    self._prev_description: str | None = self.description

  def show_event(self):
    self._set_description_visible(False)

  def set_description_opened_callback(self, callback: Callable) -> None:
    self.description_opened_callback = callback

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    if self.action_item:
      self.action_item.set_touch_valid_callback(touch_callback)

  def set_parent_rect(self, parent_rect: rl.Rectangle):
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if not self.is_visible:
      return

    # Check not in action rect
    if self.action_item:
      action_rect = self.get_right_item_rect(self._rect)
      if rl.check_collision_point_rec(mouse_pos, action_rect):
        # Click was on right item, don't toggle description
        return

    self._set_description_visible(not self.description_visible)

  def _set_description_visible(self, visible: bool):
    if self.description and self.description_visible != visible:
      self.description_visible = visible
      # do callback first in case receiver changes description
      if self.description_visible and self.description_opened_callback is not None:
        self.description_opened_callback()
        # Call _update_state to catch any description changes
        self._update_state()

      content_width = int(self._rect.width - ITEM_PADDING * 2)
      self._rect.height = self.get_item_height(self._font, content_width)

  def _update_state(self):
    # Detect changes if description is callback
    new_description = self.description
    if new_description != self._prev_description:
      self._parse_description(new_description)

  def _render(self, _):
    if not self.is_visible:
      return

    # Don't draw items that are not in parent's viewport
    if ((self._rect.y + self.rect.height) <= self._parent_rect.y or
      self._rect.y >= (self._parent_rect.y + self._parent_rect.height)):
      return

    content_x = self._rect.x + ITEM_PADDING
    text_x = content_x

    # Only draw title and icon for items that have them
    if self.title:
      # Draw icon if present
      if self.icon:
        rl.draw_texture(self._icon_texture, int(content_x), int(self._rect.y + (ITEM_BASE_HEIGHT - self._icon_texture.height) // 2), rl.WHITE)
        text_x += ICON_SIZE + ITEM_PADDING

      # Draw main text
      text_size = measure_text_cached(self._font, self.title, ITEM_TEXT_FONT_SIZE)
      item_y = self._rect.y + (ITEM_BASE_HEIGHT - text_size.y) // 2
      rl.draw_text_ex(self._font, self.title, rl.Vector2(text_x, item_y), ITEM_TEXT_FONT_SIZE, 0, ITEM_TEXT_COLOR)

    # Draw description if visible
    if self.description_visible:
      content_width = int(self._rect.width - ITEM_PADDING * 2)
      description_height = self._html_renderer.get_total_height(content_width)
      description_rect = rl.Rectangle(
        self._rect.x + ITEM_PADDING,
        self._rect.y + ITEM_DESC_V_OFFSET,
        content_width,
        description_height
      )
      self._html_renderer.render(description_rect)

    # Draw right item if present
    if self.action_item:
      right_rect = self.get_right_item_rect(self._rect)
      right_rect.y = self._rect.y
      if self.action_item.render(right_rect) and self.action_item.enabled:
        # Right item was clicked/activated
        if self.callback:
          self.callback()

  def set_icon(self, icon: str | None):
    self.icon = icon
    self._icon_texture = gui_app.texture(os.path.join("icons", self.icon), ICON_SIZE, ICON_SIZE) if self.icon else None

  def set_description(self, description: str | Callable[[], str] | None):
    self._description = description

  def _parse_description(self, new_desc):
    self._html_renderer.parse_html_content(new_desc)
    self._prev_description = new_desc

  @property
  def title(self):
    return _resolve_value(self._title, "")

  @property
  def description(self):
    return _resolve_value(self._description, "")

  def get_item_height(self, font: rl.Font, max_width: int) -> float:
    if not self.is_visible:
      return 0

    height = float(ITEM_BASE_HEIGHT)
    if self.description_visible:
      description_height = self._html_renderer.get_total_height(max_width)
      height += description_height - (ITEM_BASE_HEIGHT - ITEM_DESC_V_OFFSET) + ITEM_PADDING
    return height

  def get_right_item_rect(self, item_rect: rl.Rectangle) -> rl.Rectangle:
    if not self.action_item:
      return rl.Rectangle(0, 0, 0, 0)

    right_width = self.action_item.get_width_hint()
    if right_width == 0:  # Full width action (like DualButtonAction)
      return rl.Rectangle(item_rect.x + ITEM_PADDING, item_rect.y,
                          item_rect.width - (ITEM_PADDING * 2), ITEM_BASE_HEIGHT)

    # Clip width to available space, never overlapping this Item's title
    content_width = item_rect.width - (ITEM_PADDING * 2)
    title_width = measure_text_cached(self._font, self.title, ITEM_TEXT_FONT_SIZE).x
    right_width = min(content_width - title_width, right_width)

    right_x = item_rect.x + item_rect.width - right_width
    right_y = item_rect.y
    return rl.Rectangle(right_x, right_y, right_width, ITEM_BASE_HEIGHT)


class NumericStepperAction(ItemAction):
  def __init__(self, param_key: str, value_type: str = "STR", step: float = 1,
               min_value: float | None = None, max_value: float | None = None,
               decimal_places: int = 0, button_width: int = 120, enabled: bool | Callable[[], bool] = True,
               special_texts: dict[int | float, str] | None = None):
    total_width = button_width * 2 + 140
    super().__init__(width=total_width, enabled=enabled)

    self.param_key = param_key
    self.value_type = (value_type or "STR").upper()
    self.step = step
    self.min_value = min_value
    self.max_value = max_value
    self.decimal_places = int(decimal_places)
    self.button_width = button_width
    self.special_texts = special_texts

    self.params = Params()
    self._cached_value = None

    self._last_update_left = 0
    self._last_update_right = 0
    self._repeat_interval = 0.005
    self._left_handled = False
    self._right_handled = False

    def _left_cb():
      pass

    def _right_cb():
      pass

    self.left_button = Button("-", click_callback=_left_cb, button_style=ButtonStyle.LIST_ACTION)
    self.right_button = Button("+", click_callback=_right_cb, button_style=ButtonStyle.LIST_ACTION)
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _read_raw(self):
    try:
      if self.value_type == "BOOL":
        return self.params.get_bool(self.param_key)
      raw = self.params.get(self.param_key)
      if raw is None:
        return None
      if self.value_type == "INT":
        return int(float(raw))
      if self.value_type == "FLOAT":
        return float(raw)
      return raw
    except Exception:
      return None

  def _write_raw(self, value):
    if value is not None:
      if self.min_value is not None:
        value = max(value, self.min_value)
      if self.max_value is not None:
        value = min(value, self.max_value)
    try:
      if self.value_type == "BOOL":
        if hasattr(self.params, "put_bool"):
          self.params.put_bool(self.param_key, bool(value))
        else:
          self.params.put(self.param_key, "1" if bool(value) else "0")
      elif self.value_type == "INT":
        self.params.put(self.param_key, int(round(value)))
      elif self.value_type == "FLOAT":
        fval = round(float(value), self.decimal_places)
        self.params.put(self.param_key, fval)
      else:
        self.params.put(self.param_key, str(value))
      return value
    except Exception as e:
      print(f"[NumericStepperAction] write error for {self.param_key}: {e}")
      return None

  def _get_value_for_display(self):
    if self._cached_value is not None:
      return self._cached_value
    raw = self._read_raw()
    self._cached_value = raw
    if raw is None:
      if self.value_type == "BOOL":
        return False
      if self.value_type == "INT":
        return 0
      if self.value_type == "FLOAT":
        return 0.0
      return ""
    if self.value_type == "BOOL":
      return bool(raw)
    if self.value_type == "INT":
      return int(raw)
    if self.value_type == "FLOAT":
      return float(raw)
    return raw

  def _format_display(self, v):
    if self.special_texts:
      for k, txt in self.special_texts.items():
        if abs(v - float(k)) < 1e-6:
          return txt

    if self.value_type in ("INT", "BOOL"):
      return str(int(v))

    if self.value_type == "FLOAT":
      if self.decimal_places > 0:
        s = f"{v:.{self.decimal_places}f}"
        return s.rstrip("0").rstrip(".")
      return repr(float(v))

    return str(v)

  def _render(self, rect: rl.Rectangle) -> bool:
    spacing = 12
    btn_h = BUTTON_HEIGHT if BUTTON_HEIGHT < rect.height else int(rect.height * 0.7)
    btn_y = rect.y + (rect.height - btn_h) / 2

    right_rect = rl.Rectangle(rect.x + rect.width - self.button_width, btn_y, self.button_width, btn_h)
    left_rect = rl.Rectangle(right_rect.x - spacing - self.button_width, btn_y, self.button_width, btn_h)

    value_area_right = left_rect.x - spacing
    value_area_left = rect.x + ITEM_PADDING
    value_area_w = max(20, value_area_right - value_area_left)
    value_area = rl.Rectangle(value_area_left, rect.y, value_area_w, rect.height)

    enabled_flag = _resolve_value(self.enabled, True)
    self.left_button.set_enabled(enabled_flag)
    self.right_button.set_enabled(enabled_flag)

    self.left_button.render(left_rect)
    self.right_button.render(right_rect)

    now = time.time()

    if self.left_button.is_pressed:
      if not self._left_handled:
        cur = self._get_value_for_display()
        if self.value_type == "BOOL":
          newv = not bool(cur)
        elif self.value_type == "INT":
          newv = int(cur) - int(self.step)
        elif self.value_type == "FLOAT":
          newv = float(cur) - float(self.step)
        else:
          newv = cur
        value = self._write_raw(newv)
        self._cached_value = value
        self._last_update_left = now
        self._last_press_left = now
        self._left_handled = True
      elif now - self._last_update_left > self._repeat_interval and now - self._last_press_left > 1.0:
        cur = self._get_value_for_display()
        if self.value_type == "BOOL":
          newv = not bool(cur)
        elif self.value_type == "INT":
          newv = int(cur) - int(self.step)
        elif self.value_type == "FLOAT":
          newv = float(cur) - float(self.step)
        else:
          newv = cur
        value = self._write_raw(newv)
        self._cached_value = value
        self._last_update_left = now
      else:
        cur = self._get_value_for_display()
    else:
      self._left_handled = False
      cur = self._get_value_for_display()

    if self.right_button.is_pressed:
      if not self._right_handled:
        cur = self._get_value_for_display()
        if self.value_type == "BOOL":
          newv = not bool(cur)
        elif self.value_type == "INT":
          newv = int(cur) + int(self.step)
        elif self.value_type == "FLOAT":
          newv = float(cur) + float(self.step)
        else:
          newv = cur
        value = self._write_raw(newv)
        self._cached_value = value
        self._last_update_right = now
        self._last_press_right = now
        self._right_handled = True
      elif now - self._last_update_right > self._repeat_interval and now - self._last_press_right > 1.0:
        cur = self._get_value_for_display()
        if self.value_type == "BOOL":
          newv = not bool(cur)
        elif self.value_type == "INT":
          newv = int(cur) + int(self.step)
        elif self.value_type == "FLOAT":
          newv = float(cur) + float(self.step)
        else:
          newv = cur
        value = self._write_raw(newv)
        self._cached_value = value
        self._last_update_right = now
      else:
        cur = self._get_value_for_display()
    else:
      self._right_handled = False
      cur = self._get_value_for_display()

    disp = self._format_display(cur)
    if self.special_texts:
      disp = self.special_texts.get(str(cur), disp)

    text_size = measure_text_cached(self._font, disp, ITEM_TEXT_FONT_SIZE)
    text_x = value_area.x + (value_area.width - text_size.x)
    text_y = rect.y + (rect.height - text_size.y) / 2
    rl.draw_text_ex(self._font, disp, rl.Vector2(text_x, text_y), ITEM_TEXT_FONT_SIZE, 0, ITEM_TEXT_VALUE_COLOR)

    return False


def numeric_item(title: str, param_key: str, value_type: str = "INT", step: float = 1,
                 min_value: float | None = None, max_value: float | None = None,
                 decimal_places: int = 0, enabled: bool | Callable[[], bool] = True,
                 description: str | Callable[[], str] | None = None,
                 special_texts: dict[int | float, str] | None = None) -> ListItem:
  action = NumericStepperAction(param_key, value_type=value_type, step=step,
                                min_value=min_value, max_value=max_value, decimal_places=decimal_places,
                                button_width=120, enabled=enabled, special_texts=special_texts)
  return ListItem(title=title, description=description, action_item=action)



# Factory functions
def simple_item(title: str | Callable[[], str], callback: Callable | None = None) -> ListItem:
  return ListItem(title=title, callback=callback)


def toggle_item(title: str | Callable[[], str], description: str | Callable[[], str] | None = None, initial_state: bool = False,
                callback: Callable | None = None, icon: str = "", enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = ToggleAction(initial_state=initial_state, enabled=enabled, callback=callback)
  return ListItem(title=title, description=description, action_item=action, icon=icon)


def button_item(title: str | Callable[[], str], button_text: str | Callable[[], str], description: str | Callable[[], str] | None = None,
                callback: Callable | None = None, enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = ButtonAction(text=button_text, enabled=enabled)
  return ListItem(title=title, description=description, action_item=action, callback=callback)


def text_item(title: str | Callable[[], str], value: str | Callable[[], str], description: str | Callable[[], str] | None = None,
              callback: Callable | None = None, enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = TextAction(text=value, color=ITEM_TEXT_VALUE_COLOR, enabled=enabled)
  return ListItem(title=title, description=description, action_item=action, callback=callback)


def dual_button_item(left_text: str | Callable[[], str], right_text: str | Callable[[], str], left_callback: Callable = None, right_callback: Callable = None,
                     description: str | Callable[[], str] | None = None, enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = DualButtonAction(left_text, right_text, left_callback, right_callback, enabled)
  return ListItem(title="", description=description, action_item=action)


def triple_button_item(left_text: str, mid_text: str, right_text: str,
                       left_callback: Callable = None, mid_callback: Callable = None, right_callback: Callable = None,
                       description: str | Callable[[], str] | None = None,
                       enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = TripleButtonAction(left_text, mid_text, right_text, left_callback, mid_callback, right_callback, enabled)
  return ListItem(title="", description=description, action_item=action)

def single_button_item(button_text: str | Callable[[], str],
                       callback: Callable | None = None, enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = SingleButtonAction(button_text, enabled=enabled, callback=callback)
  return ListItem(title="", action_item=action)

def multiple_button_item(title: str | Callable[[], str], description: str | Callable[[], str], buttons: list[str | Callable[[], str]], selected_index: int,
                         button_width: int = BUTTON_WIDTH, callback: Callable = None, icon: str = ""):
  action = MultipleButtonAction(buttons, button_width, selected_index, callback=callback)
  return ListItem(title=title, description=description, icon=icon, action_item=action)
