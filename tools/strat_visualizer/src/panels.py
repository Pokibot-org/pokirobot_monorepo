"""Sidebar panels and the masonry-packing Sidebar container.

Each panel implements `get_height`, `draw`, and `handle_event`. Sidebar lays
them out vertically (right orientation) or masonry-packed (bottom orientation)
and dispatches events to whichever panel was hit.
"""
import math

import pygame as pg

from constants import (
    COLOR_DEEPBLACK, COLOR_GRAY, COLOR_TEAM_BLUE, COLOR_TEAM_YELLOW,
    SIDEBAR_ACCENT, SIDEBAR_BG, SIDEBAR_DIM, SIDEBAR_FG,
)
from key_bindings import KeyBindings


class Panel:
    title: str = ""
    # Minimum column width (in px) this panel needs to render reasonably.
    # The sidebar caps n_cols so this constraint is always honored.
    min_width: int = 200

    def get_height(self, width: int, font: "pg.font.Font") -> int:
        raise NotImplementedError

    def draw(self, surface, rect, font):
        raise NotImplementedError

    def handle_event(self, event, rect) -> bool:
        return False


class Slider:
    def __init__(self, min_v, max_v, value, step=None, fmt=lambda v: f"{v:g}"):
        self.min_v = min_v
        self.max_v = max_v
        self.value = value
        self.step = step
        self.fmt = fmt
        self.rect = pg.Rect(0, 0, 0, 0)
        self._dragging = False

    def _set_from_x(self, x):
        if self.rect.w <= 0:
            return
        t = (x - self.rect.x) / self.rect.w
        t = max(0.0, min(1.0, t))
        v = self.min_v + t * (self.max_v - self.min_v)
        if self.step:
            v = round(v / self.step) * self.step
        self.value = max(self.min_v, min(self.max_v, v))

    def draw(self, surface, rect, font, on_color=SIDEBAR_ACCENT):
        self.rect = pg.Rect(rect)
        x, y, w, h = rect
        track_y = y + h // 2
        pg.draw.line(surface, SIDEBAR_DIM, (x, track_y), (x + w, track_y), 2)
        t = 0.0 if self.max_v == self.min_v else (self.value - self.min_v) / (self.max_v - self.min_v)
        thumb_x = int(x + t * w)
        pg.draw.circle(surface, on_color, (thumb_x, track_y), 6)

    def handle_event(self, event) -> bool:
        if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
            hit_rect = self.rect.inflate(0, 16)
            if hit_rect.collidepoint(event.pos):
                self._dragging = True
                self._set_from_x(event.pos[0])
                return True
        elif event.type == pg.MOUSEMOTION and self._dragging:
            self._set_from_x(event.pos[0])
            return True
        elif event.type == pg.MOUSEBUTTONUP and event.button == 1 and self._dragging:
            self._dragging = False
            return True
        return False


class SnapPanel(Panel):
    title = "Snap"

    def __init__(self, planner):
        self.planner = planner
        self.row_h = 26
        self.GRID_OPTIONS = [25.0, 50.0, 100.0]
        try:
            init_idx = self.GRID_OPTIONS.index(planner.grid_size)
        except ValueError:
            init_idx = 1
        self.grid_slider = Slider(0, len(self.GRID_OPTIONS) - 1, init_idx, step=1,
                                  fmt=lambda v: f"{int(self.GRID_OPTIONS[int(v)])} mm")
        init_x = max(1, round(math.pi / planner.angle_step)) if planner.angle_step > 0 else 4
        self.angle_slider = Slider(1, 24, init_x, step=1, fmt=lambda v: f"π/{int(v)}")
        self._toggle_rects = {}

    def get_height(self, width, font):
        return self.row_h * 3 + 8

    def draw(self, surface, rect, font):
        x, y, w, _ = rect
        title_surf = font.render(self.title, True, SIDEBAR_ACCENT)
        surface.blit(title_surf, (x, y))
        y += self.row_h

        self._draw_row(surface, font, x, y, w, "grid", self.planner.grid_enabled,
                       self.grid_slider, f"{int(self.planner.grid_size)} mm")
        y += self.row_h
        angle_x = max(1, round(math.pi / self.planner.angle_step)) if self.planner.angle_step > 0 else 1
        self._draw_row(surface, font, x, y, w, "angle", self.planner.angle_enabled,
                       self.angle_slider, f"π/{angle_x}")

    def _draw_row(self, surface, font, x, y, w, name, enabled, slider, value_text):
        box = pg.Rect(x, y + 4, 14, 14)
        pg.draw.rect(surface, SIDEBAR_FG, box, width=1)
        if enabled:
            pg.draw.rect(surface, SIDEBAR_ACCENT, box.inflate(-4, -4))
        self._toggle_rects[name] = box

        label = font.render(name, True, SIDEBAR_FG)
        surface.blit(label, (x + 22, y + 2))

        value = font.render(value_text, True, SIDEBAR_DIM)
        surface.blit(value, (x + w - value.get_width(), y + 2))

        slider_rect = pg.Rect(x + 80, y + 4, w - 80 - value.get_width() - 8, 14)
        slider.draw(surface, slider_rect, font,
                    on_color=SIDEBAR_ACCENT if enabled else SIDEBAR_DIM)

    def handle_event(self, event, rect) -> bool:
        if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
            for name, r in self._toggle_rects.items():
                if r.collidepoint(event.pos):
                    if name == "grid":
                        self.planner.grid_enabled = not self.planner.grid_enabled
                    elif name == "angle":
                        self.planner.angle_enabled = not self.planner.angle_enabled
                    return True
        if self.grid_slider.handle_event(event):
            idx = max(0, min(len(self.GRID_OPTIONS) - 1, int(self.grid_slider.value)))
            self.planner.grid_size = self.GRID_OPTIONS[idx]
            return True
        if self.angle_slider.handle_event(event):
            x = max(1, int(self.angle_slider.value))
            self.planner.angle_step = math.pi / x
            return True
        return False


class PlanPanel(Panel):
    title = "Path planner"
    min_width = 240  # four buttons (Export/Play/Reset/Clear) need horizontal room

    def __init__(self, planner, simulator, on_preview):
        self.planner = planner
        self.simulator = simulator
        self.on_preview = on_preview
        self.row_h = 20
        self.btn_h = 28
        self.status = ""
        self._buttons = []

    def _do_export(self):
        text = self.planner.to_c_string()
        self.on_preview(text)
        self.status = "preview opened"

    def _play_pause(self):
        self.simulator.play_pause(self.planner.waypoints)

    def _reset_sim(self):
        self.simulator.reset(self.planner.waypoints)

    def get_height(self, width, font):
        return self.row_h * 4 + self.btn_h + 12

    def draw(self, surface, rect, font):
        x, y, w, _ = rect
        self._buttons = []

        title_surf = font.render(self.title, True, SIDEBAR_ACCENT)
        surface.blit(title_surf, (x, y))
        y += self.row_h

        count_surf = font.render(f"waypoints: {len(self.planner.waypoints)}", True, SIDEBAR_FG)
        surface.blit(count_surf, (x, y))
        y += self.row_h

        hint = font.render("drag pt = move, shift+drag = rotate", True, SIDEBAR_DIM)
        surface.blit(hint, (x, y))
        y += self.row_h

        if self.status:
            status_surf = font.render(self.status, True, SIDEBAR_DIM)
            surface.blit(status_surf, (x, y))
        y += self.row_h

        sim_label = "Pause" if self.simulator.running else "Play"
        labels = [("Export", self._do_export), (sim_label, self._play_pause),
                  ("Reset", self._reset_sim), ("Clear", self.planner.clear)]
        btn_w = (w - 8) // len(labels)
        for i, (label, cb) in enumerate(labels):
            bx = x + i * (btn_w + 4)
            br = pg.Rect(bx, y, btn_w, self.btn_h)
            pg.draw.rect(surface, COLOR_GRAY, br, border_radius=4)
            pg.draw.rect(surface, SIDEBAR_FG, br, width=1, border_radius=4)
            txt = font.render(label, True, SIDEBAR_FG)
            surface.blit(txt, (br.centerx - txt.get_width() // 2, br.centery - txt.get_height() // 2))
            self._buttons.append((br, label, cb))

    def handle_event(self, event, rect) -> bool:
        if event.type != pg.MOUSEBUTTONDOWN or event.button != 1:
            return False
        for br, _label, cb in self._buttons:
            if br.collidepoint(event.pos):
                cb()
                return True
        return False


class KeybindingsPanel(Panel):
    title = "Keybindings"

    def __init__(self, bindings: KeyBindings):
        self.bindings = bindings
        self.row_h = 20

    def get_height(self, width, font):
        return self.row_h * (len(self.bindings.docs()) + 1) + 8

    def draw(self, surface, rect, font):
        x, y, w, _ = rect
        title_surf = font.render(self.title, True, SIDEBAR_ACCENT)
        surface.blit(title_surf, (x, y))
        y += self.row_h
        for display, label, repeat in self.bindings.docs():
            suffix = " (hold)" if repeat else ""
            key_surf = font.render(f"[{display}]{suffix}", True, SIDEBAR_FG)
            label_surf = font.render(label, True, SIDEBAR_DIM)
            surface.blit(key_surf, (x, y))
            surface.blit(label_surf, (x + 110, y))
            y += self.row_h


class InputsPanel(Panel):
    title = "Inputs"

    def __init__(self, entries):
        # entries: list of (label, callable -> str)
        self.entries = entries
        self.row_h = 20

    def get_height(self, width, font):
        return self.row_h * (len(self.entries) + 1) + 8

    def draw(self, surface, rect, font):
        x, y, w, _ = rect
        title_surf = font.render(self.title, True, SIDEBAR_ACCENT)
        surface.blit(title_surf, (x, y))
        y += self.row_h
        for label, getter in self.entries:
            try:
                value = getter()
            except Exception as exc:
                value = f"<err: {exc}>"
            label_surf = font.render(label, True, SIDEBAR_DIM)
            value_surf = font.render(str(value), True, SIDEBAR_FG)
            surface.blit(label_surf, (x, y))
            surface.blit(value_surf, (x + 110, y))
            y += self.row_h


class SidePanel(Panel):
    title = "Side"

    def __init__(self, planner, on_toggle):
        self.planner = planner
        self.on_toggle = on_toggle
        self.row_h = 22
        self.btn_h = 28
        self._buttons = []

    def get_height(self, width, font):
        return self.row_h + self.btn_h + 8

    def draw(self, surface, rect, font):
        x, y, w, _ = rect
        self._buttons = []
        title = font.render(self.title, True, SIDEBAR_ACCENT)
        surface.blit(title, (x, y))
        y += self.row_h
        items = [("yellow", COLOR_TEAM_YELLOW), ("blue", COLOR_TEAM_BLUE)]
        bw = (w - 8) // 2
        for i, (val, color) in enumerate(items):
            br = pg.Rect(x + i * (bw + 8), y, bw, self.btn_h)
            active = self.planner.side == val
            pg.draw.rect(surface, color if active else COLOR_GRAY, br, border_radius=4)
            pg.draw.rect(surface, SIDEBAR_FG, br, width=1, border_radius=4)
            text_color = COLOR_DEEPBLACK if active else SIDEBAR_FG
            txt = font.render(val, True, text_color)
            surface.blit(txt, (br.centerx - txt.get_width() // 2, br.centery - txt.get_height() // 2))
            self._buttons.append((br, val))

    def handle_event(self, event, rect) -> bool:
        if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
            for br, val in self._buttons:
                if br.collidepoint(event.pos):
                    if val != self.planner.side:
                        self.on_toggle(val)
                    return True
        return False


class Sidebar:
    def __init__(self, panels, padding=12, gap=10):
        self.panels = panels
        self.padding = padding
        self.gap = gap
        self._font = None
        self._rect = (0, 0, 0, 0)
        self._orientation = "right"
        self._panel_rects = []

    def _ensure_font(self):
        if self._font is None:
            self._font = pg.font.SysFont("monospace", 14)

    def add_panel(self, panel: Panel):
        self.panels.append(panel)

    def set_rect(self, rect, orientation):
        self._rect = rect
        self._orientation = orientation

    def draw(self, screen):
        self._ensure_font()
        x, y, w, h = self._rect
        pg.draw.rect(screen, SIDEBAR_BG, pg.Rect(x, y, w, h))
        self._panel_rects = []
        if not self.panels:
            return

        if self._orientation == "right":
            inner_x = x + self.padding
            inner_w = w - 2 * self.padding
            cy = y + self.padding
            for panel in self.panels:
                ph = panel.get_height(inner_w, self._font)
                prect = (inner_x, cy, inner_w, ph)
                panel.draw(screen, prect, self._font)
                self._panel_rects.append((panel, prect))
                cy += ph + self.gap
        else:  # bottom: masonry — flow panels into the shortest column
            inner_w = w - 2 * self.padding
            min_col_w = max(p.min_width for p in self.panels)
            n_cols = max(1, min(len(self.panels),
                                (inner_w + self.gap) // (min_col_w + self.gap)))
            col_w = (inner_w - self.gap * (n_cols - 1)) // n_cols
            col_y = [y + self.padding] * n_cols
            for panel in self.panels:
                c = min(range(n_cols), key=lambda i: col_y[i])
                ph = panel.get_height(col_w, self._font)
                cx = x + self.padding + c * (col_w + self.gap)
                prect = (cx, col_y[c], col_w, ph)
                panel.draw(screen, prect, self._font)
                self._panel_rects.append((panel, prect))
                col_y[c] += ph + self.gap

    def handle_event(self, event) -> bool:
        for panel, prect in self._panel_rects:
            if panel.handle_event(event, prect):
                return True
        return False

    def contains(self, pos):
        x, y, w, h = self._rect
        return x <= pos[0] < x + w and y <= pos[1] < y + h
