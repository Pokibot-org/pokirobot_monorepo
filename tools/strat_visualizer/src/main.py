#!/usr/bin/env python
import os

from numpy.__config__ import show
from numpy.typing import NDArray
from shapely.geometry.base import BaseGeometry
from shapely.lib import box

import pygame as pg
import threading
import json
import math
import shutil
import subprocess
from msm import MqttSimMessengerServer, MqttSimMessengerNode
import time
import logging
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from dataclasses import dataclass, field

logger = logging.getLogger("strat_visu")

main_dir = os.path.split(os.path.abspath(__file__))[0]

from constants import (
    COLOR_DEEPBLACK, COLOR_BLACK, COLOR_RED, COLOR_YELLOW, COLOR_GREEN,
    COLOR_GRAY, COLOR_PURPLE,
    COLOR_TEAM_BLUE, COLOR_TEAM_YELLOW,
    COLOR_WP, COLOR_WP_PENDING, COLOR_WP_HALO,
    RIGHT_MARGIN_RATIO, SIDEBAR_BG, SIDEBAR_FG, SIDEBAR_DIM, SIDEBAR_ACCENT,
)

from world import (
    Obstacle, CircleObstacle, RobotObstacle, Robot, World,
    get_closest_collision_point,
)
from io_utils import copy_to_clipboard, state_load, state_save
from key_bindings import KeyBindings, _key_name
from mqtt_sim import (
    SimProcess, SimPart, MqttSimMessengerNodeWithClbk,
    PokuicomSim, PoklegscomSim, LidarSim, PokirobotSim, pokirobot_builder,
)
from game_viz import GameZoneVisualizer, load_image
from path_simulator import PathSimulator
from path_planner import PathPlanner


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

    def __init__(self, planner: "PathPlanner"):
        self.planner = planner
        self.row_h = 26
        self.GRID_OPTIONS = [25.0, 50.0, 100.0]
        try:
            init_idx = self.GRID_OPTIONS.index(planner.grid_size)
        except ValueError:
            init_idx = 1
        self.grid_slider = Slider(0, len(self.GRID_OPTIONS) - 1, init_idx, step=1,
                                  fmt=lambda v: f"{int(self.GRID_OPTIONS[int(v)])} mm")
        # angle slider exposes the denominator x in step = pi / x
        init_x = max(1, round(math.pi / planner.angle_step)) if planner.angle_step > 0 else 4
        self.angle_slider = Slider(1, 24, init_x, step=1, fmt=lambda v: f"π/{int(v)}")
        self._toggle_rects = {}  # name -> rect

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

    def __init__(self, planner: "PathPlanner", simulator: "PathSimulator", on_preview):
        self.planner = planner
        self.simulator = simulator
        self.on_preview = on_preview
        self.row_h = 20
        self.btn_h = 28
        self.status = ""
        self._buttons = []  # list of (rect, label, on_click) — populated on draw

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


def compute_layout(screen_size, bg_ratio):
    """Return (viewport_rect, sidebar_rect, orientation).

    In portrait mode the field can only occupy `w × (w / bg_ratio)`, so the
    leftover vertical space is folded into the sidebar (instead of wasted as
    black margin above/below the field). Sidebar gets at least RIGHT_MARGIN_RATIO
    of the screen so it doesn't collapse on near-square windows.
    """
    w, h = screen_size
    if h > w:
        min_sb_h = int(h * RIGHT_MARGIN_RATIO)
        field_h = int(w / bg_ratio)
        vp_h = min(field_h, h - min_sb_h)
        sb_h = h - vp_h
        return (0, 0, w, vp_h), (0, vp_h, w, sb_h), "bottom"
    min_sb_w = int(w * RIGHT_MARGIN_RATIO)
    field_w = int(h * bg_ratio)
    vp_w = min(field_w, w - min_sb_w)
    sb_w = w - vp_w
    return (0, 0, vp_w, h), (vp_w, 0, sb_w, h), "right"


class Sidebar:
    def __init__(self, panels, padding=12, gap=10):
        self.panels = panels
        self.padding = padding
        self.gap = gap
        self._font = None
        self._rect = (0, 0, 0, 0)
        self._orientation = "right"
        self._panel_rects = []  # list of (panel, rect)

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


class SidePanel(Panel):
    title = "Side"

    def __init__(self, planner: "PathPlanner", on_toggle):
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


class PreviewScreen:
    def __init__(self, code_text: str, on_back):
        self.code = code_text
        self.on_back = on_back
        self.status = ""
        self.scroll = 0
        self._buttons = []  # (rect, callback)

    def _do_copy(self):
        backend = copy_to_clipboard(self.code)
        self.status = f"copied ({backend})" if backend else "clipboard failed"

    def draw(self, screen):
        sw, sh = screen.get_size()
        screen.fill(SIDEBAR_BG)
        title_font = pg.font.SysFont("monospace", 22, bold=True)
        ui_font = pg.font.SysFont("monospace", 14, bold=True)
        code_font = pg.font.SysFont("monospace", 13)

        margin = 16
        btn_h = 32

        self._buttons = []

        # Back button
        back_rect = pg.Rect(margin, margin, 110, btn_h)
        pg.draw.rect(screen, COLOR_GRAY, back_rect, border_radius=6)
        pg.draw.rect(screen, SIDEBAR_FG, back_rect, width=1, border_radius=6)
        t = ui_font.render("← Back", True, SIDEBAR_FG)
        screen.blit(t, (back_rect.centerx - t.get_width() // 2, back_rect.centery - t.get_height() // 2))
        self._buttons.append((back_rect, self.on_back))

        # Title
        title = title_font.render("Generated code", True, SIDEBAR_ACCENT)
        screen.blit(title, (back_rect.right + margin, margin + 3))

        # Copy button
        copy_rect = pg.Rect(sw - margin - 110, margin, 110, btn_h)
        pg.draw.rect(screen, SIDEBAR_ACCENT, copy_rect, border_radius=6)
        pg.draw.rect(screen, COLOR_DEEPBLACK, copy_rect, width=1, border_radius=6)
        t = ui_font.render("Copy", True, COLOR_DEEPBLACK)
        screen.blit(t, (copy_rect.centerx - t.get_width() // 2, copy_rect.centery - t.get_height() // 2))
        self._buttons.append((copy_rect, self._do_copy))

        if self.status:
            s = ui_font.render(self.status, True, SIDEBAR_DIM)
            screen.blit(s, (sw - margin - s.get_width(), margin + btn_h + 6))

        # Code area
        code_top = margin + btn_h + 30
        code_area = pg.Rect(margin, code_top, sw - 2 * margin, sh - code_top - margin)
        pg.draw.rect(screen, (0x12, 0x12, 0x16), code_area, border_radius=6)
        pg.draw.rect(screen, SIDEBAR_DIM, code_area, width=1, border_radius=6)

        old_clip = screen.get_clip()
        screen.set_clip(code_area.inflate(-12, -12))
        line_h = code_font.get_linesize()
        y = code_area.y + 8 - self.scroll
        for line in self.code.splitlines() or [""]:
            if y + line_h >= code_area.y and y <= code_area.bottom:
                surf = code_font.render(line, True, SIDEBAR_FG)
                screen.blit(surf, (code_area.x + 10, y))
            y += line_h
        screen.set_clip(old_clip)

    def handle_event(self, event) -> bool:
        if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
            for rect, cb in self._buttons:
                if rect.collidepoint(event.pos):
                    cb()
                    return True
        if event.type == pg.MOUSEWHEEL:
            self.scroll = max(0, self.scroll - event.y * 30)
            return True
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_c:
                self._do_copy()
                self.on_back()
                return True
            if event.key in (pg.K_ESCAPE, pg.K_BACKSPACE):
                self.on_back()
                return True
        return False


class PokibotGameVisualizer:
    def __init__(self, world: World, on_pause_changed=None):
        self.world = world
        self.on_pause_changed = on_pause_changed or (lambda paused: None)

    def run(self):
        pg.init()
        clock = pg.time.Clock()
        screen = pg.display.set_mode((1280, 720), pg.RESIZABLE)
        game_viz = GameZoneVisualizer(self.world, screen)

        bindings = KeyBindings()
        planner = PathPlanner()

        def toggle_wps():
            game_viz.display_wps = not game_viz.display_wps

        def toggle_robot_obstacle():
            if "robot_obstacle" in self.world.obstacles:
                self.world.obstacles.pop("robot_obstacle")
            else:
                self.world.obstacles["robot_obstacle"] = RobotObstacle([0.0, 1000], 200)

        movement_speed = 25

        def move_obstacle(dx, dy):
            ro = self.world.obstacles.get("robot_obstacle")
            if ro is not None:
                ro.set_pos(ro.get_pos() + np.array([dx, dy]))

        def adj_grid(delta):
            options = snap_panel.GRID_OPTIONS
            try:
                idx = options.index(planner.grid_size)
            except ValueError:
                idx = 1
            idx = max(0, min(len(options) - 1, idx + delta))
            planner.grid_size = options[idx]
            snap_panel.grid_slider.value = idx

        def adj_angle(delta_x):
            cur_x = max(1, round(math.pi / planner.angle_step)) if planner.angle_step > 0 else 1
            new_x = max(int(snap_panel.angle_slider.min_v), min(int(snap_panel.angle_slider.max_v), cur_x + delta_x))
            planner.angle_step = math.pi / new_x
            snap_panel.angle_slider.value = new_x

        bindings.bind(pg.K_w, toggle_wps, doc="toggle waypoints")
        bindings.bind(pg.K_a, toggle_robot_obstacle, doc="toggle robot obstacle")
        bindings.bind(pg.K_z, planner.undo, doc="undo waypoint")
        bindings.bind(pg.K_x, planner.clear, doc="clear path")
        bindings.bind(pg.K_c, lambda: open_preview(planner.to_c_string()), doc="open code preview")
        bindings.bind(pg.K_SPACE, lambda: plan_panel._play_pause(), doc="sim play/pause")
        bindings.bind(pg.K_ESCAPE, lambda: plan_panel._reset_sim(), doc="sim reset")
        bindings.bind(pg.K_g, lambda: setattr(planner, "grid_enabled", not planner.grid_enabled), doc="toggle grid snap")
        bindings.bind(pg.K_t, lambda: setattr(planner, "angle_enabled", not planner.angle_enabled), doc="toggle angle snap")

        bindings.bind(pg.K_o, lambda: adj_grid(-1))
        bindings.bind(pg.K_p, lambda: adj_grid(+1))
        bindings.document("O/P", "grid size")

        bindings.bind(pg.K_k, lambda: adj_angle(-1))
        bindings.bind(pg.K_l, lambda: adj_angle(+1))
        bindings.document("K/L", "angle denom")

        bindings.bind(pg.K_LEFT, lambda: move_obstacle(-movement_speed, 0.0), repeat=True)
        bindings.bind(pg.K_RIGHT, lambda: move_obstacle(movement_speed, 0.0), repeat=True)
        bindings.bind(pg.K_UP, lambda: move_obstacle(0.0, movement_speed), repeat=True)
        bindings.bind(pg.K_DOWN, lambda: move_obstacle(0.0, -movement_speed), repeat=True)
        bindings.document("arrows", "move obstacle", repeat=True)

        def fmt_robots():
            return f"{len(self.world.robots)}"

        def fmt_first_robot_pos():
            if not self.world.robots:
                return "-"
            r = next(iter(self.world.robots.values()))
            return f"{r.pos[0]:.0f} {r.pos[1]:.0f} {r.pos[2]:.2f}"

        def fmt_first_robot_team():
            if not self.world.robots:
                return "-"
            r = next(iter(self.world.robots.values()))
            return "yellow" if r.team else "blue"

        def fmt_obstacle_pos():
            ro = self.world.obstacles.get("robot_obstacle")
            return f"{ro.pos[0]:.0f} {ro.pos[1]:.0f}" if ro is not None else "-"

        def fmt_wps_visible():
            return "on" if game_viz.display_wps else "off"

        inputs = InputsPanel([
            ("robots", fmt_robots),
            ("r0 pos", fmt_first_robot_pos),
            ("r0 team", fmt_first_robot_team),
            ("obs pos", fmt_obstacle_pos),
            ("wps", fmt_wps_visible),
        ])

        simulator = PathSimulator(
            self.world, planner,
            on_user_running=lambda running: self.on_pause_changed(not running),
        )

        preview_ref = [None]  # list-as-cell for mutation from closures

        def open_preview(code_text):
            preview_ref[0] = PreviewScreen(code_text, on_back=lambda: preview_ref.__setitem__(0, None))

        def toggle_side(new_side):
            if new_side == planner.side:
                return
            planner.mirror_y_axis()
            ro = self.world.obstacles.get("robot_obstacle")
            if ro is not None:
                p = ro.get_pos()
                ro.set_pos(np.array([-p[0], p[1]]))
            planner.side = new_side

        side_panel = SidePanel(planner, toggle_side)
        plan_panel = PlanPanel(planner, simulator, on_preview=open_preview)
        snap_panel = SnapPanel(planner)
        sidebar = Sidebar([KeybindingsPanel(bindings), inputs, side_panel, snap_panel, plan_panel])
        sidebar_capture = False
        sim_dt = 1.0 / 60.0

        bindings.bind(pg.K_s, lambda: toggle_side("yellow" if planner.side == "blue" else "blue"), doc="switch side")

        # --- state persistence ---
        state_path = os.path.join(main_dir, "..", "state.json")
        loaded = state_load(state_path)
        if loaded is not None:
            planner.side = loaded.get("side", "blue")
            planner.waypoints = [tuple(w) for w in loaded.get("waypoints", []) if len(w) == 3]
        last_state_fp = (planner.side, tuple(planner.waypoints))

        while True:
            screen.fill(pg.Color(33, 33, 33))

            if preview_ref[0] is not None:
                preview_ref[0].draw(screen)
                for e in pg.event.get():
                    if e.type == pg.QUIT:
                        return
                    if e.type == pg.VIDEORESIZE:
                        screen = pg.display.get_surface()
                        game_viz.screen = screen
                        continue
                    preview_ref[0].handle_event(e)
                    if preview_ref[0] is None:
                        break
                pg.display.update()
                clock.tick(60)
                continue

            viewport, sb_rect, orientation = compute_layout(screen.get_size(), game_viz.bg_ratio)
            sidebar.set_rect(sb_rect, orientation)

            simulator.step(sim_dt)
            game_viz.loop(viewport)
            planner.draw_grid(game_viz, screen)
            mouse_screen_pos = pg.mouse.get_pos()
            planner.draw(game_viz, screen, sim=simulator, mouse_screen_pos=mouse_screen_pos)

            for e in pg.event.get():
                if e.type == pg.QUIT:
                    return
                if e.type == pg.VIDEORESIZE:
                    screen = pg.display.get_surface()
                    game_viz.screen = screen
                    continue
                if e.type == pg.MOUSEWHEEL:
                    if not sidebar_capture and planner._mode is None:
                        planner.cycle_hover(-e.y)  # wheel up = previous overlap
                    continue
                if e.type in (pg.MOUSEBUTTONDOWN, pg.MOUSEBUTTONUP, pg.MOUSEMOTION):
                    if e.type == pg.MOUSEBUTTONDOWN and sidebar.contains(e.pos):
                        if sidebar.handle_event(e):
                            sidebar_capture = True
                            continue
                    if sidebar_capture:
                        sidebar.handle_event(e)
                        if e.type == pg.MOUSEBUTTONUP:
                            sidebar_capture = False
                        continue
                    if e.type == pg.MOUSEBUTTONDOWN and e.button == 1:
                        wp = game_viz.screen_to_world(e.pos)
                        if wp is not None:
                            shift = bool(pg.key.get_mods() & pg.KMOD_SHIFT)
                            planner.start_drag(wp, shift=shift)
                    elif e.type == pg.MOUSEMOTION:
                        wp = game_viz.screen_to_world(e.pos)
                        if planner._mode is None:
                            planner.set_hover(wp)
                        else:
                            planner.update_drag(wp)
                    elif e.type == pg.MOUSEBUTTONUP and e.button == 1:
                        wp = game_viz.screen_to_world(e.pos)
                        planner.end_drag(wp)
                        planner.set_hover(wp)
                    elif e.type == pg.MOUSEBUTTONDOWN and e.button == 3:
                        wp = game_viz.screen_to_world(e.pos)
                        if not planner.delete_at(wp):
                            planner.undo()
                    continue
                bindings.dispatch(e)

            bindings.poll(pg.key.get_pressed())

            sidebar.draw(screen)

            # --- auto-save state on any change ---
            fp = (planner.side, tuple(planner.waypoints))
            if fp != last_state_fp:
                state_save(state_path, planner.side, planner.waypoints)
                last_state_fp = fp

            pg.display.update()
            clock.tick(60)
        pg.quit()

class PokibotGameSimulator:
    def __init__(self):
        super().__init__()

        self.msms = MqttSimMessengerServer(self.on_device_connection, self.on_device_disconnection)
        self.world = World()
        # self.world = World(obstacles={"opponent_robot": RobotObstacle([0.0, 1.5], 0.2)})
        self.pokirobot_sim_nodes : dict[str, PokirobotSim]  = {}
        self.visualizer = PokibotGameVisualizer(self.world, on_pause_changed=self._set_mqtt_paused)
        self.last_robot_team = 0

    def _set_mqtt_paused(self, paused: bool):
        for sim in self.pokirobot_sim_nodes.values():
            for part in sim.sim_process:
                if isinstance(part, PoklegscomSim):
                    part.motor_break = paused

    def run(self):
        self.msms.start()
        self.visualizer.run()
        for id, robot in self.pokirobot_sim_nodes.items():
            robot.stop()
        self.msms.stop()

    def on_device_connection(self, dev_name, dev_id):
        if dev_id not in self.pokirobot_sim_nodes.keys():
            robot, pokirobot = pokirobot_builder(dev_id, self.msms, self.world, self.last_robot_team)
            self.last_robot_team = (self.last_robot_team + 1) % 2
            self.world.robots["pokirobot_" + dev_id] = robot
            self.pokirobot_sim_nodes[dev_id] = pokirobot

    def on_device_disconnection(self, dev_name, dev_id):
        if dev_id in self.pokirobot_sim_nodes.keys():
            self.pokirobot_sim_nodes[dev_id].stop()
            self.pokirobot_sim_nodes.pop(dev_id)
            self.world.robots.pop("pokirobot_" + dev_id)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    r = PokibotGameSimulator()
    r.run()
