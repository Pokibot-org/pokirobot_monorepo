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
from layout import compute_layout
from panels import (
    Panel, Slider, Sidebar,
    SnapPanel, PlanPanel, KeybindingsPanel, InputsPanel, SidePanel,
)


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
                    if not sidebar_capture and not planner.is_dragging():
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
                        if not planner.is_dragging():
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
