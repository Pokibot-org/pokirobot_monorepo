#!/usr/bin/env python
import logging
import math
import os

import numpy as np
import pygame as pg
from msm import MqttSimMessengerServer

from io_utils import state_load, state_save
from game_viz import GameZoneVisualizer
from key_bindings import KeyBindings
from layout import compute_layout
from mqtt_sim import PoklegscomSim, PokirobotSim, pokirobot_builder
from panels import (
    InputsPanel, KeybindingsPanel, PlanPanel, SidePanel, SnapPanel, Sidebar,
)
from path_planner import PathPlanner
from path_simulator import PathSimulator
from preview import PreviewScreen
from world import RobotObstacle, World

logger = logging.getLogger("strat_visu")

main_dir = os.path.split(os.path.abspath(__file__))[0]


class PokibotGameVisualizer:
    SIM_DT = 1.0 / 60.0

    def __init__(self, world: World, on_pause_changed=None):
        self.world = world
        self.on_pause_changed = on_pause_changed or (lambda paused: None)
        # populated by run() / _build_ui()
        self.screen = None
        self.clock = None
        self.game_viz = None
        self.bindings = None
        self.planner = None
        self.simulator = None
        self.sidebar = None
        self.kb_panel = None
        self.inputs_panel = None
        self.side_panel = None
        self.snap_panel = None
        self.plan_panel = None
        self._preview = None
        self._sidebar_capture = False
        self._state_path = None
        self._last_state_fp = None

    # ---- action handlers (called by bindings + panel callbacks) -----------

    def _toggle_wps(self):
        self.game_viz.display_wps = not self.game_viz.display_wps

    def _toggle_robot_obstacle(self):
        if "robot_obstacle" in self.world.obstacles:
            self.world.obstacles.pop("robot_obstacle")
        else:
            self.world.obstacles["robot_obstacle"] = RobotObstacle([0.0, 1000], 200)

    def _move_obstacle(self, dx, dy):
        ro = self.world.obstacles.get("robot_obstacle")
        if ro is not None:
            ro.set_pos(ro.get_pos() + np.array([dx, dy]))

    def _adj_grid(self, delta):
        options = self.snap_panel.GRID_OPTIONS
        try:
            idx = options.index(self.planner.grid_size)
        except ValueError:
            idx = 1
        idx = max(0, min(len(options) - 1, idx + delta))
        self.planner.grid_size = options[idx]
        self.snap_panel.grid_slider.value = idx

    def _adj_angle(self, delta_x):
        p = self.planner
        cur_x = max(1, round(math.pi / p.angle_step)) if p.angle_step > 0 else 1
        slider = self.snap_panel.angle_slider
        new_x = max(int(slider.min_v), min(int(slider.max_v), cur_x + delta_x))
        p.angle_step = math.pi / new_x
        slider.value = new_x

    def _open_preview(self, code_text):
        self._preview = PreviewScreen(code_text, on_back=self._close_preview)

    def _close_preview(self):
        self._preview = None

    def _toggle_side(self, new_side):
        if new_side == self.planner.side:
            return
        self.planner.mirror_y_axis()
        ro = self.world.obstacles.get("robot_obstacle")
        if ro is not None:
            p = ro.get_pos()
            ro.set_pos(np.array([-p[0], p[1]]))
        self.planner.side = new_side

    # ---- construction ------------------------------------------------------

    def _build_bindings(self):
        b = self.bindings
        p = self.planner
        movement_speed = 25

        b.bind(pg.K_w, self._toggle_wps, doc="toggle waypoints")
        b.bind(pg.K_a, self._toggle_robot_obstacle, doc="toggle robot obstacle")
        b.bind(pg.K_z, p.undo, doc="undo waypoint")
        b.bind(pg.K_x, p.clear, doc="clear path")
        b.bind(pg.K_c, lambda: self._open_preview(p.to_c_string()), doc="open code preview")
        b.bind(pg.K_SPACE, lambda: self.plan_panel._play_pause(), doc="sim play/pause")
        b.bind(pg.K_ESCAPE, lambda: self.plan_panel._reset_sim(), doc="sim reset")
        b.bind(pg.K_g, lambda: setattr(p, "grid_enabled", not p.grid_enabled), doc="toggle grid snap")
        b.bind(pg.K_t, lambda: setattr(p, "angle_enabled", not p.angle_enabled), doc="toggle angle snap")
        b.bind(pg.K_s, lambda: self._toggle_side("yellow" if p.side == "blue" else "blue"), doc="switch side")

        b.bind(pg.K_o, lambda: self._adj_grid(-1))
        b.bind(pg.K_p, lambda: self._adj_grid(+1))
        b.document("O/P", "grid size")

        b.bind(pg.K_k, lambda: self._adj_angle(-1))
        b.bind(pg.K_l, lambda: self._adj_angle(+1))
        b.document("K/L", "angle denom")

        b.bind(pg.K_LEFT, lambda: self._move_obstacle(-movement_speed, 0.0), repeat=True)
        b.bind(pg.K_RIGHT, lambda: self._move_obstacle(movement_speed, 0.0), repeat=True)
        b.bind(pg.K_UP, lambda: self._move_obstacle(0.0, movement_speed), repeat=True)
        b.bind(pg.K_DOWN, lambda: self._move_obstacle(0.0, -movement_speed), repeat=True)
        b.document("arrows", "move obstacle", repeat=True)

    def _build_inputs_panel(self) -> InputsPanel:
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
            return "on" if self.game_viz.display_wps else "off"

        return InputsPanel([
            ("robots", fmt_robots),
            ("r0 pos", fmt_first_robot_pos),
            ("r0 team", fmt_first_robot_team),
            ("obs pos", fmt_obstacle_pos),
            ("wps", fmt_wps_visible),
        ])

    def _build_ui(self):
        self.bindings = KeyBindings()
        self.planner = PathPlanner()
        self.simulator = PathSimulator(
            self.world, self.planner,
            on_user_running=lambda running: self.on_pause_changed(not running),
        )
        self.kb_panel = KeybindingsPanel(self.bindings)
        self.inputs_panel = self._build_inputs_panel()
        self.side_panel = SidePanel(self.planner, self._toggle_side)
        self.snap_panel = SnapPanel(self.planner)
        self.plan_panel = PlanPanel(self.planner, self.simulator, on_preview=self._open_preview)
        self.sidebar = Sidebar([
            self.kb_panel, self.inputs_panel, self.side_panel,
            self.snap_panel, self.plan_panel,
        ])
        self._build_bindings()
        self._load_state()

    def _load_state(self):
        self._state_path = os.path.join(main_dir, "..", "state.json")
        loaded = state_load(self._state_path)
        if loaded is not None:
            self.planner.side = loaded.get("side", "blue")
            self.planner.waypoints = [tuple(w) for w in loaded.get("waypoints", []) if len(w) == 3]
        self._last_state_fp = (self.planner.side, tuple(self.planner.waypoints))

    def _autosave_state(self):
        fp = (self.planner.side, tuple(self.planner.waypoints))
        if fp != self._last_state_fp:
            state_save(self._state_path, self.planner.side, self.planner.waypoints)
            self._last_state_fp = fp

    # ---- event handling ----------------------------------------------------

    def _handle_window_event(self, e):
        """Return 'quit' / 'consumed' / None. Shared by both event loops."""
        if e.type == pg.QUIT:
            return "quit"
        if e.type == pg.VIDEORESIZE:
            self.screen = pg.display.get_surface()
            self.game_viz.screen = self.screen
            return "consumed"
        return None

    def _handle_preview_event(self, e):
        r = self._handle_window_event(e)
        if r is not None:
            return r
        if self._preview is not None:
            self._preview.handle_event(e)
        return None

    def _handle_event(self, e):
        r = self._handle_window_event(e)
        if r is not None:
            return r
        if e.type == pg.MOUSEWHEEL:
            if not self._sidebar_capture and not self.planner.is_dragging():
                self.planner.cycle_hover(-e.y)
            return "consumed"
        if e.type in (pg.MOUSEBUTTONDOWN, pg.MOUSEBUTTONUP, pg.MOUSEMOTION):
            self._handle_mouse_event(e)
            return "consumed"
        self.bindings.dispatch(e)
        return None

    def _handle_mouse_event(self, e):
        if e.type == pg.MOUSEBUTTONDOWN and self.sidebar.contains(e.pos):
            if self.sidebar.handle_event(e):
                self._sidebar_capture = True
                return
        if self._sidebar_capture:
            self.sidebar.handle_event(e)
            if e.type == pg.MOUSEBUTTONUP:
                self._sidebar_capture = False
            return
        if e.type == pg.MOUSEBUTTONDOWN and e.button == 1:
            wp = self.game_viz.screen_to_world(e.pos)
            if wp is not None:
                shift = bool(pg.key.get_mods() & pg.KMOD_SHIFT)
                self.planner.start_drag(wp, shift=shift)
        elif e.type == pg.MOUSEMOTION:
            wp = self.game_viz.screen_to_world(e.pos)
            if not self.planner.is_dragging():
                self.planner.set_hover(wp)
            else:
                self.planner.update_drag(wp)
        elif e.type == pg.MOUSEBUTTONUP and e.button == 1:
            wp = self.game_viz.screen_to_world(e.pos)
            self.planner.end_drag(wp)
            self.planner.set_hover(wp)
        elif e.type == pg.MOUSEBUTTONDOWN and e.button == 3:
            wp = self.game_viz.screen_to_world(e.pos)
            if not self.planner.delete_at(wp):
                self.planner.undo()

    # ---- drawing -----------------------------------------------------------

    def _draw_frame(self):
        self.screen.fill(pg.Color(33, 33, 33))
        if self._preview is not None:
            self._preview.draw(self.screen)
            return
        viewport, sb_rect, orientation = compute_layout(
            self.screen.get_size(), self.game_viz.bg_ratio,
        )
        self.sidebar.set_rect(sb_rect, orientation)
        self.simulator.step(self.SIM_DT)
        self.game_viz.loop(viewport)
        self.planner.draw_grid(self.game_viz, self.screen)
        self.planner.draw(
            self.game_viz, self.screen,
            sim=self.simulator, mouse_screen_pos=pg.mouse.get_pos(),
        )

    # ---- entry point -------------------------------------------------------

    def run(self):
        pg.init()
        self.clock = pg.time.Clock()
        self.screen = pg.display.set_mode((1280, 720), pg.RESIZABLE)
        self.game_viz = GameZoneVisualizer(self.world, self.screen)
        self._build_ui()
        self._loop()
        pg.quit()

    def _loop(self):
        while True:
            self._draw_frame()
            if self._preview is not None:
                for e in pg.event.get():
                    if self._handle_preview_event(e) == "quit":
                        return
            else:
                for e in pg.event.get():
                    if self._handle_event(e) == "quit":
                        return
                self.bindings.poll(pg.key.get_pressed())
                self.sidebar.draw(self.screen)
                self._autosave_state()
            pg.display.update()
            self.clock.tick(60)

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
