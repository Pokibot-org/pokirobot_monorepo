"""Application orchestration: PokibotGameVisualizer owns the pygame window
and event loop; PokibotGameSimulator wires it to the MQTT messenger so the
firmware can drive simulated robots over the same topics it uses in prod."""
import math
import os

import numpy as np
import pygame as pg
from msm import MqttSimMessengerServer

from game_viz import GameZoneVisualizer
from io_utils import state_load, state_save
from key_bindings import KeyBindings
from layout import compute_layout
from mqtt_sim import PoklegscomSim, PokirobotSim, pokirobot_builder
from panels import (
    InputsPanel, KeybindingsPanel, PlanPanel, Sidebar, SidePanel, SnapPanel,
)
from actions import normalize_actions_list
from path_planner import PathPlanner
from path_simulator import PathSimulator
from preview import PreviewScreen
from waypoint_modal import WaypointModal
from world import RobotObstacle, World

_main_dir = os.path.split(os.path.abspath(__file__))[0]


class PokibotGameVisualizer:
    """Pygame window + main loop. Owns the planner, simulator, sidebar
    panels, and key bindings. PokibotGameSimulator hooks `on_pause_changed`
    to keep the mqtt-driven robots in sync with the local play/pause state."""

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
        self._wp_modal = None
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

    def _insert_relative(self, idx, before: bool):
        if 0 <= idx < len(self.planner.waypoints):
            x, y, a, _ = self.planner.waypoints[idx]
            insert_idx = idx if before else idx + 1
            self.planner.insert_waypoint(insert_idx, x, y, a, [])
            self.planner.start_placing(insert_idx)

    def _open_wp_modal(self, wp_index, anchor_screen_pos):
        self._wp_modal = WaypointModal(
            self.planner,
            wp_index,
            anchor_screen_pos,
            on_close=lambda: setattr(self, "_wp_modal", None),
            on_insert_before=lambda idx: self._insert_relative(idx, before=True),
            on_insert_after=lambda idx: self._insert_relative(idx, before=False),
        )

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

        def _delete_hovered():
            idx = p._hovered_index()
            if idx is not None:
                p.delete_index(idx)
        b.bind(pg.K_DELETE, _delete_hovered, doc="delete hovered waypoint")
        b.bind(pg.K_BACKSPACE, _delete_hovered)

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
        self._state_path = os.path.join(_main_dir, "..", "state.json")
        loaded = state_load(self._state_path)
        if loaded is not None:
            self.planner.side = loaded.get("side", "blue")
            wps = []
            for w in loaded.get("waypoints", []):
                if len(w) == 3:
                    wps.append((w[0], w[1], w[2], []))
                elif len(w) == 4:
                    wps.append((w[0], w[1], w[2], normalize_actions_list(w[3])))
            self.planner.waypoints = wps
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
        if self._wp_modal is not None:
            self._wp_modal.handle_event(e)
            return "consumed"
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
        if self.planner.is_placing():
            wp = self.game_viz.screen_to_world(e.pos)
            if e.type == pg.MOUSEMOTION:
                self.planner.update_placing(wp)
            elif e.type == pg.MOUSEBUTTONDOWN and e.button == 1:
                self.planner.update_placing(wp)
                self.planner.commit_placing()
            elif e.type == pg.MOUSEBUTTONDOWN and e.button == 3:
                self.planner.cancel_placing()
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
            hit = self.planner.find_index_near(wp) if wp is not None else None
            if hit is not None:
                self._open_wp_modal(hit, e.pos)
            else:
                self.planner.undo()

    # ---- drawing -----------------------------------------------------------

    def _draw_action_toast(self):
        result = self.simulator.get_active_action_progress()
        if result is None:
            return
        label, frac = result
        rp = self.simulator.robot.pos
        rx, ry = self.game_viz.get_on_board_pos((rp[0], rp[1]))
        try:
            font = pg.font.SysFont("monospace", 14)
        except Exception:
            font = pg.font.Font(None, 16)
        text_surf = font.render(label, True, (255, 255, 255))
        pad_x, pad_y = 10, 4
        toast_w = text_surf.get_width() + pad_x * 2
        toast_h = 24
        bar_h = 3
        total_h = toast_h + bar_h
        sw, sh = self.screen.get_size()
        tx = rx - toast_w // 2
        ty = ry - 40 - total_h
        tx = max(4, min(tx, sw - toast_w - 4))
        ty = max(4, min(ty, sh - total_h - 4))
        bg = pg.Surface((toast_w, total_h), pg.SRCALPHA)
        pg.draw.rect(bg, (10, 10, 14, 220), pg.Rect(0, 0, toast_w, toast_h), border_radius=6)
        self.screen.blit(bg, (tx, ty))
        self.screen.blit(text_surf, (tx + pad_x, ty + (toast_h - text_surf.get_height()) // 2))
        bar_fill = int(toast_w * frac)
        if bar_fill > 0:
            pg.draw.rect(self.screen, (0xCF, 0xD1, 0x86),
                         pg.Rect(tx, ty + toast_h, bar_fill, bar_h), border_radius=2)

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
        self._draw_action_toast()

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
                if self._wp_modal is not None:
                    self._wp_modal.draw(self.screen)
                self._autosave_state()
            pg.display.update()
            self.clock.tick(60)


class PokibotGameSimulator:
    """Top-level application: spins up the MQTT messenger, builds a
    PokirobotSim per connected device, and hands the visualizer a hook so
    play/pause toggles motor_break across every PoklegscomSim."""

    def __init__(self):
        self.msms = MqttSimMessengerServer(self.on_device_connection, self.on_device_disconnection)
        self.world = World()
        self.pokirobot_sim_nodes: dict[str, PokirobotSim] = {}
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
        for robot in self.pokirobot_sim_nodes.values():
            robot.stop()
        self.msms.stop()

    def on_device_connection(self, dev_name, dev_id):
        if dev_id in self.pokirobot_sim_nodes:
            return
        robot, pokirobot = pokirobot_builder(dev_id, self.msms, self.world, self.last_robot_team)
        self.last_robot_team = (self.last_robot_team + 1) % 2
        self.world.robots["pokirobot_" + dev_id] = robot
        self.pokirobot_sim_nodes[dev_id] = pokirobot

    def on_device_disconnection(self, dev_name, dev_id):
        if dev_id not in self.pokirobot_sim_nodes:
            return
        self.pokirobot_sim_nodes[dev_id].stop()
        self.pokirobot_sim_nodes.pop(dev_id)
        self.world.robots.pop("pokirobot_" + dev_id)
