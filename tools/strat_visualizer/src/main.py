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
