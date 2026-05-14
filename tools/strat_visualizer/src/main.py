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

class SimProcess:
    def __init__(self, loop_clbk, frequency) -> None:
        self.sim_running = False
        self.loop_clbk = loop_clbk
        self.cycle_time = 1/frequency

    def start(self):
        def sim_task():
            t0 = time.perf_counter()
            time_counter = 0
            while self.sim_running:
                self.loop_clbk(self.cycle_time)
                now = time.perf_counter()
                elapsed_time = now - t0
                target_time = time_counter + self.cycle_time
                if elapsed_time < target_time:
                    time.sleep(target_time - elapsed_time)
                time_counter += self.cycle_time

        self.sim_running = True
        t = threading.Thread(target=sim_task, daemon=True)
        t.start()

    def stop(self):
        self.sim_running = False

class MqttSimMessengerNodeWithClbk(MqttSimMessengerNode):
    def __init__(self, parent, id, name, clbk) -> None:
        super().__init__(parent, id, name)
        self.clbk = clbk

    def process_topic(self, topic, payload):
        self.clbk(self, topic, payload)

class SimPart:
    def __init__(self, sim_process = []) -> None:
        self.sim_process = sim_process

    def start(self):
        for process in self.sim_process:
            process.start()

    def stop(self):
        for process in self.sim_process:
            process.stop()

class PokuicomSim(SimPart):
    def __init__(self, msm: MqttSimMessengerNode, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.pokuicom = MqttSimMessengerNodeWithClbk(msm, 0, "pokuicom", self.process_pokuicom)
        self.pokuicom.subscribe("request")
        self.pokuicom.subscribe("score")

    def process_pokuicom(self, parent: MqttSimMessengerNode, topic, payload):
        # types: SOCRE TEAM MATCHSTERTED
        payload = json.loads(payload)
        match topic:
            case "request":
                match payload["value"]:
                    case 0:
                        pass
                    case 1:
                        parent.send("team", f"{self.robot.team}")
                    case 2:
                        parent.send("match", f"{self.robot.tirette}")
                        if self.robot.tirette != 3:
                            self.robot.tirette = 3
            case "score":
                logger.info(f'New score: {payload["value"]}')

class PoklegscomSim(SimPart):
    def __init__(self, msm: MqttSimMessengerNode, robot: Robot) -> None:
        super().__init__([SimProcess(self.movement_sim, 60), SimProcess(self.pos_publish, 30)])
        self.robot = robot
        self.wp_index = 0
        self.wps = []
        self.motor_break = False
        self.speed = 500
        self.angle_speed = 3.14/2
        self.sensivity = 0.01

        self.poklegscom = MqttSimMessengerNodeWithClbk(msm, 0, "poklegscom", self.process_poklegscom)
        self.poklegscom.subscribe("set_pos")
        self.poklegscom.subscribe("set_waypoints")
        self.poklegscom.subscribe("set_break")

    def process_poklegscom(self, parent: MqttSimMessengerNode, topic, raw_payload):
        # types: SCORE TEAM MATCH_STARTED
        try:
            payload = json.loads(raw_payload)
        except Exception as e:
            logger.error(e)
            logger.error(raw_payload)
            return

        match topic:
            case "set_pos":
                self.robot.pos = np.array([payload["x"], payload["y"], payload["a"]])
            case "set_waypoints":
                self.wp_index = 0
                self.wps = [np.array([x["x"], x["y"], x["a"]]) for x in payload]
                self.robot.wps = self.wps
                logger.info(f"set wps {payload}")
            case "set_break":
                self.motor_break = payload["state"]

    def movement_sim(self, dt):
        sensivity = np.array([
            self.sensivity,
            self.sensivity,
            self.sensivity
        ])

        nb_wps = len(self.wps)

        if self.wp_index < nb_wps and not self.motor_break:
            target_pos = self.wps[self.wp_index]

            delta = target_pos - self.robot.pos

            # Separate XY and angle
            delta_xy = delta[:2]
            delta_angle = delta[2]

            # Smooth proportional XY cap
            dist_xy = np.linalg.norm(delta_xy)

            if dist_xy > 1e-6:
                max_step_xy = self.speed * dt

                # move proportionally toward target
                step_xy = delta_xy / dist_xy * min(dist_xy, max_step_xy)
            else:
                step_xy = np.zeros(2)

            # Independent angular clamp
            step_angle = np.clip(
                delta_angle,
                -self.angle_speed * dt,
                self.angle_speed * dt
            )

            self.robot.pos += np.array([
                step_xy[0],
                step_xy[1],
                step_angle
            ])

            if np.all(np.abs(target_pos - self.robot.pos) < sensivity):
                self.wp_index += 1
                self.robot.wps = self.wps[min(self.wp_index, nb_wps - 1):]

    def pos_publish(self, dt):
        self.poklegscom.send("pos", f"{self.robot.pos[0]} {self.robot.pos[1]} {self.robot.pos[2]}")
        wps = self.wps
        wps_len = len(wps)
        wp_index = self.wp_index
        if wp_index < wps_len:
            robot_pos = np.array([self.robot.pos[0], self.robot.pos[1]])
            nb = 3
            last_target_index = min(wp_index + nb, wps_len - 1)
            targets_pos = [np.array([wps[i][0], wps[i][1]]) for i in range(wp_index, last_target_index + 1)]
            diffs = [target_pos - robot_pos for target_pos in targets_pos]
            angles = [ np.atan2(diff[1], diff[0]) for diff in diffs]
            self.robot.dir = float(np.average(angles))
            self.poklegscom.send("dir", f"{self.robot.dir}")


class LidarSim(SimPart):
    def __init__(self, msm: MqttSimMessengerNode, robot: Robot, world: World, id=0) -> None:
        super().__init__([SimProcess(self.lidar_scan, 10)])
        self.robot = robot
        self.world = world
        self.publish = False
        self.resolution = 360

        self.msm = MqttSimMessengerNodeWithClbk(msm, id, "lidar", self.process_com)


    def process_com(self, parent: MqttSimMessengerNode, topic, raw_payload):
        try:
            payload = json.loads(raw_payload)
        except Exception as e:
            logger.error(e)
            logger.error(raw_payload)
            return

        match topic:
            case "start_publish":
                self.publish = True


    def lidar_scan(self, dt):
        quant = 3
        debug_point_list = []
        obstacle_shape = unary_union([obstacle.get_shape() for _, obstacle in self.world.obstacles.items()])
        to_send_point_list = []
        for i in range(self.resolution):
            lidar_angle = i/self.resolution * 2 * np.pi
            table_angle = lidar_angle + self.robot.pos[2]
            dir = [np.cos(table_angle), np.sin(table_angle)]
            robot_pos = Point(self.robot.pos[0:2])
            shapely_point = get_closest_collision_point(obstacle_shape, robot_pos , Point(dir))
            if shapely_point:
                debug_point_list.append([shapely_point.x, shapely_point.y])
                to_send_point_list.append({"angle": round(lidar_angle, quant), "distance": round(robot_pos.distance(shapely_point), quant), "intensity": 255})
        self.robot.lidar_points = debug_point_list
        self.msm.send("lidar_points", json.dumps(to_send_point_list))


class PokirobotSim(SimPart):
    def __init__(self, sim_process) -> None:
        super().__init__(sim_process)
        self.start()

def pokirobot_builder(id: str, msms: MqttSimMessengerServer, world: World, team=0) -> tuple[Robot, PokirobotSim]:
    robot = Robot(team=team)
    pokirobot_msm = MqttSimMessengerNode(msms, id, 'pokirobot')
    return robot, PokirobotSim([PokuicomSim(pokirobot_msm, robot), PoklegscomSim(pokirobot_msm, robot), LidarSim(pokirobot_msm, robot, world, 0)])

# quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, "../img", name)
    return pg.image.load(path).convert()

class GameZoneVisualizer:
    def __init__(self, world: World, screen):
        self.world = world
        self.screen = screen
        self.right_margin_ratio = RIGHT_MARGIN_RATIO

        self.real_size = [3000, 2000]
        self.mins = [-1500, 0]
        self.maxs = [1500, 2000]
        self.dim_x = [0, 1]
        self.dim_y = [0, 1]
        self.rl_to_px_ratio = 1000
        self.display_wps = True
        self.bg = load_image("vinyle.png")
        self.bg_ratio = self.bg.get_width() / self.bg.get_height()
        # Robot art — index by team (0=blue, 1=yellow). Source assets face +Y
        # (up) in the image, so we apply a -90° offset when rotating.
        self._robot_imgs = {
            0: pg.image.load(os.path.join(main_dir, "../img", "robot_blue.png")).convert_alpha(),
            1: pg.image.load(os.path.join(main_dir, "../img", "robot_yellow.png")).convert_alpha(),
        }
        self._robot_scaled = {}  # (team, diameter_px) -> scaled Surface

    def get_on_board_pos(self, pos):
        # dim_x = [left, right], dim_y = [top, bottom] of the board in screen coords.
        return (
            self.dim_x[0] + (pos[0] - self.mins[0]) * self.rl_to_px_ratio,
            self.dim_y[1] - (pos[1] - self.mins[1]) * self.rl_to_px_ratio,
        )

    def get_on_board_dim(self, dim):
        return dim * self.rl_to_px_ratio

    ROBOT_IMG_MM_PER_PX = 1.0  # source assets: 1 pixel = 1 mm

    def _scaled_robot_img(self, team: int):
        # cache key quantises the scale so subpixel ratio changes don't churn
        key = (team, round(self.rl_to_px_ratio, 4))
        cached = self._robot_scaled.get(key)
        if cached is not None:
            return cached
        src = self._robot_imgs[team]
        sw, sh = src.get_width(), src.get_height()
        screen_per_src_px = self.ROBOT_IMG_MM_PER_PX * self.rl_to_px_ratio
        target = (max(1, int(round(sw * screen_per_src_px))),
                  max(1, int(round(sh * screen_per_src_px))))
        scaled = pg.transform.smoothscale(src, target)
        if len(self._robot_scaled) > 8:
            self._robot_scaled.clear()
        self._robot_scaled[key] = scaled
        return scaled

    def draw_robot(self, robot: Robot):
        on_board_pos = self.get_on_board_pos(robot.pos)
        on_board_radius = self.get_on_board_dim(robot.radius)
        team = 1 if robot.team else 0
        scaled = self._scaled_robot_img(team)
        rotated = pg.transform.rotate(scaled, math.degrees(robot.dir) - 90)
        rect = rotated.get_rect(center=on_board_pos)
        self.screen.blit(rotated, rect)
        # Debug overlay: commanded heading (red) vs actual pos[2] angle (gray)
        pg.draw.line(
            self.screen,
            COLOR_RED,
            on_board_pos,
            (
                on_board_pos[0] + on_board_radius * math.cos(robot.dir),
                on_board_pos[1] + on_board_radius * math.sin(robot.dir + math.pi),
            ),
            width=4,
        )
        pg.draw.line(
            self.screen,
            COLOR_GRAY,
            on_board_pos,
            (
                on_board_pos[0] + on_board_radius * math.cos(robot.pos[2]),
                on_board_pos[1] + on_board_radius * math.sin(robot.pos[2] + math.pi),
            ),
            width=4,
        )

    def draw_obstacle(self, obstacle: Obstacle):
        if type(obstacle) is CircleObstacle:
            on_board_pos = self.get_on_board_pos(obstacle.pos)
            on_board_radius = self.get_on_board_dim(obstacle.radius)
            pg.draw.circle(self.screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
            pg.draw.circle(self.screen, COLOR_GREEN, on_board_pos, on_board_radius*0.98)
        elif type(obstacle) is RobotObstacle:
            on_board_pos = self.get_on_board_pos(obstacle.pos)
            on_board_radius = self.get_on_board_dim(obstacle.radius)
            on_board_scan_radius = self.get_on_board_dim(obstacle.scan_radius)
            pg.draw.circle(self.screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
            pg.draw.circle(self.screen, COLOR_GREEN, on_board_pos, on_board_radius*0.98)
            pg.draw.circle(self.screen, COLOR_BLACK, on_board_pos, on_board_scan_radius)

    def draw_debug_point(self, pos, radius=0.01, color=COLOR_RED):
        on_board_pos = self.get_on_board_pos(pos)
        on_board_radius = self.get_on_board_dim(radius)
        pg.draw.circle(self.screen, color, on_board_pos, on_board_radius)

    def screen_to_world(self, screen_pos):
        # Inverse of get_on_board_pos. Returns None if outside the board.
        sx, sy = screen_pos
        if self.rl_to_px_ratio <= 0:
            return None
        wx = (sx - self.dim_x[0]) / self.rl_to_px_ratio + self.mins[0]
        wy = (self.dim_y[1] - sy) / self.rl_to_px_ratio + self.mins[1]
        if not (self.mins[0] <= wx <= self.maxs[0] and self.mins[1] <= wy <= self.maxs[1]):
            return None
        return (wx, wy)

    def loop(self, viewport=None):
        if viewport is None:
            info = pg.display.Info()
            vp_x, vp_y, vp_w, vp_h = 0, 0, info.current_w, info.current_h
        else:
            vp_x, vp_y, vp_w, vp_h = viewport

        max_w = vp_h * self.bg_ratio
        desired_w = min(vp_w, max_w)
        desired_h = desired_w / self.bg_ratio
        margin_w = (vp_w - desired_w) / 2
        margin_h = (vp_h - desired_h) / 2
        background = pg.transform.smoothscale(self.bg, (desired_w, desired_h))
        board_x = vp_x + margin_w
        board_y = vp_y + margin_h
        self.dim_x = [board_x, board_x + background.get_width()]
        self.dim_y = [board_y, board_y + background.get_height()]
        self.rl_to_px_ratio = (self.dim_x[1] - self.dim_x[0]) / self.real_size[0]
        self.screen.blit(background, (self.dim_x[0], self.dim_y[0]))

        for key, obstacle in self.world.obstacles.items():
            self.draw_obstacle(obstacle)

        for name, robot in self.world.robots.items():
            self.draw_robot(robot)
            for point in robot.lidar_points:
                self.draw_debug_point(point)
            if self.display_wps:
                for wp in robot.wps:
                    self.draw_debug_point((wp[0], wp[1]), color=COLOR_PURPLE)

def copy_to_clipboard(text: str) -> str:
    """Return the backend name that succeeded, or '' on failure."""
    for cmd in (["wl-copy"], ["xclip", "-selection", "clipboard"], ["xsel", "-bi"]):
        if shutil.which(cmd[0]):
            try:
                subprocess.run(cmd, input=text.encode("utf-8"), check=True, timeout=2)
                return cmd[0]
            except Exception as exc:
                logger.warning(f"{cmd[0]} failed: {exc}")
    try:
        if not pg.scrap.get_init():
            pg.scrap.init()
        pg.scrap.put(pg.SCRAP_TEXT, text.encode("utf-8"))
        return "pygame.scrap"
    except Exception as exc:
        logger.warning(f"pygame.scrap failed: {exc}")
    return ""


class PathSimulator:
    PREVIEW_KEY = "sim_preview"

    def __init__(self, world: World, planner=None, on_user_running=None):
        self.world = world
        self.planner = planner
        self.robot = Robot(team=0)
        self.wps = []
        self.wp_index = 0
        self.started = False
        self.running = False
        self.speed = 500.0  # mm/s
        self.angle_speed = math.pi / 2  # rad/s
        self.pos_tol = 5.0  # mm
        self.ang_tol = 0.02  # rad
        self._on_user_paused = on_user_running or (lambda running: None)

    def _sync_team(self):
        if self.planner is not None:
            self.robot.team = 1 if self.planner.side == "yellow" else 0

    def _load(self, waypoints):
        if not waypoints:
            return False
        self._sync_team()
        self.wps = [np.array([x, y, a], dtype=float) for (x, y, a) in waypoints]
        first = self.wps[0]
        self.robot.pos = np.array([first[0], first[1], first[2]], dtype=float)
        self.robot.dir = first[2]
        self.robot.wps = self.wps[:]
        self.wp_index = 1 if len(self.wps) > 1 else len(self.wps)
        self.world.robots[self.PREVIEW_KEY] = self.robot
        self.started = True
        return True

    def play_pause(self, waypoints):
        if not self.started or self.wp_index >= len(self.wps):
            if not self._load(waypoints):
                return
            self.running = self.wp_index < len(self.wps)
        else:
            self.running = not self.running
        self._on_user_paused(self.running)

    def reset(self, waypoints):
        self._load(waypoints)
        self.running = False
        self._on_user_paused(False)

    def stop(self):
        self.running = False
        self.started = False
        self.world.robots.pop(self.PREVIEW_KEY, None)
        self._on_user_paused(False)

    def _avoidance_velocity(self, desired_dir, dist_to_target):
        """Potential-field steering. desired_dir is a unit vector toward the target."""
        influence = 350.0  # mm — start reacting at this clearance
        k_rep = 800.0
        cmd = desired_dir * self.speed
        rp = self.robot.pos[:2]
        for obs in list(self.world.obstacles.values()):
            try:
                op = np.array(obs.get_pos()[:2], dtype=float)
            except Exception:
                continue
            obs_r = float(getattr(obs, "radius", 100.0))
            diff = rp - op
            d = float(np.linalg.norm(diff))
            clearance = d - obs_r - self.robot.radius
            if d < 1e-3 or clearance > influence:
                continue
            # smooth, asymptotic repulsion as clearance shrinks toward 0
            c = max(clearance, 5.0)
            strength = k_rep * (1.0 / c - 1.0 / influence)
            cmd = cmd + (diff / d) * strength * self.speed
        # cap magnitude to nominal speed
        mag = float(np.linalg.norm(cmd))
        if mag > self.speed:
            cmd = cmd / mag * self.speed
        return cmd

    def step(self, dt):
        self._sync_team()
        if not self.running or self.wp_index >= len(self.wps):
            self.running = False
            return
        target = self.wps[self.wp_index]
        delta = target - self.robot.pos
        delta_xy = delta[:2]
        delta_a = (float(delta[2]) + math.pi) % (2 * math.pi) - math.pi
        dist = float(np.linalg.norm(delta_xy))

        if dist > self.pos_tol:
            desired_dir = delta_xy / dist
            vel = self._avoidance_velocity(desired_dir, dist)
            step_xy = vel * dt
            # do not overshoot when close to target
            step_dist = float(np.linalg.norm(step_xy))
            if step_dist > dist:
                step_xy = step_xy / step_dist * dist
        else:
            step_xy = np.zeros(2)

        step_a = max(-self.angle_speed * dt, min(self.angle_speed * dt, delta_a))
        self.robot.pos = self.robot.pos + np.array([step_xy[0], step_xy[1], step_a])
        if dist > self.pos_tol and float(np.linalg.norm(step_xy)) > 1e-3:
            self.robot.dir = math.atan2(step_xy[1], step_xy[0])
        else:
            self.robot.dir = float(self.robot.pos[2])
        if dist < self.pos_tol and abs(delta_a) < self.ang_tol:
            self.wp_index += 1
            self.robot.wps = self.wps[self.wp_index:]
            if self.wp_index >= len(self.wps):
                self.running = False


class PathPlanner:
    HIT_RADIUS_MM = 80.0

    def __init__(self):
        self.waypoints = []  # list of (x, y, angle)
        self.side = "blue"
        # snap state
        self.grid_enabled = True
        self.grid_size = 50.0  # mm
        self.angle_enabled = True
        self.angle_step = math.pi / 4  # 45deg
        # drag state
        self._mode = None  # None | "create" | "move" | "rotate"
        self._edit_index = None
        self._drag_start = None
        self._drag_current = None
        # hover state — used for tooltip + cycling overlapped waypoints
        self._hover_world = None
        self._hover_indices = []  # all wps within HIT_RADIUS, sorted by distance
        self._hover_cycle = 0     # which one of _hover_indices is "active"

    @staticmethod
    def _wrap_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def mirror_y_axis(self):
        """Match convert_pos_for_team_no_angle: flip x, keep angle."""
        self.waypoints = [(-x, y, a) for (x, y, a) in self.waypoints]

    def _snap_pos(self, pos):
        if not self.grid_enabled or self.grid_size <= 0:
            return pos
        gs = self.grid_size
        return (round(pos[0] / gs) * gs, round(pos[1] / gs) * gs)

    def _snap_angle(self, angle):
        if not self.angle_enabled or self.angle_step <= 0:
            return angle
        return round(angle / self.angle_step) * self.angle_step

    def find_indices_near(self, world_pos):
        if world_pos is None:
            return []
        wx, wy = world_pos
        r2 = self.HIT_RADIUS_MM ** 2
        hits = []
        for i, (x, y, _) in enumerate(self.waypoints):
            d2 = (x - wx) ** 2 + (y - wy) ** 2
            if d2 <= r2:
                hits.append((d2, i))
        hits.sort()
        return [i for _, i in hits]

    def find_index_near(self, world_pos):
        idxs = self.find_indices_near(world_pos)
        return idxs[0] if idxs else None

    def set_hover(self, world_pos):
        # called on MOUSEMOTION while not dragging
        self._hover_world = world_pos
        prev = list(self._hover_indices)
        self._hover_indices = self.find_indices_near(world_pos)
        # reset cycle if the set of overlapping wps changed
        if self._hover_indices != prev:
            self._hover_cycle = 0
        elif self._hover_indices:
            self._hover_cycle %= len(self._hover_indices)

    def cycle_hover(self, delta: int):
        if not self._hover_indices:
            return
        self._hover_cycle = (self._hover_cycle + int(delta)) % len(self._hover_indices)

    def _hovered_index(self):
        if not self._hover_indices:
            return None
        return self._hover_indices[self._hover_cycle % len(self._hover_indices)]

    def delete_at(self, world_pos) -> bool:
        # delete the currently-active hovered wp if any, otherwise nearest
        i = self._hovered_index() if self._hover_world == world_pos else None
        if i is None:
            i = self.find_index_near(world_pos)
        if i is None:
            return False
        self.waypoints.pop(i)
        self._hover_indices = []
        self._hover_cycle = 0
        return True

    def start_drag(self, world_pos, shift=False):
        if world_pos is None:
            return
        # honor the active hovered wp so overlap-cycling actually picks the one shown
        hit = self._hovered_index() if self._hover_world == world_pos else None
        if hit is None:
            hit = self.find_index_near(world_pos)
        if hit is not None:
            self._edit_index = hit
            self._mode = "rotate" if shift else "move"
        else:
            self._mode = "create"
            self._drag_start = self._snap_pos(world_pos)
        self._drag_current = world_pos

    def update_drag(self, world_pos):
        if self._mode is None or world_pos is None:
            return
        self._drag_current = world_pos
        if self._mode == "move":
            x, y = self._snap_pos(world_pos)
            _, _, a = self.waypoints[self._edit_index]
            self.waypoints[self._edit_index] = (x, y, a)
        elif self._mode == "rotate":
            x, y, _ = self.waypoints[self._edit_index]
            dx, dy = world_pos[0] - x, world_pos[1] - y
            if dx * dx + dy * dy >= 1.0:
                a = self._snap_angle(math.atan2(dy, dx))
                self.waypoints[self._edit_index] = (x, y, a)

    def end_drag(self, world_pos):
        if self._mode == "create" and self._drag_start is not None:
            end = world_pos if world_pos is not None else self._drag_current
            sx, sy = self._drag_start
            if end is None:
                end = (sx, sy)
            ex, ey = end
            if (ex - sx) ** 2 + (ey - sy) ** 2 < 1.0:
                angle = self.waypoints[-1][2] if self.waypoints else 0.0
            else:
                angle = self._snap_angle(math.atan2(ey - sy, ex - sx))
            self.waypoints.append((sx, sy, angle))
        self._mode = None
        self._edit_index = None
        self._drag_start = None
        self._drag_current = None

    def undo(self):
        if self.waypoints:
            self.waypoints.pop()

    def clear(self):
        self.waypoints.clear()
        self._mode = None
        self._edit_index = None
        self._drag_start = None
        self._drag_current = None

    def pending(self):
        if self._mode != "create" or self._drag_start is None:
            return None
        sx, sy = self._drag_start
        if self._drag_current is None:
            return (sx, sy, 0.0)
        ex, ey = self._drag_current
        if (ex - sx) ** 2 + (ey - sy) ** 2 < 1.0:
            return (sx, sy, 0.0)
        return (sx, sy, self._snap_angle(math.atan2(ey - sy, ex - sx)))

    def to_c_string(self) -> str:
        """Self-contained one-liner per waypoint — paste any line into match().
        Each line uses a compound literal so it does not depend on prior declarations.
        Insert actuator calls (pokarm_pinch(true); etc.) between lines."""
        if not self.waypoints:
            return "// no waypoints\n"
        lines = []
        for i, (x, y, a) in enumerate(self.waypoints, start=1):
            lines.append(
                f"nav_go_to_direct(convert_pos_for_team_no_angle(color, "
                f"(pos2_t){{.x = {x:.3f}f, .y = {y:.3f}f, .a = {a:.6f}f}}, 0.0f), K_FOREVER); "
                f"nav_wait_events(&nav_events); // step {i}"
            )
        return "\n".join(lines) + "\n"

    def export(self, path):
        text = self.to_c_string()
        with open(path, "w") as f:
            f.write(text)
        return path

    def draw_grid(self, game_viz, screen):
        if not self.grid_enabled or self.grid_size <= 0:
            return
        x_min, x_max = game_viz.mins[0], game_viz.maxs[0]
        y_min, y_max = game_viz.mins[1], game_viz.maxs[1]
        board_left = int(game_viz.dim_x[0])
        board_top = int(game_viz.dim_y[0])
        board_w = int(game_viz.dim_x[1] - game_viz.dim_x[0])
        board_h = int(game_viz.dim_y[1] - game_viz.dim_y[0])
        if board_w <= 0 or board_h <= 0:
            return
        surf = pg.Surface((board_w, board_h), pg.SRCALPHA)
        color = (255, 255, 255, 35)
        gs = self.grid_size
        x = math.ceil(x_min / gs) * gs
        while x <= x_max:
            p1 = game_viz.get_on_board_pos((x, y_min))
            p2 = game_viz.get_on_board_pos((x, y_max))
            pg.draw.line(surf, color, (p1[0] - board_left, p1[1] - board_top), (p2[0] - board_left, p2[1] - board_top), 1)
            x += gs
        y = math.ceil(y_min / gs) * gs
        while y <= y_max:
            p1 = game_viz.get_on_board_pos((x_min, y))
            p2 = game_viz.get_on_board_pos((x_max, y))
            pg.draw.line(surf, color, (p1[0] - board_left, p1[1] - board_top), (p2[0] - board_left, p2[1] - board_top), 1)
            y += gs
        screen.blit(surf, (board_left, board_top))

    def _draw_arrow(self, screen, start, end, color):
        # halo + bright stroke + filled arrowhead
        ax, ay = start
        bx, by = end
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 1:
            return
        ux, uy = dx / L, dy / L
        head_len = max(10.0, min(18.0, L * 0.4))
        head_w = head_len * 0.7
        shaft_end = (bx - ux * head_len * 0.7, by - uy * head_len * 0.7)
        # dark halo
        pg.draw.line(screen, COLOR_WP_HALO, start, shaft_end, width=7)
        # bright shaft
        pg.draw.line(screen, color, start, shaft_end, width=3)
        # arrowhead triangle
        base = (bx - ux * head_len, by - uy * head_len)
        left = (base[0] - uy * head_w * 0.5, base[1] + ux * head_w * 0.5)
        right = (base[0] + uy * head_w * 0.5, base[1] - ux * head_w * 0.5)
        pg.draw.polygon(screen, COLOR_WP_HALO, [
            (bx + ux * 1, by + uy * 1),
            (left[0] - uy * 1, left[1] + ux * 1),
            (right[0] + uy * 1, right[1] - ux * 1),
        ])
        pg.draw.polygon(screen, color, [(bx, by), left, right])

    def _draw_segment_alpha(self, screen, start, end, alpha, color=COLOR_WP):
        if alpha >= 255:
            pg.draw.line(screen, COLOR_WP_HALO, start, end, width=6)
            pg.draw.line(screen, color, start, end, width=3)
            return
        pad = 6  # halo half-width + headroom
        x0 = int(min(start[0], end[0])) - pad
        y0 = int(min(start[1], end[1])) - pad
        x1 = int(max(start[0], end[0])) + pad
        y1 = int(max(start[1], end[1])) + pad
        w, h = max(1, x1 - x0), max(1, y1 - y0)
        overlay = pg.Surface((w, h), pg.SRCALPHA)
        s = (start[0] - x0, start[1] - y0)
        e = (end[0] - x0, end[1] - y0)
        pg.draw.line(overlay, COLOR_WP_HALO, s, e, width=6)
        pg.draw.line(overlay, color, s, e, width=3)
        overlay.set_alpha(alpha)
        screen.blit(overlay, (x0, y0))

    def _draw_arrow_alpha(self, screen, start, end, color, alpha):
        if alpha >= 255:
            self._draw_arrow(screen, start, end, color)
            return
        # render onto a small local overlay sized to the arrow bbox
        pad = 10  # covers halo + arrowhead width
        x0 = int(min(start[0], end[0])) - pad
        y0 = int(min(start[1], end[1])) - pad
        x1 = int(max(start[0], end[0])) + pad
        y1 = int(max(start[1], end[1])) + pad
        w, h = max(1, x1 - x0), max(1, y1 - y0)
        overlay = pg.Surface((w, h), pg.SRCALPHA)
        self._draw_arrow(overlay, (start[0] - x0, start[1] - y0), (end[0] - x0, end[1] - y0), color)
        overlay.set_alpha(alpha)
        screen.blit(overlay, (x0, y0))

    def _draw_marker_alpha(self, screen, sp, color, label_text, font, alpha):
        if alpha >= 255:
            self._draw_marker(screen, sp, color, label_text, font)
            return
        # local overlay sized to the marker bbox; cheaper than full-screen overlay
        outer_r = 13
        pad = outer_r + 6
        size = pad * 2
        overlay = pg.Surface((size, size), pg.SRCALPHA)
        self._draw_marker(overlay, (pad, pad), color, label_text, font)
        overlay.set_alpha(alpha)
        screen.blit(overlay, (sp[0] - pad, sp[1] - pad))

    def _draw_hover_tooltip(self, screen, font, idx, mouse_screen_pos):
        if idx >= len(self.waypoints):
            return
        x, y, a = self.waypoints[idx]
        a_deg = math.degrees(a)
        lines = [
            f"#{idx + 1}  x={x:.0f}  y={y:.0f}",
            f"a={a:.3f} rad ({a_deg:.1f}°)",
        ]
        if len(self._hover_indices) > 1:
            pos = (self._hover_cycle % len(self._hover_indices)) + 1
            lines.append(f"overlap {pos}/{len(self._hover_indices)} — scroll to cycle")
        surfs = [font.render(line, True, SIDEBAR_FG) for line in lines]
        pad = 6
        w = max(s.get_width() for s in surfs) + pad * 2
        h = sum(s.get_height() for s in surfs) + pad * 2
        sw, sh = screen.get_size()
        tx = min(mouse_screen_pos[0] + 16, sw - w - 4)
        ty = min(mouse_screen_pos[1] + 16, sh - h - 4)
        bg = pg.Surface((w, h), pg.SRCALPHA)
        bg.fill((10, 10, 14, 220))
        screen.blit(bg, (tx, ty))
        pg.draw.rect(screen, SIDEBAR_DIM, pg.Rect(tx, ty, w, h), width=1, border_radius=4)
        cy = ty + pad
        for s in surfs:
            screen.blit(s, (tx + pad, cy))
            cy += s.get_height()

    def _draw_marker(self, screen, sp, color, label_text, font):
        outer_r = 13
        inner_r = 9
        # drop shadow
        pg.draw.circle(screen, COLOR_WP_HALO, (sp[0] + 1, sp[1] + 2), outer_r + 2)
        # dark outer ring
        pg.draw.circle(screen, COLOR_WP_HALO, sp, outer_r + 2)
        # bright accent ring
        pg.draw.circle(screen, color, sp, outer_r)
        # white inner disk for legibility of the index
        pg.draw.circle(screen, (245, 245, 245), sp, inner_r)
        if label_text:
            label = font.render(label_text, True, COLOR_WP_HALO)
            screen.blit(label, (sp[0] - label.get_width() // 2, sp[1] - label.get_height() // 2))

    # Gradient stops from "current target" to "far / passed": radiant cyan-blue
    # → blue → purple → near-black. Sampled by _color_for().
    _GRADIENT = [
        (0x00, 0xE5, 0xFF),  # 0 — radiant cyan (next target)
        (0x3A, 0x7B, 0xFF),  # 1 — bright blue
        (0x6C, 0x3A, 0xD0),  # 2 — violet
        (0x32, 0x16, 0x60),  # 3 — deep purple
        (0x12, 0x08, 0x24),  # 4 — near-black
    ]

    def _path_offset(self, idx, sim) -> float:
        """Continuous offset (>=0) along the path from sim.wp_index.
        Passed waypoints saturate at the max offset so they sit at the darkest stop."""
        if sim is None or not getattr(sim, "started", False):
            return 0.0
        wp_index = getattr(sim, "wp_index", idx)
        if idx < wp_index:
            return float(len(self._GRADIENT) - 1)
        return float(idx - wp_index)

    def _alpha_for(self, idx, sim) -> int:
        if sim is None or not getattr(sim, "started", False):
            return 255
        min_alpha, step = 60, 50
        offset = self._path_offset(idx, sim)
        # Passed waypoints (offset saturated) → min_alpha directly
        wp_index = getattr(sim, "wp_index", idx)
        if idx < wp_index:
            return min_alpha
        return max(min_alpha, 255 - int(offset) * step)

    def _color_for(self, idx, sim):
        if sim is None or not getattr(sim, "started", False):
            return self._GRADIENT[0]
        offset = self._path_offset(idx, sim)
        last = len(self._GRADIENT) - 1
        t = max(0.0, min(offset, last))
        lo = int(t)
        hi = min(lo + 1, last)
        frac = t - lo
        c0 = self._GRADIENT[lo]
        c1 = self._GRADIENT[hi]
        return (
            int(c0[0] + (c1[0] - c0[0]) * frac),
            int(c0[1] + (c1[1] - c0[1]) * frac),
            int(c0[2] + (c1[2] - c0[2]) * frac),
        )

    def draw(self, game_viz, screen, sim=None, mouse_screen_pos=None):
        arrow_len_world = 180  # mm
        try:
            font = pg.font.SysFont("monospace", 14, bold=True)
        except Exception:
            font = pg.font.Font(None, 16)

        hovered = self._hovered_index()

        # Precompute screen positions, per-waypoint alpha + gradient color
        sps = [game_viz.get_on_board_pos((x, y)) for (x, y, _) in self.waypoints]
        wp_alpha = [self._alpha_for(i, sim) for i in range(len(self.waypoints))]
        wp_color = [self._color_for(i, sim) for i in range(len(self.waypoints))]

        # Two-pass draw so opaque (important) items sit on top of faded ones.
        # Pass 0: alpha < 255 (passed / far-future), Pass 1: alpha == 255.
        for opaque_pass in (False, True):
            # Path segments — use the "more important" endpoint's color
            if len(self.waypoints) >= 2:
                for i in range(len(sps) - 1):
                    alpha = max(wp_alpha[i], wp_alpha[i + 1])
                    if (alpha >= 255) != opaque_pass:
                        continue
                    color = wp_color[i] if wp_alpha[i] >= wp_alpha[i + 1] else wp_color[i + 1]
                    self._draw_segment_alpha(screen, sps[i], sps[i + 1], alpha, color)
            # Direction arrows
            for i, (x, y, a) in enumerate(self.waypoints):
                alpha = wp_alpha[i]
                if (alpha >= 255) != opaque_pass:
                    continue
                sp = sps[i]
                tip = game_viz.get_on_board_pos((x + arrow_len_world * math.cos(a), y + arrow_len_world * math.sin(a)))
                self._draw_arrow_alpha(screen, sp, tip, wp_color[i], alpha)
            # Markers
            for i, (x, y, a) in enumerate(self.waypoints):
                alpha = wp_alpha[i]
                if (alpha >= 255) != opaque_pass:
                    continue
                color = COLOR_WP_PENDING if i == hovered else wp_color[i]
                self._draw_marker_alpha(screen, sps[i], color, str(i + 1), font, alpha)

        pending = self.pending()
        if pending is not None:
            x, y, a = pending
            sp = game_viz.get_on_board_pos((x, y))
            tip = game_viz.get_on_board_pos((x + arrow_len_world * math.cos(a), y + arrow_len_world * math.sin(a)))
            self._draw_arrow(screen, sp, tip, COLOR_WP_PENDING)
            self._draw_marker(screen, sp, COLOR_WP_PENDING, "", font)

        # Hover tooltip
        if hovered is not None and mouse_screen_pos is not None:
            self._draw_hover_tooltip(screen, font, hovered, mouse_screen_pos)


class KeyBindings:
    """Holds two independent collections:
    - handlers: (key, callable, repeat) — what actually fires on input.
    - docs: (display, label, repeat) — what the keybindings panel shows.

    Several handlers can share a single doc row (e.g. arrow keys → "move obstacle")
    by registering handlers with `bind(..., document=False)` and then calling
    `document(display, label, ...)` once for the group.
    """

    def __init__(self):
        self._handlers = []  # (key, handler, repeat)
        self._docs = []      # (display, label, repeat)

    def bind(self, key, handler, repeat=False, doc=None):
        self._handlers.append((key, handler, repeat))
        if doc is not None:
            self._docs.append((_key_name(key), doc, repeat))

    def document(self, display, label, repeat=False):
        self._docs.append((display, label, repeat))

    def dispatch(self, event):
        if event.type != pg.KEYDOWN:
            return
        for key, handler, repeat in self._handlers:
            if not repeat and key == event.key:
                handler()

    def poll(self, pressed):
        for key, handler, repeat in self._handlers:
            if repeat and pressed[key]:
                handler()

    def docs(self):
        return list(self._docs)


def _key_name(key):
    name = pg.key.name(key)
    return name.upper() if len(name) == 1 else name


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


def state_load(path: str):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as exc:
        logger.warning(f"state load failed: {exc}")
        return None


def state_save(path: str, side: str, waypoints):
    try:
        with open(path, "w") as f:
            json.dump({
                "side": side,
                "waypoints": [[x, y, a] for (x, y, a) in waypoints],
            }, f, indent=2)
    except Exception as exc:
        logger.warning(f"state save failed: {exc}")


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
