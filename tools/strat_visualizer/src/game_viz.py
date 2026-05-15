"""GameZoneVisualizer — renders the board background, obstacles, and robots."""
import math
import os

import pygame as pg

from constants import (
    COLOR_BLACK, COLOR_DEEPBLACK, COLOR_GRAY, COLOR_GREEN, COLOR_PURPLE,
    COLOR_RED, RIGHT_MARGIN_RATIO,
)
from world import CircleObstacle, Obstacle, Robot, RobotObstacle, World

_main_dir = os.path.split(os.path.abspath(__file__))[0]


def load_image(name):
    path = os.path.join(_main_dir, "../img", name)
    return pg.image.load(path).convert()


class GameZoneVisualizer:
    ROBOT_IMG_MM_PER_PX = 1.0  # source assets: 1 pixel = 1 mm
    # Cap the source bg surface so the per-frame fit-scale is cheap. The
    # final blit is bounded by the screen (~2k wide tops), so a 4k source
    # is already 2× supersampled — anything beyond that is wasted work
    # (and decompressed RAM). The 2026 raw asset is 10630×7087.
    MAX_BG_SRC_WIDTH_PX = 4000

    def __init__(self, world: World, screen, competition):
        self.world = world
        self.screen = screen
        self.competition = competition
        self.right_margin_ratio = RIGHT_MARGIN_RATIO

        w, h = competition.table_size_mm
        x0, y0 = competition.table_mins
        self.real_size = [w, h]
        self.mins = [x0, y0]
        self.maxs = [x0 + w, y0 + h]
        self.dim_x = [0, 1]
        self.dim_y = [0, 1]
        self.rl_to_px_ratio = 1000
        self.display_wps = True
        raw_bg = pg.image.load(competition.bg_image_path).convert()
        if raw_bg.get_width() > self.MAX_BG_SRC_WIDTH_PX:
            new_w = self.MAX_BG_SRC_WIDTH_PX
            new_h = max(1, int(round(raw_bg.get_height() * new_w / raw_bg.get_width())))
            self.bg = pg.transform.smoothscale(raw_bg, (new_w, new_h))
        else:
            self.bg = raw_bg
        self.bg_ratio = self.bg.get_width() / self.bg.get_height()
        self._bg_scaled = None  # (w, h, Surface) — cached fit-scale, invalidated on size change
        # Robot art — index by team (0=blue, 1=yellow). Source assets face +Y
        # (up) in the image, so we apply a -90° offset when rotating.
        self._robot_imgs = {
            team: pg.image.load(path).convert_alpha()
            for team, path in competition.robot_images.items()
        }
        self._robot_scaled = {}  # (team, ratio) -> scaled Surface

    def get_on_board_pos(self, pos):
        return (
            self.dim_x[0] + (pos[0] - self.mins[0]) * self.rl_to_px_ratio,
            self.dim_y[1] - (pos[1] - self.mins[1]) * self.rl_to_px_ratio,
        )

    def get_on_board_dim(self, dim):
        return dim * self.rl_to_px_ratio

    def _scaled_robot_img(self, team: int):
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
        rotated = pg.transform.rotate(scaled, math.degrees(robot.pos[2]))
        rect = rotated.get_rect(center=on_board_pos)
        self.screen.blit(rotated, rect)
        # Debug overlay: commanded heading (red) vs actual pos[2] angle (gray)
        pg.draw.line(
            self.screen, COLOR_RED, on_board_pos,
            (on_board_pos[0] + on_board_radius * math.cos(robot.dir),
             on_board_pos[1] + on_board_radius * math.sin(robot.dir + math.pi)),
            width=4,
        )
        pg.draw.line(
            self.screen, COLOR_GRAY, on_board_pos,
            (on_board_pos[0] + on_board_radius * math.cos(robot.pos[2]),
             on_board_pos[1] + on_board_radius * math.sin(robot.pos[2] + math.pi)),
            width=4,
        )

    def draw_obstacle(self, obstacle: Obstacle):
        if type(obstacle) is CircleObstacle:
            on_board_pos = self.get_on_board_pos(obstacle.pos)
            on_board_radius = self.get_on_board_dim(obstacle.radius)
            pg.draw.circle(self.screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
            pg.draw.circle(self.screen, COLOR_GREEN, on_board_pos, on_board_radius * 0.98)
        elif type(obstacle) is RobotObstacle:
            on_board_pos = self.get_on_board_pos(obstacle.pos)
            on_board_radius = self.get_on_board_dim(obstacle.radius)
            on_board_scan_radius = self.get_on_board_dim(obstacle.scan_radius)
            pg.draw.circle(self.screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
            pg.draw.circle(self.screen, COLOR_GREEN, on_board_pos, on_board_radius * 0.98)
            pg.draw.circle(self.screen, COLOR_BLACK, on_board_pos, on_board_scan_radius)

    def draw_debug_point(self, pos, radius=0.01, color=COLOR_RED):
        on_board_pos = self.get_on_board_pos(pos)
        on_board_radius = self.get_on_board_dim(radius)
        pg.draw.circle(self.screen, color, on_board_pos, on_board_radius)

    def screen_to_world(self, screen_pos):
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
        target_size = (int(desired_w), int(desired_h))
        if self._bg_scaled is None or self._bg_scaled[0] != target_size:
            self._bg_scaled = (target_size, pg.transform.smoothscale(self.bg, target_size))
        background = self._bg_scaled[1]
        board_x = vp_x + margin_w
        board_y = vp_y + margin_h
        self.dim_x = [board_x, board_x + background.get_width()]
        self.dim_y = [board_y, board_y + background.get_height()]
        self.rl_to_px_ratio = (self.dim_x[1] - self.dim_x[0]) / self.real_size[0]
        self.screen.blit(background, (self.dim_x[0], self.dim_y[0]))

        for _, obstacle in self.world.obstacles.items():
            self.draw_obstacle(obstacle)

        for _, robot in self.world.robots.items():
            self.draw_robot(robot)
            for point in robot.lidar_points:
                self.draw_debug_point(point)
            if self.display_wps:
                for wp in robot.wps:
                    self.draw_debug_point((wp[0], wp[1]), color=COLOR_PURPLE)
