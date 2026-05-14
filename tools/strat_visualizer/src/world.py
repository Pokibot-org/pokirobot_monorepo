"""World, Robot, and Obstacle types — pure data + collision helpers, no rendering."""
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry


class Obstacle:
    def __init__(self, shape: Polygon) -> None:
        self.shape = shape

    def get_shape(self):
        return self.shape

    def update_shape(self, shape):
        self.shape = shape

    def set_pos(self, pos):
        pass

    def get_pos(self):
        return np.array([0.0, 0.0])


def get_closest_collision_point(shape: BaseGeometry, ray_origin: Point, ray_direction: Point, ray_len=1000):
    ray_end = Point(ray_origin.x + ray_direction.x * ray_len, ray_origin.y + ray_direction.y * ray_len)
    ray = LineString([ray_origin, ray_end])

    intersection = ray.intersection(shape)

    if intersection.is_empty:
        return None

    if isinstance(intersection, Point):
        return intersection

    if intersection.geom_type in ["MultiPoint", "GeometryCollection"]:
        points = [pt for pt in intersection.geoms if isinstance(pt, Point)]
        if not points:
            return None
        closest_point = min(points, key=lambda p: p.distance(Point(ray_origin)))
        return closest_point

    if isinstance(intersection, LineString):
        first_point = Point(intersection.coords[0])
        last_point = Point(intersection.coords[-1])
        return min([first_point, last_point], key=lambda p: p.distance(Point(ray_origin)))

    return None


class CircleObstacle(Obstacle):
    def __init__(self, pos, radius) -> None:
        self.pos = np.array(pos)
        self.radius = radius
        shape = Point(pos[0], pos[1]).buffer(radius)
        super().__init__(shape)

    def set_pos(self, pos):
        shape = Point(pos[0], pos[1]).buffer(self.radius)
        self.update_shape(shape)
        self.pos = pos

    def get_pos(self):
        return self.pos


class RobotObstacle(Obstacle):
    def __init__(self, pos, radius, scan_radius=0.035) -> None:
        self.pos = np.array(pos)
        self.radius = radius
        self.scan_radius = scan_radius
        shape = Point(pos[0], pos[1]).buffer(scan_radius)
        super().__init__(shape)

    def set_pos(self, pos):
        shape = Point(pos[0], pos[1]).buffer(self.scan_radius)
        self.update_shape(shape)
        self.pos = pos

    def get_pos(self):
        return self.pos


class Robot:
    def __init__(self, radius=180, team=0) -> None:
        self.radius = radius
        self.pos = np.array([0.0, 1000, 0.0])
        self.dir = 0.0
        self.team = team
        self.lidar_points: list[tuple[float, float]] = []
        self.wps: list[NDArray] = []
        self.tirette = 2  # IN


@dataclass
class World:
    obstacles: dict[str, Obstacle] = field(default_factory=dict)
    robots: dict[str, Robot] = field(default_factory=dict)
