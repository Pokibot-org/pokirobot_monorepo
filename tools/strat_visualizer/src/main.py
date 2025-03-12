#!/usr/bin/env python
import os
import pygame as pg
import threading
import json
import math
from msm import MqttSimMessengerServer, SimNode
import time
import logging
import numpy as np
from shapely.geometry import LineString, Point, Polygon

logger = logging.getLogger("strat_visu")

main_dir = os.path.split(os.path.abspath(__file__))[0]


class Obstacle:
    def __init__(self, shape: Polygon) -> None:
        self.shape = shape

    def get_closest_collision_point(self, ray_origin, ray_direction, ray_len = 1000):
        # Create a long line segment in the direction of the ray
        ray_end = Point(ray_origin.x + ray_direction.x * ray_len, ray_origin.y + ray_direction.y * ray_len)
        ray = LineString([ray_origin, ray_end])

        # Find the intersection points
        intersection = ray.intersection(self.shape)

        if intersection.is_empty:
            return None  # No intersection

        # If there are multiple intersection points, find the closest one
        if intersection.geom_type == 'MultiPoint' or intersection.geom_type == 'GeometryCollection':
            closest_point = None
            min_distance = float('inf')

            for point in intersection:
                distance = ray_origin.distance(point)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point

            return closest_point
        else:
            return intersection

class CircleObstacle(Obstacle):
    def __init__(self, pos, radius) -> None:
        self.pos = pos
        self.radius = radius
        shape = Point(pos[0], pos[1]).buffer(radius)
        super().__init__(shape)

class Board:
    def __init__(self) -> None:
        self.real_size = [3.0, 2.0]
        self.mins = [-1.5, 0.0]
        self.maxs = [1.5, 2.0]
        self.dim_x = None
        self.dim_y = None


# quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, "../img", name)
    return pg.image.load(path).convert()


def draw_robot(screen, board, robot):
    ratio = board.dim_x[1] / board.real_size[0]
    on_board_radius = robot.radius * ratio
    on_board_pos = ((robot.pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (robot.pos[1] - board.mins[1]) * ratio)

    pg.draw.circle(screen, (30, 30, 100), on_board_pos, on_board_radius)
    pg.draw.line(
        screen,
        (130, 30, 30),
        on_board_pos,
        (
            on_board_pos[0] + on_board_radius * math.cos(robot.pos[2]),
            on_board_pos[1] + on_board_radius * math.sin(robot.pos[2] + math.pi),
        ),
        width=4,
    )

def draw_obstacle(screen, board, obstacle: CircleObstacle):
    ratio = board.dim_x[1] / board.real_size[0]
    on_board_radius = obstacle.radius * ratio
    on_board_pos = ((obstacle.pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (obstacle.pos[1] - board.mins[1]) * ratio)

    pg.draw.circle(screen, (30, 50, 30), on_board_pos, on_board_radius)

class SimNodeWithClbk(SimNode):
    def __init__(self, parent, id, name, clbk) -> None:
        super().__init__(parent, id, name)
        self.clbk = clbk

    def process_topic(self, topic, payload):
        self.clbk(self, topic, payload)

class PokibotSimNode(SimNode):
    def __init__(self, parent, id) -> None:
        super().__init__(parent, id, "")
        self.radius = 0.19
        self.pos = np.array([0, 0, 0])
        self.team = 0
        self.wp_index = 0
        self.wps = []
        self.motor_break = False
        self.speed = 0.5
        self.angle_speed = 3.14/2
        self.sensivity = 0.01

        self.pokuicom = SimNodeWithClbk(self, 0, "pokuicom", self.process_pokuicom)
        self.add_child(self.pokuicom)
        self.pokuicom.subscribe("request")
        self.pokuicom.subscribe("score")

        self.poklegscom = SimNodeWithClbk(self, 0, "poklegscom", self.process_poklegscom)
        self.poklegscom.subscribe("set_pos")
        self.poklegscom.subscribe("set_waypoints")
        self.poklegscom.subscribe("set_break")
        self.add_child(self.poklegscom)

        self.start_simulation()

    def _sim_loop(self):
        sim_tick = 30
        time.sleep(1/sim_tick)
        max_speed_vec = np.array([self.speed/sim_tick, self.speed/sim_tick, self.angle_speed/sim_tick])
        sensivity = np.array([self.sensivity, self.sensivity, self.sensivity])
        if self.wp_index < len(self.wps) and not self.motor_break:
            target_pos = self.wps[self.wp_index]
            self.pos += np.maximum(np.minimum(target_pos - self.pos, max_speed_vec), -max_speed_vec)
            if np.all(np.abs(target_pos - self.pos) < sensivity):
                self.wp_index += 1
        self.poklegscom.send("pos", f"{self.pos[0]} {self.pos[1]} {self.pos[2]}")

    def process_pokuicom(self, parent: SimNode, topic, payload):
        # types: SOCRE TEAM MATCHSTERTED
        payload = json.loads(payload)
        match topic:
            case "request":
                match payload["value"]:
                    case 0:
                        pass
                    case 1:
                        parent.send("team", f"{self.team}")
                    case 2:
                        parent.send("match", f"{1}")
            case "score":
                logger.info(f"New score: {payload["value"]}")

    def process_poklegscom(self, parent: SimNode, topic, raw_payload):
        # types: SOCRE TEAM MATCHSTERTED
        try:
            payload = json.loads(raw_payload)
        except Exception as e:
            logger.error(e)
            logger.error(raw_payload)
            return

        match topic:
            case "set_pos":
                self.pos = np.array([payload["x"], payload["y"], payload["a"]])
            case "set_waypoints":
                self.wp_index = 0
                self.wps = [np.array([x["x"], x["y"], x["a"]]) for x in payload]
                logger.info(f"set wps {payload}")
            case "set_break":
                self.motor_break = payload["state"]


class PokibotGameSimulator:
    def __init__(self):
        super().__init__()

        self.msms = MqttSimMessengerServer(self.process_message, self.device_disconnected)
        self.obstacles = [CircleObstacle([0, 1.5], 0.2)]
        self.robot_nodes : dict[str, PokibotSimNode]  = {}

    def run(self):
        def pg_fun():
            clock = pg.time.Clock()
            res = 1024
            ratio = 2/3
            screen = pg.display.set_mode((res, res*ratio), pg.RESIZABLE)

            raw_background = load_image("vinyle.png")
            raw_background_ratio = raw_background.get_width() / raw_background.get_height()
            # pg.display.set_caption("test")

            board = Board()
            # This is a simple event handler that enables player input.
            while True:
                info = pg.display.Info()
                screen.fill(pg.Color(0, 0, 0))
                background = pg.transform.scale(
                    raw_background, (info.current_h * raw_background_ratio, info.current_h)
                )
                board.dim_x = [0, background.get_width()]
                board.dim_y = [0, background.get_height()]
                screen.blit(background, (board.dim_x[0], board.dim_y[0]))

                for name, robot in self.robot_nodes.items():
                    draw_robot(screen, board, robot)

                for obstacle in self.obstacles:
                    draw_obstacle(screen, board, obstacle)

                for e in pg.event.get():
                    # quit upon screen exit
                    if e.type == pg.QUIT:
                        return

                clock.tick(60)
                pg.display.update()
                pg.time.delay(100)

        self.msms.start()
        pg.init()
        pg_fun()
        pg.quit()
        self.msms.stop()

    def device_disconnected(self, dev_name):
        self.robot_nodes[dev_name].stop_simulation()
        self.robot_nodes.pop(dev_name)

    def process_message(self, dev_name, topic_list, payload):
        if dev_name not in self.robot_nodes.keys():
            self.robot_nodes[dev_name] = PokibotSimNode(self.msms, dev_name)
            print(self.robot_nodes[dev_name])
        self.robot_nodes[dev_name].on_message(topic_list, payload)




if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    r = PokibotGameSimulator()
    r.run()
