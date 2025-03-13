#!/usr/bin/env python
import os
import pygame as pg
import threading
import json
import math
from msm import MqttSimMessengerServer, MqttSimMessengerNode
import time
import logging
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from dataclasses import dataclass

logger = logging.getLogger("strat_visu")

main_dir = os.path.split(os.path.abspath(__file__))[0]

COLOR_DEEPBLACK = (0, 0, 0)
COLOR_RED = (0xE3, 0x65, 0x5B)
COLOR_YELLOW = (0xCF, 0xD1, 0x86)
COLOR_GREEN = (0x5B, 0x8C, 0x5A)
COLOR_GRAY = (0x59, 0x61, 0x57)
COLOR_PURPLE = (0x52, 0x41, 0x4C)

COLOR_TEAM_BLUE = (0x00, 0x5B, 0x8C)
COLOR_TEAM_YELLOW = (0xF7, 0xB5, 0x00)

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
        self.dim_x = [0, 1]
        self.dim_y = [0, 1]

class Robot:
    def __init__(self, radius=0.19, team=0) -> None:
        self.radius = radius
        self.pos = np.array([0, 0, 0])
        self.team = team


# quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, "../img", name)
    return pg.image.load(path).convert()

def draw_robot(screen, board: Board, robot: Robot):
    ratio = board.dim_x[1] / board.real_size[0]
    on_board_radius = robot.radius * ratio
    on_board_pos = ((robot.pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (robot.pos[1] - board.mins[1]) * ratio)

    color = COLOR_TEAM_YELLOW if robot.team else COLOR_TEAM_BLUE
    pg.draw.circle(screen, (0,0,0), on_board_pos, on_board_radius)
    pg.draw.circle(screen, color, on_board_pos, on_board_radius * 0.9)
    pg.draw.line(
        screen,
        COLOR_GRAY,
        on_board_pos,
        (
            on_board_pos[0] + on_board_radius * math.cos(robot.pos[2]),
            on_board_pos[1] + on_board_radius * math.sin(robot.pos[2] + math.pi),
        ),
        width=4,
    )

def draw_obstacle(screen, board: Board, obstacle: Obstacle):
    ratio = board.dim_x[1] / board.real_size[0]

    if type(obstacle) is CircleObstacle:
        on_board_radius = obstacle.radius * ratio
        on_board_pos = ((obstacle.pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (obstacle.pos[1] - board.mins[1]) * ratio)
        pg.draw.circle(screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
        pg.draw.circle(screen, COLOR_GREEN, on_board_pos, on_board_radius*0.98)

def draw_debug_point(screen, board: Board, pos, radius=0.01, color=COLOR_RED):
    ratio = board.dim_x[1] / board.real_size[0]
    on_board_radius = radius * ratio
    on_board_pos = ((pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (pos[1] - board.mins[1]) * ratio)
    pg.draw.circle(screen, color, on_board_pos, on_board_radius)

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
                        parent.send("match", f"{1}")
            case "score":
                logger.info(f"New score: {payload["value"]}")

class PoklegscomSim(SimPart):
    def __init__(self, msm: MqttSimMessengerNode, robot: Robot) -> None:
        super().__init__([SimProcess(self.movement_sim, 60), SimProcess(self.pos_publish, 30)])
        self.robot = robot
        self.wp_index = 0
        self.wps = []
        self.motor_break = False
        self.speed = 0.5
        self.angle_speed = 3.14/2
        self.sensivity = 0.01

        self.poklegscom = MqttSimMessengerNodeWithClbk(msm, 0, "poklegscom", self.process_poklegscom)
        self.poklegscom.subscribe("set_pos")
        self.poklegscom.subscribe("set_waypoints")
        self.poklegscom.subscribe("set_break")

    def process_poklegscom(self, parent: MqttSimMessengerNode, topic, raw_payload):
        # types: SCORE TEAM MATCHSTERTED
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
                logger.info(f"set wps {payload}")
            case "set_break":
                self.motor_break = payload["state"]

    def movement_sim(self, dt):
        max_speed_vec = np.array([self.speed*dt, self.speed*dt, self.angle_speed*dt])
        sensivity = np.array([self.sensivity, self.sensivity, self.sensivity])
        if self.wp_index < len(self.wps) and not self.motor_break:
            target_pos = self.wps[self.wp_index]
            self.robot.pos += np.maximum(np.minimum(target_pos - self.robot.pos, max_speed_vec), -max_speed_vec)
            if np.all(np.abs(target_pos - self.robot.pos) < sensivity):
                self.wp_index += 1

    def pos_publish(self, dt):
        self.poklegscom.send("pos", f"{self.robot.pos[0]} {self.robot.pos[1]} {self.robot.pos[2]}")

class PokirobotSim(SimPart):
    def __init__(self, sim_process) -> None:
        super().__init__(sim_process)
        self.start()

@dataclass
class World:
    obstacles: list[Obstacle]

def pokirobot_builder(id: str, msms: MqttSimMessengerServer, world: World, team=0) -> tuple[Robot, PokirobotSim]:
    robot = Robot(team=team)
    pokirobot_msm = MqttSimMessengerNode(msms, id, 'pokirobot')
    return robot, PokirobotSim([PokuicomSim(pokirobot_msm, robot), PoklegscomSim(pokirobot_msm, robot)])

class PokibotGameSimulator:
    def __init__(self):
        super().__init__()

        self.msms = MqttSimMessengerServer(self.on_device_connection, self.on_device_disconnection)
        self.world = World(obstacles=[CircleObstacle([0, 1.5], 0.2)])
        self.robots : dict[str, Robot]  = {}
        self.pokirobot_sim_nodes : dict[str, PokirobotSim]  = {}

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

                for name, robot in self.robots.items():
                    draw_robot(screen, board, robot)

                for obstacle in self.world.obstacles:
                    draw_obstacle(screen, board, obstacle)

                for e in pg.event.get():
                    # quit upon screen exit
                    if e.type == pg.QUIT:
                        return

                pg.display.update()
                clock.tick(60)

        self.msms.start()
        pg.init()
        pg_fun()
        for id, robot in self.pokirobot_sim_nodes.items():
            robot.stop()
        pg.quit()
        self.msms.stop()

    def on_device_connection(self, dev_name, dev_id):
        if dev_id not in self.pokirobot_sim_nodes.keys():
            robot, pokirobot = pokirobot_builder(dev_id, self.msms, self.world)
            self.robots["pokirobot_" + dev_id] = robot
            self.pokirobot_sim_nodes[dev_id] = pokirobot

    def on_device_disconnection(self, dev_name, dev_id):
        if dev_id in self.pokirobot_sim_nodes.keys():
            self.pokirobot_sim_nodes[dev_id].stop()
            self.pokirobot_sim_nodes.pop(dev_id)
            self.robots.pop("pokirobot_" + dev_id)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    r = PokibotGameSimulator()
    r.run()
