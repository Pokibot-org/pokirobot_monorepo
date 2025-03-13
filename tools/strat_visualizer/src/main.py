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

def draw_obstacle(screen, board, obstacle: Obstacle):
    ratio = board.dim_x[1] / board.real_size[0]

    if type(obstacle) is CircleObstacle:
        on_board_radius = obstacle.radius * ratio
        on_board_pos = ((obstacle.pos[0] - board.mins[0]) * ratio, board.dim_y[1] - (obstacle.pos[1] - board.mins[1]) * ratio)
        pg.draw.circle(screen, COLOR_DEEPBLACK, on_board_pos, on_board_radius)
        pg.draw.circle(screen, COLOR_GREEN, on_board_pos, on_board_radius*0.98)

def draw_debug_point(screen, board, pos, radius=0.01, color=COLOR_RED):
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

class PokuicomSim:
    def __init__(self, msm: MqttSimMessengerNode) -> None:
        self.msm = msm

class SimNodeWithClbk(MqttSimMessengerNode):
    def __init__(self, parent, id, name, clbk) -> None:
        super().__init__(parent, id, name)
        self.clbk = clbk

    def process_topic(self, topic, payload):
        self.clbk(self, topic, payload)

class PokirobotSim(MqttSimMessengerNode):
    def __init__(self, parent, id, team) -> None:
        super().__init__(parent, id, "")
        self.msm = MqttSimMessengerNode(parent, id, "")
        self.radius = 0.19
        self.pos = np.array([0, 0, 0])
        self.team = team
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

        self.sim_process = [SimProcess(self.movement_sim, 60), SimProcess(self.pos_publish, 30)]
        self.start_simulation()

    def start_simulation(self):
        for process in self.sim_process:
            process.start()
        logger.info("start sim processes")

    def stop_simulation(self):
        for process in self.sim_process:
            process.stop()

    def movement_sim(self, dt):
        max_speed_vec = np.array([self.speed*dt, self.speed*dt, self.angle_speed*dt])
        sensivity = np.array([self.sensivity, self.sensivity, self.sensivity])
        if self.wp_index < len(self.wps) and not self.motor_break:
            target_pos = self.wps[self.wp_index]
            self.pos += np.maximum(np.minimum(target_pos - self.pos, max_speed_vec), -max_speed_vec)
            if np.all(np.abs(target_pos - self.pos) < sensivity):
                self.wp_index += 1

    def pos_publish(self, dt):
        self.poklegscom.send("pos", f"{self.pos[0]} {self.pos[1]} {self.pos[2]}")

    def process_pokuicom(self, parent: MqttSimMessengerNode, topic, payload):
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

    def process_poklegscom(self, parent: MqttSimMessengerNode, topic, raw_payload):
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

@dataclass
class World:
    obstacles: list[Obstacle]

def robot_builder(id: str, msms: MqttSimMessengerServer, world: World, team=0) -> PokirobotSim:
    return PokirobotSim(msms, id, team)

class PokibotGameSimulator:
    def __init__(self):
        super().__init__()

        self.msms = MqttSimMessengerServer(self.process_message, self.device_disconnected)
        self.world = World(obstacles=[CircleObstacle([0, 1.5], 0.2)])
        self.robot_nodes : dict[str, PokirobotSim]  = {}

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
        pg.quit()
        self.msms.stop()

    def device_disconnected(self, dev_name):
        self.robot_nodes[dev_name].stop_simulation()
        self.robot_nodes.pop(dev_name)

    def process_message(self, dev_name, topic_list, payload):
        if dev_name not in self.robot_nodes.keys():
            self.robot_nodes[dev_name] = robot_builder(dev_name, self.msms, self.world)
            print(self.robot_nodes[dev_name])
        self.robot_nodes[dev_name].on_message(topic_list, payload)




if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    r = PokibotGameSimulator()
    r.run()
