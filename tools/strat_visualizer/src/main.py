#!/usr/bin/env python
import os
import pygame as pg
import threading
import json
import math
from msm import MqttSimMessengerServer, SimNode
import time
import logging
logger = logging.getLogger("strat_visu")

main_dir = os.path.split(os.path.abspath(__file__))[0]


class Board:
    def __init__(self) -> None:
        self.real_size = [2000, 3000]
        self.dim_x = None
        self.dim_y = None


# quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, "../img", name)
    return pg.image.load(path).convert()


def draw_robot(screen, board, robot):
    ratio = board.dim_x[1] / board.real_size[1]
    on_board_radius = robot.radius * ratio
    on_board_pos = (robot.pos[0] * ratio, board.dim_y[1] - robot.pos[1] * ratio)

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


class SimNodeWithClbk(SimNode):
    def __init__(self, parent, id, name, clbk) -> None:
        super().__init__(parent, id, name)
        self.clbk = clbk

    def process_topic(self, topic, payload):
        self.clbk(self, topic, payload)

class PokibotSimNode(SimNode):
    def __init__(self, parent, id) -> None:
        super().__init__(parent, id, "")
        self.radius = 190
        self.pos = [0, 0, 0]
        self.team = 0
        self.wp_index = 0
        self.wps = []
        self.motor_break = False
        self.add_child(SimNodeWithClbk(self, 0, "pokuicom", self.process_pokuicom))
        self.poklegscom = SimNodeWithClbk(self, 0, "poklegscom", self.process_poklegscom)
        self.add_child(self.poklegscom)

        self.start_simulation()
        print(self.childs)

    def _sim_task(self):
        while 1:
            time.sleep(0.1)
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

    def process_poklegscom(self, parent: SimNode, topic, payload):
        # types: SOCRE TEAM MATCHSTERTED
        payload = json.loads(payload)
        match topic:
            case "set_pos":
                self.pos = [payload["x"], payload["y"], payload["a"]]
            case "set_waypoints":
                logger.info(f"set wps {payload}")
            case "set_break":
                self.motor_break = payload["state"]


class RobotMSM(MqttSimMessengerServer):
    def __init__(self):
        super().__init__()
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
                # Get all keys currently pressed, and move when an arrow key is held.
                # keys = pg.key.get_pressed()
                # if keys[pg.ESC]:
                #     return

                for e in pg.event.get():
                    # quit upon screen exit
                    if e.type == pg.QUIT:
                        return

                clock.tick(60)
                pg.display.update()
                pg.time.delay(100)
        self.start()
        pg.init()
        pg_fun()
        pg.quit()
        self.stop()

    def on_connect(self):
        pass

    def device_disconnected(self, dev_name):
        pass

    def process_message(self, dev_name, topic_list, payload):
        if dev_name not in self.robot_nodes.keys():
            self.robot_nodes[dev_name] = PokibotSimNode(self, dev_name)
            print(self.robot_nodes[dev_name])
        self.robot_nodes[dev_name].on_message(topic_list, payload)




if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    r = RobotMSM()
    r.run()
