"""MQTT-driven robot simulation parts.

Each *Sim class subscribes to a slice of the firmware's MQTT topic surface
and pushes the simulated robot state back. PokirobotSim composes them.
"""
import json
import logging
import threading
import time

import numpy as np
from msm import MqttSimMessengerServer, MqttSimMessengerNode
from shapely.geometry import Point
from shapely.ops import unary_union

from world import Robot, World, get_closest_collision_point

logger = logging.getLogger("strat_visu")


class SimProcess:
    def __init__(self, loop_clbk, frequency) -> None:
        self.sim_running = False
        self.loop_clbk = loop_clbk
        self.cycle_time = 1 / frequency

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
    def __init__(self, sim_process=[]) -> None:
        self.sim_process = sim_process

    def start(self):
        for process in self.sim_process:
            process.start()

    def stop(self):
        for process in self.sim_process:
            process.stop()


class PokuicomSim(SimPart):
    # Number of "PLUGGED" responses to send before auto-pulling the tirette.
    # Brain has two sequential polling loops (wait PLUGGED, then wait UNPLUGGED).
    # If we flip 2 -> 3 on the very first request, a slow RX schedule on the
    # brain side can collapse both responses and the first loop never sees "2".
    TIRETTE_PLUGGED_HOLD = 10

    def __init__(self, msm: MqttSimMessengerNode, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self._tirette_plugged_sends = 0
        self.pokuicom = MqttSimMessengerNodeWithClbk(msm, 0, "pokuicom", self.process_pokuicom)
        self.pokuicom.subscribe("request")
        self.pokuicom.subscribe("score")

    def process_pokuicom(self, parent: MqttSimMessengerNode, topic, payload):
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
                        if self.robot.tirette == 2:
                            self._tirette_plugged_sends += 1
                            if self._tirette_plugged_sends >= self.TIRETTE_PLUGGED_HOLD:
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
        self.speed = 400
        self.angle_speed = 3.14 / 2
        self.sensivity = 0.01

        self.poklegscom = MqttSimMessengerNodeWithClbk(msm, 0, "poklegscom", self.process_poklegscom)
        self.poklegscom.subscribe("set_pos")
        self.poklegscom.subscribe("set_waypoints")
        self.poklegscom.subscribe("set_break")

    def process_poklegscom(self, parent: MqttSimMessengerNode, topic, raw_payload):
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
        sensivity = np.array([self.sensivity, self.sensivity, self.sensivity])

        nb_wps = len(self.wps)

        if self.wp_index < nb_wps and not self.motor_break:
            target_pos = self.wps[self.wp_index]

            delta = target_pos - self.robot.pos
            delta_xy = delta[:2]
            delta_angle = delta[2]

            dist_xy = np.linalg.norm(delta_xy)

            if dist_xy > 1e-6:
                max_step_xy = self.speed * dt
                step_xy = delta_xy / dist_xy * min(dist_xy, max_step_xy)
            else:
                step_xy = np.zeros(2)

            step_angle = np.clip(
                delta_angle,
                -self.angle_speed * dt,
                self.angle_speed * dt,
            )

            self.robot.pos += np.array([step_xy[0], step_xy[1], step_angle])

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
            angles = [np.atan2(diff[1], diff[0]) for diff in diffs]
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
        obstacle_shape = unary_union([o.get_shape() for _, o in self.world.obstacles.items()])
        to_send_point_list = []
        for i in range(self.resolution):
            lidar_angle = i / self.resolution * 2 * np.pi
            table_angle = lidar_angle + self.robot.pos[2]
            dir = [np.cos(table_angle), np.sin(table_angle)]
            robot_pos = Point(self.robot.pos[0:2])
            shapely_point = get_closest_collision_point(obstacle_shape, robot_pos, Point(dir))
            if shapely_point:
                debug_point_list.append([shapely_point.x, shapely_point.y])
                to_send_point_list.append({
                    "angle": round(lidar_angle, quant),
                    "distance": round(robot_pos.distance(shapely_point), quant),
                    "intensity": 255,
                })
        self.robot.lidar_points = debug_point_list
        self.msm.send("lidar_points", json.dumps(to_send_point_list))


class PokirobotSim(SimPart):
    def __init__(self, sim_process) -> None:
        super().__init__(sim_process)
        self.start()


def pokirobot_builder(id: str, msms: MqttSimMessengerServer, world: World, team=0) -> tuple[Robot, PokirobotSim]:
    robot = Robot(team=team)
    pokirobot_msm = MqttSimMessengerNode(msms, id, "pokirobot")
    return robot, PokirobotSim([
        PokuicomSim(pokirobot_msm, robot),
        PoklegscomSim(pokirobot_msm, robot),
        LidarSim(pokirobot_msm, robot, world, 0),
    ])
