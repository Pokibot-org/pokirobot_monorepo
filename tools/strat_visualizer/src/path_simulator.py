"""PathSimulator — local preview that drives a Robot through waypoints with
obstacle avoidance. Used by the play/pause/reset buttons in PlanPanel."""
import math

import numpy as np

from world import Robot, World


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
        # Reset returns to the pre-start state so paths/waypoints render opaque.
        self.started = False
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
            step_dist = float(np.linalg.norm(step_xy))
            if step_dist > dist:
                step_xy = step_xy / step_dist * dist
        else:
            step_xy = np.zeros(2)

        step_a = max(-self.angle_speed * dt, min(self.angle_speed * dt, delta_a))
        self.robot.pos = self.robot.pos + np.array([step_xy[0], step_xy[1], step_a])
        # Holonomic robot: heading is independent of motion direction and rotates
        # progressively toward the waypoint angle via step_a above.
        self.robot.dir = float(self.robot.pos[2])
        if dist < self.pos_tol and abs(delta_a) < self.ang_tol:
            self.wp_index += 1
            self.robot.wps = self.wps[self.wp_index:]
            if self.wp_index >= len(self.wps):
                self.running = False
