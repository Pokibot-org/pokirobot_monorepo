"""PathSimulator — local preview that drives a Robot through the planner's
waypoints with obstacle avoidance.

The simulator reads `self.planner.waypoints` live every step, so edits made
while the robot is following the path (moving a waypoint, deleting one,
inserting one) are picked up immediately. `wp_index` is the cursor — the
index of the next waypoint to reach."""
import math

import numpy as np

from world import Robot, World


class PathSimulator:
    PREVIEW_KEY = "sim_preview"

    def __init__(self, world: World, planner=None, on_user_running=None):
        self.world = world
        self.planner = planner
        self.robot = Robot(team=0)
        self.wp_index = 0
        self.started = False
        self.running = False
        self.speed = 500.0  # mm/s
        self.angle_speed = math.pi / 2  # rad/s
        self.pos_tol = 5.0  # mm
        self.ang_tol = 0.02  # rad
        self._on_user_paused = on_user_running or (lambda running: None)
        self.action_duration = 0.8
        self.current_action = None
        self._action_queue = []
        self._action_elapsed = 0.0
        self._step_mode = False

    # ---- planner reads -----------------------------------------------------

    def _wps(self):
        return self.planner.waypoints if self.planner is not None else []

    def _wp_pose(self, i):
        x, y, a, _acts = self._wps()[i]
        return np.array([x, y, a], dtype=float)

    def _wp_actions(self, i):
        return list(self._wps()[i][3])

    def _sync_team(self):
        if self.planner is not None:
            self.robot.team = 1 if self.planner.side == "yellow" else 0

    def _publish_remaining(self):
        wps = self._wps()
        self.robot.wps = [self._wp_pose(i) for i in range(self.wp_index, len(wps))]

    # ---- lifecycle ---------------------------------------------------------

    def _start_from(self, start_index: int) -> bool:
        wps = self._wps()
        if not wps:
            return False
        self._sync_team()
        start_index = max(0, min(start_index, len(wps) - 1))
        first = self._wp_pose(start_index)
        self.robot.pos = first.copy()
        self.robot.dir = float(first[2])
        self.wp_index = start_index + 1
        self._publish_remaining()
        self._action_queue = []
        self._action_elapsed = 0.0
        self.current_action = None
        self.world.robots[self.PREVIEW_KEY] = self.robot
        self.started = True
        return True

    def play_pause(self, waypoints=None):
        wps = self._wps()
        if not self.started or self.wp_index >= len(wps):
            if not self._start_from(0):
                return
            self.running = self.wp_index < len(wps)
        else:
            self.running = not self.running
        self._on_user_paused(self.running)

    def reset(self, waypoints=None):
        self._start_from(0)
        self.running = False
        self._step_mode = False
        # Reset returns to the pre-start state so paths/waypoints render opaque.
        self.started = False
        self._on_user_paused(False)

    def stop(self):
        self.running = False
        self._step_mode = False
        self.started = False
        self.world.robots.pop(self.PREVIEW_KEY, None)
        self._on_user_paused(False)

    def step_one(self, waypoints=None):
        """Run until the next waypoint's actions complete, then pause."""
        wps = self._wps()
        if not self.started or self.wp_index >= len(wps):
            if not self._start_from(0):
                return
        if self.wp_index >= len(wps):
            return
        self._step_mode = True
        self.running = True
        self._on_user_paused(True)

    def goto_index(self, i: int):
        """Snap the robot to waypoint `i`'s pose and pause. Used by the
        right-click 'go to this waypoint' option and by step_prev."""
        wps = self._wps()
        if not wps:
            return
        i = max(0, min(i, len(wps) - 1))
        self._start_from(i)
        self.running = False
        self._step_mode = False
        self._on_user_paused(False)

    def step_prev(self):
        """Snap to the previous waypoint. If the robot was heading from wp[i-1]
        to wp[i], this puts it back at wp[i-2] (or wp[0] at the start)."""
        wps = self._wps()
        if not wps:
            return
        # wp_index is the next target; the wp the robot most recently "arrived"
        # at is wp_index - 1. Going backward one step lands on wp_index - 2.
        prev = max(0, self.wp_index - 2)
        self.goto_index(prev)

    def step_next(self):
        """Snap to the next waypoint, skipping its actions. Symmetric to
        step_prev. The motion-based forward step is still available via
        `step_one`."""
        wps = self._wps()
        if not wps:
            return
        # current arrived = wp_index - 1; next = wp_index.
        nxt = min(len(wps) - 1, self.wp_index)
        self.goto_index(nxt)

    # ---- per-frame ---------------------------------------------------------

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

    def get_active_action_progress(self):
        if self.current_action is None:
            return None
        frac = min(1.0, self._action_elapsed / self.action_duration) if self.action_duration > 0 else 1.0
        return (self.current_action, frac)

    def step(self, dt):
        self._sync_team()
        wps = self._wps()
        # Clamp wp_index if the user deleted waypoints out from under us.
        if self.wp_index > len(wps):
            self.wp_index = len(wps)
        self._publish_remaining()

        if self.current_action is not None:
            self._action_elapsed += dt
            if self._action_elapsed >= self.action_duration:
                if self._action_queue:
                    aid, args = self._action_queue.pop(0)
                    self.current_action = self.planner.catalog.format(aid, args)
                    self._action_elapsed = 0.0
                else:
                    self.current_action = None
                    self._action_elapsed = 0.0
                    if self._step_mode:
                        self.running = False
                        self._step_mode = False
                        self._on_user_paused(False)
            return
        if not self.running or self.wp_index >= len(wps):
            self.running = False
            return
        target = self._wp_pose(self.wp_index)
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
            arrived_actions = self._wp_actions(self.wp_index)
            self.wp_index += 1
            self._publish_remaining()
            if arrived_actions:
                self._action_queue = arrived_actions[1:]
                aid, args = arrived_actions[0]
                self.current_action = self.planner.catalog.format(aid, args)
                self._action_elapsed = 0.0
            elif self._step_mode:
                self.running = False
                self._step_mode = False
                self._on_user_paused(False)
            if self.wp_index >= len(wps):
                self.running = False
                self._step_mode = False
