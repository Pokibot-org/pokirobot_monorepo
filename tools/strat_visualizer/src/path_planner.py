"""PathPlanner — waypoint editing state + rendering.

Owns: the waypoint list, drag/snap state, hover state (for tooltip + overlap
cycling), and the per-frame drawing pipeline (segments, arrows, markers,
tooltip, alpha + gradient by path-order distance from the simulator).
"""
import math

import pygame as pg

from actions import format_action
from constants import COLOR_WP, COLOR_WP_HALO, COLOR_WP_PENDING, SIDEBAR_ACCENT, SIDEBAR_DIM, SIDEBAR_FG


class PathPlanner:
    HIT_RADIUS_MM = 80.0

    def __init__(self):
        self.waypoints = []  # list of (x, y, angle, [action_id, ...])
        self.labels = []     # parallel list of free-form per-waypoint notes
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
        # floating-placement state — a freshly-inserted wp follows the mouse
        # until the user left-clicks (commit) or right-clicks (cancel).
        self._placing_index = None
        # hover state — used for tooltip + cycling overlapped waypoints
        self._hover_world = None
        self._hover_indices = []  # all wps within HIT_RADIUS, sorted by distance
        self._hover_cycle = 0     # which one of _hover_indices is "active"

    @staticmethod
    def _wrap_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def mirror_y_axis(self):
        """Match convert_pos_for_team_no_angle: flip x, keep angle."""
        self.waypoints = [(-x, y, a, list(acts)) for (x, y, a, acts) in self.waypoints]

    def get_actions(self, idx):
        if 0 <= idx < len(self.waypoints):
            return list(self.waypoints[idx][3])
        return []

    def set_actions(self, idx, actions):
        if 0 <= idx < len(self.waypoints):
            x, y, a, _ = self.waypoints[idx]
            self.waypoints[idx] = (x, y, a, list(actions))

    def insert_waypoint(self, idx, x, y, angle, actions=None, label=""):
        self.waypoints.insert(idx, (x, y, angle, list(actions or [])))
        self.labels.insert(idx, label)

    def delete_index(self, idx):
        if 0 <= idx < len(self.waypoints):
            self.waypoints.pop(idx)
            if idx < len(self.labels):
                self.labels.pop(idx)
            if self._placing_index is not None:
                if idx == self._placing_index:
                    self._placing_index = None
                elif idx < self._placing_index:
                    self._placing_index -= 1

    def get_label(self, idx) -> str:
        if 0 <= idx < len(self.labels):
            return self.labels[idx]
        return ""

    def set_label(self, idx, text: str):
        while len(self.labels) < len(self.waypoints):
            self.labels.append("")
        if 0 <= idx < len(self.waypoints):
            self.labels[idx] = text

    def is_placing(self) -> bool:
        return self._placing_index is not None

    def start_placing(self, idx):
        if 0 <= idx < len(self.waypoints):
            self._placing_index = idx

    def update_placing(self, world_pos):
        if self._placing_index is None or world_pos is None:
            return
        x, y = self._snap_pos(world_pos)
        _, _, a, acts = self.waypoints[self._placing_index]
        self.waypoints[self._placing_index] = (x, y, a, acts)

    def commit_placing(self):
        self._placing_index = None

    def cancel_placing(self):
        if self._placing_index is None:
            return
        self.waypoints.pop(self._placing_index)
        if self._placing_index < len(self.labels):
            self.labels.pop(self._placing_index)
        self._placing_index = None

    def _snap_pos(self, pos):
        if not self.grid_enabled or self.grid_size <= 0:
            return pos
        gs = self.grid_size
        return (round(pos[0] / gs) * gs, round(pos[1] / gs) * gs)

    def _snap_angle(self, angle):
        if not self.angle_enabled or self.angle_step <= 0:
            return angle
        return round(angle / self.angle_step) * self.angle_step

    # ---- hover / hit testing ------------------------------------------------

    def find_indices_near(self, world_pos):
        if world_pos is None:
            return []
        wx, wy = world_pos
        r2 = self.HIT_RADIUS_MM ** 2
        hits = []
        for i, (x, y, _, _) in enumerate(self.waypoints):
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

    def is_dragging(self) -> bool:
        return self._mode is not None

    # ---- editing ------------------------------------------------------------

    def delete_at(self, world_pos) -> bool:
        # delete the currently-active hovered wp if any, otherwise nearest
        i = self._hovered_index() if self._hover_world == world_pos else None
        if i is None:
            i = self.find_index_near(world_pos)
        if i is None:
            return False
        self.waypoints.pop(i)
        if i < len(self.labels):
            self.labels.pop(i)
        self._hover_indices = []
        self._hover_cycle = 0
        return True

    def start_drag(self, world_pos, shift=False):
        if world_pos is None:
            return
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
            _, _, a, acts = self.waypoints[self._edit_index]
            self.waypoints[self._edit_index] = (x, y, a, acts)
        elif self._mode == "rotate":
            x, y, _, acts = self.waypoints[self._edit_index]
            dx, dy = world_pos[0] - x, world_pos[1] - y
            if dx * dx + dy * dy >= 1.0:
                a = self._snap_angle(math.atan2(dy, dx))
                self.waypoints[self._edit_index] = (x, y, a, acts)

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
            self.waypoints.append((sx, sy, angle, []))
            self.labels.append("")
        self._mode = None
        self._edit_index = None
        self._drag_start = None
        self._drag_current = None

    def undo(self):
        if self.waypoints:
            self.waypoints.pop()
            if self.labels:
                self.labels.pop()

    def clear(self):
        self.waypoints.clear()
        self.labels.clear()
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

    # ---- export -------------------------------------------------------------

    def to_c_string(self) -> str:
        """Self-contained one-liner per waypoint; action comments follow each line."""
        if not self.waypoints:
            return "// no waypoints\n"
        lines = []
        for i, (x, y, a, acts) in enumerate(self.waypoints, start=1):
            label = self.get_label(i - 1)
            if label:
                lines.append(f"// {label}")
            lines.append(
                f"nav_go_to_direct(convert_pos_for_team_no_angle(color, "
                f"(pos2_t){{.x = {x:.3f}f, .y = {y:.3f}f, .a = {a:.6f}f}}, 0.0f), K_FOREVER); "
                f"nav_wait_events(&nav_events); // step {i}"
            )
            for aid, args in acts:
                if args:
                    arg_str = " ".join(f"{k}={v}" for k, v in args.items())
                    lines.append(f"// action: {aid} {arg_str}")
                else:
                    lines.append(f"// action: {aid}")
        return "\n".join(lines) + "\n"

    def export(self, path):
        text = self.to_c_string()
        with open(path, "w") as f:
            f.write(text)
        return path

    # ---- drawing ------------------------------------------------------------

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
            pg.draw.line(surf, color, (p1[0] - board_left, p1[1] - board_top),
                         (p2[0] - board_left, p2[1] - board_top), 1)
            x += gs
        y = math.ceil(y_min / gs) * gs
        while y <= y_max:
            p1 = game_viz.get_on_board_pos((x_min, y))
            p2 = game_viz.get_on_board_pos((x_max, y))
            pg.draw.line(surf, color, (p1[0] - board_left, p1[1] - board_top),
                         (p2[0] - board_left, p2[1] - board_top), 1)
            y += gs
        screen.blit(surf, (board_left, board_top))

    def _draw_arrow(self, screen, start, end, color):
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
        pg.draw.line(screen, COLOR_WP_HALO, start, shaft_end, width=7)
        pg.draw.line(screen, color, start, shaft_end, width=3)
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
        pad = 6
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
        pad = 10
        x0 = int(min(start[0], end[0])) - pad
        y0 = int(min(start[1], end[1])) - pad
        x1 = int(max(start[0], end[0])) + pad
        y1 = int(max(start[1], end[1])) + pad
        w, h = max(1, x1 - x0), max(1, y1 - y0)
        overlay = pg.Surface((w, h), pg.SRCALPHA)
        self._draw_arrow(overlay, (start[0] - x0, start[1] - y0),
                         (end[0] - x0, end[1] - y0), color)
        overlay.set_alpha(alpha)
        screen.blit(overlay, (x0, y0))

    def _draw_marker(self, screen, sp, color, label_text, font, action_count=0):
        outer_r = 13
        inner_r = 9
        pg.draw.circle(screen, COLOR_WP_HALO, (sp[0] + 1, sp[1] + 2), outer_r + 2)
        pg.draw.circle(screen, COLOR_WP_HALO, sp, outer_r + 2)
        pg.draw.circle(screen, color, sp, outer_r)
        pg.draw.circle(screen, (245, 245, 245), sp, inner_r)
        if label_text:
            label = font.render(label_text, True, COLOR_WP_HALO)
            screen.blit(label, (sp[0] - label.get_width() // 2, sp[1] - label.get_height() // 2))
        if action_count > 0:
            badge_r = 7
            bx = sp[0] + outer_r - 1
            by = sp[1] - outer_r + 1
            pg.draw.circle(screen, SIDEBAR_ACCENT, (bx, by), badge_r)
            digit = font.render(str(action_count), True, (20, 20, 20))
            screen.blit(digit, (bx - digit.get_width() // 2, by - digit.get_height() // 2))

    def _draw_marker_alpha(self, screen, sp, color, label_text, font, alpha, action_count=0):
        if alpha >= 255:
            self._draw_marker(screen, sp, color, label_text, font, action_count)
            return
        outer_r = 13
        pad = outer_r + 10
        size = pad * 2
        overlay = pg.Surface((size, size), pg.SRCALPHA)
        self._draw_marker(overlay, (pad, pad), color, label_text, font, action_count)
        overlay.set_alpha(alpha)
        screen.blit(overlay, (sp[0] - pad, sp[1] - pad))

    def _draw_hover_tooltip(self, screen, font, idx, mouse_screen_pos):
        if idx >= len(self.waypoints):
            return
        x, y, a, acts = self.waypoints[idx]
        a_deg = math.degrees(a)
        label = self.get_label(idx)
        lines = []
        if label:
            lines.append(f"“{label}”")
        lines.append(f"#{idx + 1}  x={x:.0f}  y={y:.0f}")
        lines.append(f"a={a:.3f} rad ({a_deg:.1f}°)")
        if acts:
            if len(acts) <= 3:
                labels = ", ".join(format_action(aid, args) for (aid, args) in acts)
            else:
                labels = ", ".join(format_action(aid, args) for (aid, args) in acts[:2])
                labels += f", +{len(acts) - 2} more"
            lines.append(f"actions: {labels}")
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

    # Gradient stops from "current target" to "far / passed".
    _GRADIENT = [
        (0x00, 0xE5, 0xFF),  # 0 — radiant cyan (next target)
        (0x3A, 0x7B, 0xFF),  # 1 — bright blue
        (0x6C, 0x3A, 0xD0),  # 2 — violet
        (0x32, 0x16, 0x60),  # 3 — deep purple
        (0x12, 0x08, 0x24),  # 4 — near-black
    ]

    def _path_offset(self, idx, sim) -> float:
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

        sps = [game_viz.get_on_board_pos((x, y)) for (x, y, _, _) in self.waypoints]
        wp_alpha = [self._alpha_for(i, sim) for i in range(len(self.waypoints))]
        wp_color = [self._color_for(i, sim) for i in range(len(self.waypoints))]

        # Two-pass draw so opaque (important) items sit on top of faded ones.
        for opaque_pass in (False, True):
            if len(self.waypoints) >= 2:
                for i in range(len(sps) - 1):
                    alpha = max(wp_alpha[i], wp_alpha[i + 1])
                    if (alpha >= 255) != opaque_pass:
                        continue
                    color = wp_color[i] if wp_alpha[i] >= wp_alpha[i + 1] else wp_color[i + 1]
                    self._draw_segment_alpha(screen, sps[i], sps[i + 1], alpha, color)
            for i, (x, y, a, _acts) in enumerate(self.waypoints):
                alpha = wp_alpha[i]
                if (alpha >= 255) != opaque_pass:
                    continue
                sp = sps[i]
                tip = game_viz.get_on_board_pos((x + arrow_len_world * math.cos(a),
                                                 y + arrow_len_world * math.sin(a)))
                self._draw_arrow_alpha(screen, sp, tip, wp_color[i], alpha)
            for i, (x, y, a, acts) in enumerate(self.waypoints):
                alpha = wp_alpha[i]
                if (alpha >= 255) != opaque_pass:
                    continue
                color = COLOR_WP_PENDING if i == hovered else wp_color[i]
                self._draw_marker_alpha(screen, sps[i], color, str(i + 1), font, alpha,
                                        action_count=len(acts))

        pending = self.pending()
        if pending is not None:
            x, y, a = pending
            sp = game_viz.get_on_board_pos((x, y))
            tip = game_viz.get_on_board_pos((x + arrow_len_world * math.cos(a),
                                             y + arrow_len_world * math.sin(a)))
            self._draw_arrow(screen, sp, tip, COLOR_WP_PENDING)
            self._draw_marker(screen, sp, COLOR_WP_PENDING, "", font)

        if hovered is not None and mouse_screen_pos is not None:
            self._draw_hover_tooltip(screen, font, hovered, mouse_screen_pos)
