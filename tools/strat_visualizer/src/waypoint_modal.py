"""Floating modals for editing a waypoint's queued actions and configuring
action parameters. WaypointModal is the top-level panel; ActionConfigModal is
the per-action sub-panel for picking enum args (e.g. recalibration direction)."""
import pygame as pg

from actions import (
    action_default_args, action_has_params, action_ids, action_label,
    action_param_specs, format_action,
)
from constants import SIDEBAR_ACCENT, SIDEBAR_DIM, SIDEBAR_FG

_PANEL_W = 280
_ROW_H = 24
_PAD = 10
_HEADER_H = 28
_SECTION_H = 20
_BORDER_RADIUS = 6
_REMOVE_BTN_W = 28


def _make_font():
    try:
        return pg.font.SysFont("monospace", 14)
    except Exception:
        return pg.font.Font(None, 16)


class ActionConfigModal:
    """Sub-modal: pick enum values for an action's params, then confirm."""

    def __init__(self, aid, initial_args, anchor_screen_pos, on_confirm, on_cancel):
        self.aid = aid
        self.args = dict(action_default_args(aid))
        self.args.update(initial_args or {})
        self.anchor = anchor_screen_pos
        self.on_confirm = on_confirm
        self.on_cancel = on_cancel
        self._font = _make_font()
        self._rect = pg.Rect(0, 0, _PANEL_W, 0)
        self._row_rects = []

    def _compute_height(self):
        specs = action_param_specs(self.aid)
        h = _PAD + _HEADER_H
        for _name, choices in specs:
            h += _SECTION_H + len(choices) * _ROW_H
        h += _ROW_H + _PAD  # confirm
        return h

    def _compute_pos(self, screen_size):
        sw, sh = screen_size
        ax, ay = self.anchor
        h = self._compute_height()
        x = max(4, min(ax + 16, sw - _PANEL_W - 4))
        y = max(4, min(ay + 16, sh - h - 4))
        return x, y

    def draw(self, screen):
        h = self._compute_height()
        x, y = self._compute_pos(screen.get_size())
        self._rect = pg.Rect(x, y, _PANEL_W, h)
        bg = pg.Surface((_PANEL_W, h), pg.SRCALPHA)
        bg.fill((20, 20, 24, 245))
        screen.blit(bg, (x, y))
        pg.draw.rect(screen, SIDEBAR_ACCENT, self._rect, width=1, border_radius=_BORDER_RADIUS)

        font = self._font
        self._row_rects = []
        cy = y + _PAD
        header = font.render(action_label(self.aid), True, SIDEBAR_ACCENT)
        screen.blit(header, (x + _PAD, cy + (_HEADER_H - header.get_height()) // 2))
        cy += _HEADER_H

        for name, choices in action_param_specs(self.aid):
            sec = font.render(name, True, SIDEBAR_DIM)
            screen.blit(sec, (x + _PAD, cy + (_SECTION_H - sec.get_height()) // 2))
            cy += _SECTION_H
            for choice in choices:
                rr = pg.Rect(x, cy, _PANEL_W, _ROW_H)
                selected = self.args.get(name) == choice
                if selected:
                    hl = pg.Surface((_PANEL_W, _ROW_H), pg.SRCALPHA)
                    hl.fill((*SIDEBAR_ACCENT, 60))
                    screen.blit(hl, (x, cy))
                glyph = "●" if selected else "○"
                lsurf = font.render(f" {glyph}  {choice}", True, SIDEBAR_FG)
                screen.blit(lsurf, (x + _PAD, cy + (_ROW_H - lsurf.get_height()) // 2))
                self._row_rects.append(("choice", (name, choice), rr))
                cy += _ROW_H

        rr = pg.Rect(x, cy, _PANEL_W, _ROW_H)
        confirm = font.render(" confirm", True, SIDEBAR_ACCENT)
        screen.blit(confirm, (x + _PAD, cy + (_ROW_H - confirm.get_height()) // 2))
        self._row_rects.append(("confirm", None, rr))

    def handle_event(self, event) -> bool:
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                self.on_cancel()
                return True
            if event.key in (pg.K_RETURN, pg.K_KP_ENTER):
                self.on_confirm(dict(self.args))
                return True
            return False
        if event.type == pg.MOUSEBUTTONDOWN:
            if not self._rect.collidepoint(event.pos):
                self.on_cancel()
                return True
            if event.button == 1:
                for kind, data, rr in self._row_rects:
                    if rr.collidepoint(event.pos):
                        if kind == "choice":
                            name, choice = data
                            self.args[name] = choice
                        elif kind == "confirm":
                            self.on_confirm(dict(self.args))
            return True
        if event.type in (pg.MOUSEMOTION, pg.MOUSEBUTTONUP):
            return self._rect.collidepoint(event.pos)
        return False


class WaypointModal:
    def __init__(self, planner, wp_index, anchor_screen_pos, on_close,
                 on_insert_before=None, on_insert_after=None):
        self.planner = planner
        self.wp_index = wp_index
        self.anchor = anchor_screen_pos
        self.on_close = on_close
        self.on_insert_before = on_insert_before or (lambda idx: None)
        self.on_insert_after = on_insert_after or (lambda idx: None)
        self._font = _make_font()

        queued = self.planner.get_actions(self.wp_index)
        # cursor sits on the first "Add action" row by default: skip label + queued/(none).
        self._cursor = 1 + max(1, len(queued))
        self._rect = pg.Rect(0, 0, _PANEL_W, 0)
        self._row_rects = []
        self._sub_modal = None
        self._editing_label = False
        self._label_buffer = ""

    def _queued(self):
        return self.planner.get_actions(self.wp_index)

    def _total_rows(self):
        queued = self._queued()
        n_queued = max(1, len(queued))
        return 1 + n_queued + len(action_ids()) + 3

    def _clamp_cursor(self):
        total = self._total_rows()
        self._cursor = max(0, min(self._cursor, total - 1))

    def _compute_height(self):
        queued = self._queued()
        n_queued = max(1, len(queued))
        h = _PAD + _HEADER_H + _ROW_H  # label row
        h += _SECTION_H + n_queued * _ROW_H
        h += _SECTION_H + len(action_ids()) * _ROW_H
        h += _SECTION_H + 3 * _ROW_H + _PAD
        return h

    def _compute_pos(self, screen_size):
        sw, sh = screen_size
        ax, ay = self.anchor
        h = self._compute_height()
        x = max(4, min(ax + 16, sw - _PANEL_W - 4))
        y = max(4, min(ay + 16, sh - h - 4))
        return x, y

    def draw(self, screen):
        h = self._compute_height()
        x, y = self._compute_pos(screen.get_size())
        self._rect = pg.Rect(x, y, _PANEL_W, h)
        bg = pg.Surface((_PANEL_W, h), pg.SRCALPHA)
        bg.fill((20, 20, 24, 235))
        screen.blit(bg, (x, y))
        pg.draw.rect(screen, SIDEBAR_DIM, self._rect, width=1, border_radius=_BORDER_RADIUS)

        font = self._font
        self._row_rects = []
        cursor_row = 0
        cy = y + _PAD

        header = font.render(f"Waypoint #{self.wp_index + 1}", True, SIDEBAR_ACCENT)
        screen.blit(header, (x + _PAD, cy + (_HEADER_H - header.get_height()) // 2))
        cy += _HEADER_H

        label_rect = pg.Rect(x, cy, _PANEL_W, _ROW_H)
        if self._cursor == cursor_row:
            hl = pg.Surface((_PANEL_W, _ROW_H), pg.SRCALPHA)
            hl.fill((*SIDEBAR_ACCENT, 50))
            screen.blit(hl, (x, cy))
        prefix = font.render("label:", True, SIDEBAR_DIM)
        screen.blit(prefix, (x + _PAD, cy + (_ROW_H - prefix.get_height()) // 2))
        text_x = x + _PAD + prefix.get_width() + 6
        if self._editing_label:
            shown = self._label_buffer + "_"
            text_color = SIDEBAR_ACCENT
        else:
            current = self.planner.get_label(self.wp_index)
            shown = current if current else "(click to edit)"
            text_color = SIDEBAR_FG if current else SIDEBAR_DIM
        text_surf = font.render(shown, True, text_color)
        screen.blit(text_surf, (text_x, cy + (_ROW_H - text_surf.get_height()) // 2))
        self._row_rects.append(("label", None, label_rect, None))
        cursor_row += 1
        cy += _ROW_H

        sec = font.render("Queued actions", True, SIDEBAR_DIM)
        screen.blit(sec, (x + _PAD, cy + (_SECTION_H - sec.get_height()) // 2))
        cy += _SECTION_H

        queued = self._queued()
        if queued:
            for i, (aid, args) in enumerate(queued):
                row_rect = pg.Rect(x, cy, _PANEL_W, _ROW_H)
                rm_rect = pg.Rect(x + _PANEL_W - _REMOVE_BTN_W, cy, _REMOVE_BTN_W, _ROW_H)
                if self._cursor == cursor_row:
                    hl = pg.Surface((_PANEL_W, _ROW_H), pg.SRCALPHA)
                    hl.fill((*SIDEBAR_ACCENT, 50))
                    screen.blit(hl, (x, cy))
                label_surf = font.render(format_action(aid, args), True, SIDEBAR_FG)
                screen.blit(label_surf, (x + _PAD, cy + (_ROW_H - label_surf.get_height()) // 2))
                x_btn = font.render("[×]", True, SIDEBAR_DIM)
                screen.blit(x_btn, (rm_rect.right - _PAD - x_btn.get_width(),
                                    cy + (_ROW_H - x_btn.get_height()) // 2))
                self._row_rects.append(("queued", i, row_rect, rm_rect))
                cursor_row += 1
                cy += _ROW_H
        else:
            none_surf = font.render("(none)", True, SIDEBAR_DIM)
            screen.blit(none_surf, (x + _PAD, cy + (_ROW_H - none_surf.get_height()) // 2))
            self._row_rects.append(("queued_empty", -1, pg.Rect(x, cy, _PANEL_W, _ROW_H), None))
            cursor_row += 1
            cy += _ROW_H

        sec2 = font.render("Add action", True, SIDEBAR_DIM)
        screen.blit(sec2, (x + _PAD, cy + (_SECTION_H - sec2.get_height()) // 2))
        cy += _SECTION_H
        for aid in action_ids():
            rr = pg.Rect(x, cy, _PANEL_W, _ROW_H)
            if self._cursor == cursor_row:
                hl = pg.Surface((_PANEL_W, _ROW_H), pg.SRCALPHA)
                hl.fill((*SIDEBAR_ACCENT, 50))
                screen.blit(hl, (x, cy))
            suffix = " …" if action_has_params(aid) else ""
            lsurf = font.render(action_label(aid) + suffix, True, SIDEBAR_FG)
            screen.blit(lsurf, (x + _PAD, cy + (_ROW_H - lsurf.get_height()) // 2))
            self._row_rects.append(("add", aid, rr, None))
            cursor_row += 1
            cy += _ROW_H

        sec3 = font.render("Waypoint", True, SIDEBAR_DIM)
        screen.blit(sec3, (x + _PAD, cy + (_SECTION_H - sec3.get_height()) // 2))
        cy += _SECTION_H
        wp_actions = [
            ("wp_insert_after", "insert waypoint after"),
            ("wp_insert_before", "insert waypoint before"),
            ("wp_delete", "delete waypoint"),
        ]
        for kind, text in wp_actions:
            rr = pg.Rect(x, cy, _PANEL_W, _ROW_H)
            if self._cursor == cursor_row:
                hl = pg.Surface((_PANEL_W, _ROW_H), pg.SRCALPHA)
                hl.fill((*SIDEBAR_ACCENT, 50))
                screen.blit(hl, (x, cy))
            color = SIDEBAR_FG if kind != "wp_delete" else (0xE3, 0x65, 0x5B)
            lsurf = font.render(text, True, color)
            screen.blit(lsurf, (x + _PAD, cy + (_ROW_H - lsurf.get_height()) // 2))
            self._row_rects.append((kind, None, rr, None))
            cursor_row += 1
            cy += _ROW_H

        if self._sub_modal is not None:
            self._sub_modal.draw(screen)

    def _open_config_for_queued(self, qi):
        queued = self._queued()
        if qi >= len(queued):
            return
        aid, args = queued[qi]
        if not action_has_params(aid):
            return

        def on_confirm(new_args):
            acts = self.planner.get_actions(self.wp_index)
            if 0 <= qi < len(acts):
                old_aid, _ = acts[qi]
                acts[qi] = (old_aid, dict(new_args))
                self.planner.set_actions(self.wp_index, acts)
            self._sub_modal = None

        def on_cancel():
            self._sub_modal = None

        self._sub_modal = ActionConfigModal(aid, args, self.anchor, on_confirm, on_cancel)

    def _open_config_for_add(self, aid):
        def on_confirm(new_args):
            acts = self.planner.get_actions(self.wp_index)
            acts.append((aid, dict(new_args)))
            self.planner.set_actions(self.wp_index, acts)
            self._sub_modal = None

        def on_cancel():
            self._sub_modal = None

        self._sub_modal = ActionConfigModal(aid, {}, self.anchor, on_confirm, on_cancel)

    def _remove_queued(self, qi):
        acts = self.planner.get_actions(self.wp_index)
        if 0 <= qi < len(acts):
            acts.pop(qi)
            self.planner.set_actions(self.wp_index, acts)
            self._clamp_cursor()

    def _move_queued(self, qi, delta):
        acts = self.planner.get_actions(self.wp_index)
        target = qi + delta
        if not (0 <= qi < len(acts)) or not (0 <= target < len(acts)):
            return False
        acts[qi], acts[target] = acts[target], acts[qi]
        self.planner.set_actions(self.wp_index, acts)
        self._cursor = target + 1  # +1 for the label row at cursor 0
        return True

    def _cursor_queued_index(self):
        """If cursor is on a queued action row, return its index in the actions
        list; otherwise return None. Label row sits at cursor 0."""
        if self._cursor < 1:
            return None
        queued = self._queued()
        qi = self._cursor - 1
        return qi if 0 <= qi < len(queued) else None

    def _append_default(self, aid):
        acts = self.planner.get_actions(self.wp_index)
        acts.append((aid, action_default_args(aid)))
        self.planner.set_actions(self.wp_index, acts)

    def _begin_label_edit(self):
        self._label_buffer = self.planner.get_label(self.wp_index)
        self._editing_label = True

    def _commit_label_edit(self):
        self.planner.set_label(self.wp_index, self._label_buffer)
        self._editing_label = False

    def _cancel_label_edit(self):
        self._editing_label = False

    def _activate_row(self, kind, data):
        if kind == "label":
            self._begin_label_edit()
            return
        if kind == "queued":
            self._open_config_for_queued(data)
        elif kind == "add":
            if action_has_params(data):
                self._open_config_for_add(data)
            else:
                self._append_default(data)
        elif kind == "wp_insert_after":
            self.on_insert_after(self.wp_index)
            self.close()
        elif kind == "wp_insert_before":
            self.on_insert_before(self.wp_index)
            self.close()
        elif kind == "wp_delete":
            self.planner.delete_index(self.wp_index)
            self.close()

    def _activate_cursor_row(self):
        idx = 0
        for kind, data, _row, _rm in self._row_rects:
            if idx == self._cursor:
                if kind == "queued_empty":
                    return
                self._activate_row(kind, data)
                return
            idx += 1

    def handle_event(self, event) -> bool:
        if self._sub_modal is not None:
            return self._sub_modal.handle_event(event)

        if self._editing_label:
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE:
                    self._cancel_label_edit()
                    return True
                if event.key in (pg.K_RETURN, pg.K_KP_ENTER):
                    self._commit_label_edit()
                    return True
                if event.key == pg.K_BACKSPACE:
                    self._label_buffer = self._label_buffer[:-1]
                    return True
                if event.unicode and event.unicode.isprintable():
                    self._label_buffer += event.unicode
                    return True
                return True
            if event.type == pg.MOUSEBUTTONDOWN:
                if self._rect.collidepoint(event.pos):
                    self._commit_label_edit()
                    return True
                self._commit_label_edit()
                self.close()
                return True
            if event.type in (pg.MOUSEMOTION, pg.MOUSEBUTTONUP):
                return self._rect.collidepoint(event.pos)
            return False

        if event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                self.close()
                return True
            if event.key == pg.K_UP:
                if event.mod & pg.KMOD_SHIFT:
                    qi = self._cursor_queued_index()
                    if qi is not None:
                        self._move_queued(qi, -1)
                    return True
                self._cursor = max(0, self._cursor - 1)
                return True
            if event.key == pg.K_DOWN:
                if event.mod & pg.KMOD_SHIFT:
                    qi = self._cursor_queued_index()
                    if qi is not None:
                        self._move_queued(qi, +1)
                    return True
                self._cursor = min(self._total_rows() - 1, self._cursor + 1)
                return True
            if event.key in (pg.K_DELETE, pg.K_BACKSPACE):
                qi = self._cursor_queued_index()
                if qi is not None:
                    self._remove_queued(qi)
                return True
            if event.key in (pg.K_RETURN, pg.K_KP_ENTER):
                self._activate_cursor_row()
                return True
            return False

        if event.type == pg.MOUSEBUTTONDOWN:
            if not self._rect.collidepoint(event.pos):
                self.close()
                return True
            if event.button == 1:
                for i, (kind, data, row_rect, rm_rect) in enumerate(self._row_rects):
                    if rm_rect is not None and rm_rect.collidepoint(event.pos):
                        self._cursor = i
                        self._remove_queued(data)
                        return True
                    if row_rect.collidepoint(event.pos):
                        self._cursor = i
                        self._activate_row(kind, data)
                        return True
            return True

        if event.type in (pg.MOUSEMOTION, pg.MOUSEBUTTONUP):
            return self._rect.collidepoint(event.pos)
        return False

    def close(self):
        self.on_close()
