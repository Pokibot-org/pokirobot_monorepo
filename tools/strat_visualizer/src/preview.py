"""PreviewScreen — full-window overlay that shows the generated C snippet
and offers a Copy / Back button. Triggered by the Export button (or `c` key)."""
import pygame as pg

from constants import (
    COLOR_DEEPBLACK, COLOR_GRAY, SIDEBAR_ACCENT, SIDEBAR_BG, SIDEBAR_DIM, SIDEBAR_FG,
)
from io_utils import copy_to_clipboard


class PreviewScreen:
    def __init__(self, code_text: str, on_back):
        self.code = code_text
        self.on_back = on_back
        self.status = ""
        self.scroll = 0
        self._buttons = []  # (rect, callback)

    def _do_copy(self):
        backend = copy_to_clipboard(self.code)
        self.status = f"copied ({backend})" if backend else "clipboard failed"

    def draw(self, screen):
        sw, sh = screen.get_size()
        screen.fill(SIDEBAR_BG)
        title_font = pg.font.SysFont("monospace", 22, bold=True)
        ui_font = pg.font.SysFont("monospace", 14, bold=True)
        code_font = pg.font.SysFont("monospace", 13)

        margin = 16
        btn_h = 32

        self._buttons = []

        back_rect = pg.Rect(margin, margin, 110, btn_h)
        pg.draw.rect(screen, COLOR_GRAY, back_rect, border_radius=6)
        pg.draw.rect(screen, SIDEBAR_FG, back_rect, width=1, border_radius=6)
        t = ui_font.render("← Back", True, SIDEBAR_FG)
        screen.blit(t, (back_rect.centerx - t.get_width() // 2, back_rect.centery - t.get_height() // 2))
        self._buttons.append((back_rect, self.on_back))

        title = title_font.render("Generated code", True, SIDEBAR_ACCENT)
        screen.blit(title, (back_rect.right + margin, margin + 3))

        copy_rect = pg.Rect(sw - margin - 110, margin, 110, btn_h)
        pg.draw.rect(screen, SIDEBAR_ACCENT, copy_rect, border_radius=6)
        pg.draw.rect(screen, COLOR_DEEPBLACK, copy_rect, width=1, border_radius=6)
        t = ui_font.render("Copy", True, COLOR_DEEPBLACK)
        screen.blit(t, (copy_rect.centerx - t.get_width() // 2, copy_rect.centery - t.get_height() // 2))
        self._buttons.append((copy_rect, self._do_copy))

        if self.status:
            s = ui_font.render(self.status, True, SIDEBAR_DIM)
            screen.blit(s, (sw - margin - s.get_width(), margin + btn_h + 6))

        code_top = margin + btn_h + 30
        code_area = pg.Rect(margin, code_top, sw - 2 * margin, sh - code_top - margin)
        pg.draw.rect(screen, (0x12, 0x12, 0x16), code_area, border_radius=6)
        pg.draw.rect(screen, SIDEBAR_DIM, code_area, width=1, border_radius=6)

        old_clip = screen.get_clip()
        screen.set_clip(code_area.inflate(-12, -12))
        line_h = code_font.get_linesize()
        y = code_area.y + 8 - self.scroll
        for line in self.code.splitlines() or [""]:
            if y + line_h >= code_area.y and y <= code_area.bottom:
                surf = code_font.render(line, True, SIDEBAR_FG)
                screen.blit(surf, (code_area.x + 10, y))
            y += line_h
        screen.set_clip(old_clip)

    def handle_event(self, event) -> bool:
        if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
            for rect, cb in self._buttons:
                if rect.collidepoint(event.pos):
                    cb()
                    return True
        if event.type == pg.MOUSEWHEEL:
            self.scroll = max(0, self.scroll - event.y * 30)
            return True
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_c:
                self._do_copy()
                self.on_back()
                return True
            if event.key in (pg.K_ESCAPE, pg.K_BACKSPACE):
                self.on_back()
                return True
        return False
