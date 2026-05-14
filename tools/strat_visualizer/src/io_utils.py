"""Filesystem + clipboard helpers used by the visualizer."""
import json
import logging
import shutil
import subprocess

import pygame as pg

logger = logging.getLogger("strat_visu")


def copy_to_clipboard(text: str) -> str:
    """Return the backend name that succeeded, or '' on failure."""
    for cmd in (["wl-copy"], ["xclip", "-selection", "clipboard"], ["xsel", "-bi"]):
        if shutil.which(cmd[0]):
            try:
                subprocess.run(cmd, input=text.encode("utf-8"), check=True, timeout=2)
                return cmd[0]
            except Exception as exc:
                logger.warning(f"{cmd[0]} failed: {exc}")
    try:
        if not pg.scrap.get_init():
            pg.scrap.init()
        pg.scrap.put(pg.SCRAP_TEXT, text.encode("utf-8"))
        return "pygame.scrap"
    except Exception as exc:
        logger.warning(f"pygame.scrap failed: {exc}")
    return ""


def state_load(path: str):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as exc:
        logger.warning(f"state load failed: {exc}")
        return None


def state_save(path: str, side: str, waypoints, labels=None):
    labels = labels or []
    try:
        with open(path, "w") as f:
            json.dump({
                "side": side,
                "waypoints": [
                    [x, y, a, [[aid, dict(args)] for (aid, args) in acts]]
                    for (x, y, a, acts) in waypoints
                ],
                "labels": [labels[i] if i < len(labels) else "" for i in range(len(waypoints))],
            }, f, indent=2)
    except Exception as exc:
        logger.warning(f"state save failed: {exc}")
