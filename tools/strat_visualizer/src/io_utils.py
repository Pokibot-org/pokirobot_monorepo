"""Filesystem + clipboard helpers used by the visualizer."""
import json
import logging
import os
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


_LEGACY_BUCKET = "eurobot_2025"
_LEGACY_FILE = "state.json"


def _per_competition_path(base_dir: str, competition_id: str) -> str:
    return os.path.join(base_dir, f"state.{competition_id}.json")


def _load_raw(path: str):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as exc:
        logger.warning(f"state load failed: {exc}")
        return None


def _slot_from_legacy(legacy_blob, competition_id: str):
    """Pull `competition_id`'s data out of an old shared `state.json`. Handles
    both the multi-bucket format (`{competitions: {id: slot}}`) and the very
    old flat format (`{side, waypoints, labels}` — only ever 2025)."""
    if not isinstance(legacy_blob, dict):
        return None
    if "competitions" in legacy_blob:
        return legacy_blob.get("competitions", {}).get(competition_id)
    return legacy_blob if competition_id == _LEGACY_BUCKET else None


def state_load(base_dir: str, competition_id: str):
    """Return the saved slot for `competition_id`, or None. State now lives
    in one file per competition (`state.<id>.json`) under `base_dir`. If the
    per-competition file is missing but a legacy shared `state.json` exists,
    pull the matching slot from there (one-time migration on first save)."""
    per_path = _per_competition_path(base_dir, competition_id)
    blob = _load_raw(per_path)
    if blob is not None:
        return blob
    legacy = _load_raw(os.path.join(base_dir, _LEGACY_FILE))
    return _slot_from_legacy(legacy, competition_id)


def state_save(base_dir: str, competition_id: str, side: str, waypoints, labels=None):
    labels = labels or []
    slot = {
        "side": side,
        "waypoints": [
            [x, y, a, [[aid, dict(args)] for (aid, args) in acts]]
            for (x, y, a, acts) in waypoints
        ],
        "labels": [labels[i] if i < len(labels) else "" for i in range(len(waypoints))],
    }
    per_path = _per_competition_path(base_dir, competition_id)
    try:
        with open(per_path, "w") as f:
            json.dump(slot, f, indent=2)
    except Exception as exc:
        logger.warning(f"state save failed: {exc}")
