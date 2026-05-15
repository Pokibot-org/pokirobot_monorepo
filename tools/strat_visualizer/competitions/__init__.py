"""Competition configs. Each year lives in its own subpackage (e.g.
`eurobot_2026`) exposing a module-level `competition: Competition`. There is
no central registry — `load_competition` resolves a year or full id by
importing the matching submodule."""

import importlib
import os
import re

from .base import Action, ActionCatalog, Competition, EnumParam, IntParam, Param


_HERE = os.path.dirname(os.path.abspath(__file__))
_YEAR_RE = re.compile(r"^eurobot_(\d{4})$")


def list_competitions() -> list[str]:
    """Return existing competition ids (subfolder names matching `eurobot_<year>`),
    sorted descending by year so the latest is first."""
    out = []
    for name in os.listdir(_HERE):
        if not os.path.isdir(os.path.join(_HERE, name)):
            continue
        m = _YEAR_RE.match(name)
        if m:
            out.append((int(m.group(1)), name))
    out.sort(reverse=True)
    return [name for _, name in out]


def default_competition_id() -> str:
    """Pick the latest existing `eurobot_<year>` folder. Used by the CLI as
    the default when --competition is not given. Falls back to a sentinel
    that load_competition will reject loudly if no folders exist."""
    ids = list_competitions()
    return ids[0] if ids else "eurobot_unknown"


def load_competition(id_or_year) -> Competition:
    """Accept `2026`, `"2026"`, or `"eurobot_2026"` and return the
    `competition` attribute of the matching subpackage. Raises
    `ModuleNotFoundError` with the attempted name if the folder is missing."""
    name = str(id_or_year)
    if name.isdigit():
        name = f"eurobot_{name}"
    mod = importlib.import_module(f"competitions.{name}")
    return mod.competition
