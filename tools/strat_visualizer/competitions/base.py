"""Core types for the competition system.

`Param` describes one action argument; today only `EnumParam` exists but the
shape is open for ints/floats. `Action` is one item in a competition's action
menu — it knows its UI label, parameter list, how to format itself for display,
and how to emit a firmware call to the C exporter. `ActionCatalog` wraps the
per-competition list, handles legacy aliasing, and normalizes raw entries
loaded from state.json.

`Competition` is the top-level config: board geometry, robot art + radius,
per-side start positions, and the action catalog. Each year lives in its own
subpackage under `competitions/` exposing a module-level `competition`
instance built from absolute asset paths (resolved against the year folder)."""

from dataclasses import dataclass, field


class Param:
    """Base class. Subclasses define `default()` and `validate(v)`. `coerce`
    returns the canonical in-memory form of an already-valid value (used to
    keep JSON-loaded values type-stable — e.g. an IntParam round-trip stays
    an int, not a float)."""

    name: str

    def default(self):
        raise NotImplementedError

    def validate(self, v) -> bool:
        raise NotImplementedError

    def coerce(self, v):
        return v


class EnumParam(Param):
    def __init__(self, name, choices):
        self.name = name
        self.choices = list(choices)

    def default(self):
        return self.choices[0]

    def validate(self, v) -> bool:
        return v in self.choices


class IntParam(Param):
    """Bounded integer parameter rendered as a slider in the modal. `unit`
    is shown next to the current value (purely cosmetic)."""

    def __init__(self, name, min_v: int, max_v: int, default: int = None,
                 step: int = 1, unit: str = ""):
        self.name = name
        self.min_v = int(min_v)
        self.max_v = int(max_v)
        self.step = max(1, int(step))
        self.unit = unit
        self._default = self.min_v if default is None else int(default)
        self._default = max(self.min_v, min(self.max_v, self._default))

    def default(self):
        return self._default

    def validate(self, v) -> bool:
        try:
            iv = int(v)
        except (TypeError, ValueError):
            return False
        return self.min_v <= iv <= self.max_v

    def coerce(self, v):
        return int(v)


class Action:
    """One action declaration. Override `to_c` per-action for non-trivial
    firmware emission; the default emits a comment matching the legacy
    exporter output."""

    def __init__(self, id, label, params=()):
        self.id = id
        self.label = label
        self.params = list(params)

    def has_params(self) -> bool:
        return bool(self.params)

    def default_args(self) -> dict:
        return {p.name: p.default() for p in self.params}

    def format(self, args) -> str:
        if not self.params:
            return self.label
        parts = [str(args.get(p.name, "?")) for p in self.params]
        return f"{self.label} ({', '.join(parts)})"

    def to_c(self, args, wp=None) -> str:
        """Emit C code for this action. `wp` is the (x, y, a) of the waypoint
        the action is attached to, so subclasses that need a position (e.g.
        recalibration) can reference it. The default emits a comment line."""
        if not args:
            return f"// action: {self.id}"
        arg_str = " ".join(f"{k}={v}" for k, v in args.items())
        return f"// action: {self.id} {arg_str}"


class ActionCatalog:
    def __init__(self, actions, legacy_aliases=None):
        self.actions = list(actions)
        self._by_id = {a.id: a for a in self.actions}
        self.legacy_aliases = dict(legacy_aliases or {})

    def get(self, aid):
        return self._by_id.get(aid)

    def ids(self):
        return [a.id for a in self.actions]

    def label(self, aid) -> str:
        a = self.get(aid)
        return a.label if a else aid

    def has_params(self, aid) -> bool:
        a = self.get(aid)
        return bool(a and a.params)

    def params(self, aid):
        a = self.get(aid)
        return list(a.params) if a else []

    def default_args(self, aid) -> dict:
        a = self.get(aid)
        return a.default_args() if a else {}

    def format(self, aid, args) -> str:
        a = self.get(aid)
        return a.format(args) if a else aid

    def to_c(self, aid, args, wp=None) -> str:
        a = self.get(aid)
        return a.to_c(args, wp=wp) if a else f"// unknown action: {aid}"

    def normalize(self, raw):
        """Accept str | [id, args] | (id, args). Apply legacy aliases, fill
        defaults, drop unknown ids. Return (id, args_dict) or None."""
        if isinstance(raw, str):
            aid, args = raw, {}
        elif isinstance(raw, (list, tuple)) and len(raw) >= 1:
            aid = raw[0]
            args = dict(raw[1]) if len(raw) >= 2 and isinstance(raw[1], dict) else {}
        else:
            return None
        if aid in self.legacy_aliases:
            new_id, new_args = self.legacy_aliases[aid]
            aid = new_id
            for k, v in new_args.items():
                args.setdefault(k, v)
        action = self.get(aid)
        if action is None:
            return None
        out = {}
        for p in action.params:
            v = args.get(p.name)
            out[p.name] = p.coerce(v) if (v is not None and p.validate(v)) else p.default()
        return (aid, out)

    def normalize_list(self, raw_list):
        out = []
        for item in raw_list or []:
            n = self.normalize(item)
            if n is not None:
                out.append(n)
        return out


@dataclass(frozen=True)
class Competition:
    """One year's setup. Paths are absolute — each year subpackage resolves
    them against its own folder so the visualizer doesn't need to know about
    a shared asset directory.

    `start_positions` maps side ("blue" | "yellow") to a list of robot poses
    `(x_mm, y_mm, theta_rad)`. The Nth connecting robot is placed at the Nth
    pose for its side; missing entries fall back to the first."""

    id: str
    label: str
    table_size_mm: tuple[int, int]            # width, height
    table_mins: tuple[int, int]               # x_min, y_min (world coords)
    bg_image_path: str                        # absolute path to board background
    robot_radius_mm: float
    robot_images: dict[int, str]              # team (0=blue, 1=yellow) -> abs path
    start_positions: dict[str, list[tuple[float, float, float]]]
    actions: "ActionCatalog"
    fixed_obstacles: tuple = field(default_factory=tuple)
