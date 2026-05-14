"""Hardcoded catalog of waypoint actions, with optional enum parameters.

In-memory shape: a queued action is `(id, args_dict)`. New entries get default
args from the catalog; the loader normalises old string-only entries from
state.json into the new shape."""

ACTION_CATALOG = [
    {
        "id": "play_sound",
        "label": "play sound",
        "params": [("sample", ["coin", "windows_start", "oh_la_je_suis_bien"])],
    },
    {
        "id": "recalibration",
        "label": "recalibration",
        "params": [("direction", ["NORTH", "SOUTH", "EAST", "WEST"])],
    },
    {"id": "actionner_down", "label": "actionner down", "params": []},
    {"id": "actionner_up", "label": "actionner up", "params": []},
]

_BY_ID = {a["id"]: a for a in ACTION_CATALOG}

_LEGACY_ALIASES = {"coin_sound": ("play_sound", {"sample": "coin"})}


def action_ids():
    return [a["id"] for a in ACTION_CATALOG]


def action_label(aid):
    a = _BY_ID.get(aid)
    return a["label"] if a else aid


def action_has_params(aid):
    a = _BY_ID.get(aid)
    return bool(a and a["params"])


def action_param_specs(aid):
    a = _BY_ID.get(aid)
    return list(a["params"]) if a else []


def action_default_args(aid):
    a = _BY_ID.get(aid)
    if not a:
        return {}
    return {name: choices[0] for name, choices in a["params"]}


def format_action(aid, args):
    label = action_label(aid)
    specs = action_param_specs(aid)
    if not specs:
        return label
    parts = [str(args.get(name, "?")) for name, _ in specs]
    return f"{label} ({', '.join(parts)})"


def normalize_action(raw):
    """Accept str | [id, args] | (id, args). Apply legacy aliases. Fill defaults.
    Drop entries with unknown id. Return (id, args_dict) or None."""
    if isinstance(raw, str):
        aid, args = raw, {}
    elif isinstance(raw, (list, tuple)) and len(raw) >= 1:
        aid = raw[0]
        args = dict(raw[1]) if len(raw) >= 2 and isinstance(raw[1], dict) else {}
    else:
        return None
    if aid in _LEGACY_ALIASES:
        new_id, new_args = _LEGACY_ALIASES[aid]
        aid = new_id
        for k, v in new_args.items():
            args.setdefault(k, v)
    if aid not in _BY_ID:
        return None
    for k, v in action_default_args(aid).items():
        args.setdefault(k, v)
    valid = {name: choices for name, choices in action_param_specs(aid)}
    args = {k: v for k, v in args.items() if k in valid and v in valid[k]}
    for k, v in action_default_args(aid).items():
        args.setdefault(k, v)
    return (aid, args)


def normalize_actions_list(raw_list):
    out = []
    for item in raw_list or []:
        n = normalize_action(item)
        if n is not None:
            out.append(n)
    return out
