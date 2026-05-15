"""Reusable Eurobot action subclasses.

Year configs compose `ActionCatalog`s from these classes — see
`eurobot_2026/actions.py`. Edit `to_c` here when the firmware API changes
(or subclass per year if a year diverges).

For actions whose C API is not yet defined in `brain_app/`, emission is
prefixed with a `// FIXME` and uses an invented signature."""

from .base import Action


_RECAL_DIR_MAP = {
    "NORTH": "recal_dir_n",
    "SOUTH": "recal_dir_s",
    "EAST": "recal_dir_e",
    "WEST": "recal_dir_w",
}


class RecalibrationAction(Action):
    """brain_app/src/main.c: `wall_recalibration(color, pos2_t recal_start_pos,
    float offset, enum recal_dir dir)`. We use the attached waypoint as the
    start pos; `offset` is hardcoded to 0.0f with a FIXME — it varies per
    call site in main.c (M_PI / 0.0f) and is not yet surfaced as a
    visualizer-side parameter."""

    def to_c(self, args, wp=None) -> str:
        direction = _RECAL_DIR_MAP.get(args.get("direction"), "recal_dir_n")
        if wp is None:
            return (
                "// FIXME: recalibration emitted without waypoint context\n"
                f"wall_recalibration(color, /* recal_start_pos */ (pos2_t){{0}}, 0.0f, {direction});"
            )
        x, y, a = wp
        return (
            "// FIXME: offset hardcoded to 0.0f — confirm against call site in brain_app\n"
            f"wall_recalibration(color, "
            f"(pos2_t){{.x = {x:.3f}f, .y = {y:.3f}f, .a = {a:.6f}f}}, "
            f"0.0f, {direction});"
        )


class PlaySoundAction(Action):
    def to_c(self, args, wp=None) -> str:
        sample = args.get("sample", "coin")
        return (
            "// FIXME: no brain_app audio API yet — invented signature\n"
            f"play_sound(SOUND_{sample.upper()});"
        )


class ActionnerDownAction(Action):
    def to_c(self, args, wp=None) -> str:
        return (
            "// FIXME: no brain_app actionner API yet — invented signature\n"
            "actionner_set(ACTIONNER_DOWN);"
        )


class ActionnerUpAction(Action):
    def to_c(self, args, wp=None) -> str:
        return (
            "// FIXME: no brain_app actionner API yet — invented signature\n"
            "actionner_set(ACTIONNER_UP);"
        )


class ActionnerHeightAction(Action):
    """Move the actionner to a specific height. `height` arg is in cm; we
    emit mm to match brain_app's pos2_t / mm convention elsewhere."""

    def to_c(self, args, wp=None) -> str:
        height_cm = int(args.get("height", 0))
        height_mm = height_cm * 10
        return (
            "// FIXME: no brain_app actionner API yet — invented signature\n"
            f"actionner_set_height({height_mm}); // {height_cm} cm"
        )
