"""Action catalog for Eurobot 2025. See competitions/eurobot_2026/actions.py
for the canonical layout — both years compose the same shared action classes."""

from competitions.base import ActionCatalog, EnumParam
from competitions.eurobot_actions import (
    ActionnerDownAction, ActionnerUpAction, PlaySoundAction, RecalibrationAction,
)


CATALOG = ActionCatalog(
    actions=[
        PlaySoundAction(
            id="play_sound",
            label="play sound",
            params=[EnumParam("sample", ["coin", "windows_start", "oh_la_je_suis_bien"])],
        ),
        RecalibrationAction(
            id="recalibration",
            label="recalibration",
            params=[EnumParam("direction", ["NORTH", "SOUTH", "EAST", "WEST"])],
        ),
        ActionnerDownAction(id="actionner_down", label="actionner down"),
        ActionnerUpAction(id="actionner_up", label="actionner up"),
    ],
    legacy_aliases={
        "coin_sound": ("play_sound", {"sample": "coin"}),
    },
)
