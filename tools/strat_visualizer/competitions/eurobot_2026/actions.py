"""Action catalog for Eurobot 2026. Composes shared Eurobot actions from
`competitions/eurobot_actions.py`. Override one of them in this file (or
subclass locally) if 2026 needs different C emission or params."""

from competitions.base import ActionCatalog, EnumParam, IntParam
from competitions.eurobot_actions import (
    ActionnerHeightAction, PlaySoundAction, RecalibrationAction,
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
        ActionnerHeightAction(
            id="actionner_height",
            label="actionner height",
            params=[IntParam("height", min_v=0, max_v=30, default=0, unit="cm")],
        ),
    ],
    legacy_aliases={
        "coin_sound": ("play_sound", {"sample": "coin"}),
        # 2025 vocabulary → 2026 height. Approximate mappings so existing paths
        # don't lose their actionner steps when run under 2026.
        "actionner_down": ("actionner_height", {"height": 0}),
        "actionner_up": ("actionner_height", {"height": 30}),
    },
)
