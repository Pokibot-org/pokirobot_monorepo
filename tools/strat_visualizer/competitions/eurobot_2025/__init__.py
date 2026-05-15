"""Eurobot 2025 competition config."""

import os

from competitions.base import Competition

from .actions import CATALOG


_HERE = os.path.dirname(os.path.abspath(__file__))


def _img(name: str) -> str:
    return os.path.join(_HERE, "img", name)


competition = Competition(
    id="eurobot_2025",
    label="Eurobot 2025",
    table_size_mm=(3000, 2000),
    table_mins=(-1500, 0),
    bg_image_path=_img("bg.png"),
    robot_radius_mm=180.0,
    robot_images={
        0: _img("robot_blue.png"),
        1: _img("robot_yellow.png"),
    },
    start_positions={
        "blue": [(0.0, 1000.0, 0.0)],
        "yellow": [(0.0, 1000.0, 0.0)],
    },
    actions=CATALOG,
)
