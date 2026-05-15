#!/usr/bin/env python
"""Strat visualizer entry point — parse CLI flags, load the competition
config, then start the app."""
import argparse
import logging
import os
import sys

# `competitions/` lives at the visualizer root (sibling of src/) so adding a
# new year is just dropping a folder there — no edits under src/.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app import PokibotGameSimulator
from competitions import default_competition_id, list_competitions, load_competition


def _parse_args(argv):
    p = argparse.ArgumentParser(description="Pokibot strat visualizer")
    p.add_argument(
        "--competition", "-c",
        default=default_competition_id(),
        help=(
            f"Competition id or year (e.g. 2026, eurobot_2026). Defaults to "
            f"the latest folder under competitions/ "
            f"(available: {', '.join(list_competitions()) or 'none'})."
        ),
    )
    return p.parse_args(argv)


def main(argv=None):
    logging.basicConfig(level=logging.INFO)
    args = _parse_args(argv)
    try:
        competition = load_competition(args.competition)
    except ModuleNotFoundError:
        sys.stderr.write(f"unknown competition: {args.competition!r}\n")
        return 2
    logging.info("loaded competition %s (%s)", competition.id, competition.label)
    PokibotGameSimulator(competition).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
