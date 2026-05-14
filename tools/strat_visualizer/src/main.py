#!/usr/bin/env python
"""Strat visualizer entry point — configure logging and start the app."""
import logging

from app import PokibotGameSimulator


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    PokibotGameSimulator().run()
