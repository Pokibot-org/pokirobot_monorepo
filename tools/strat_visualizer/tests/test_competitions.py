"""Smoke tests for the multi-competition refactor.

Covers: catalog round-trip (legacy aliases, normalization, C emission) and
WaypointModal/ActionConfigModal instantiation — the latter regressed at least
once already because the constructor signature changed across two callsites.

Run from the visualizer root with:
    PYTHONNOUSERSITE=1 guix shell python python-pygame python-numpy \\
        python-shapely python-paho-mqtt -- python3 -m unittest discover tests
"""
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "src"))

# pygame requires SDL_VIDEODRIVER to be set for headless test runs.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from competitions import default_competition_id, list_competitions, load_competition


class CompetitionLoaderTests(unittest.TestCase):
    def test_list_lists_existing_year_folders(self):
        ids = list_competitions()
        self.assertIn("eurobot_2025", ids)
        self.assertIn("eurobot_2026", ids)

    def test_default_is_latest_year(self):
        # competitions/ currently has 2025 and 2026; latest is 2026.
        self.assertEqual(default_competition_id(), "eurobot_2026")

    def test_load_accepts_year_int_and_full_id(self):
        a = load_competition(2026)
        b = load_competition("2026")
        c = load_competition("eurobot_2026")
        self.assertIs(a, b)
        self.assertIs(b, c)

    def test_load_unknown_raises(self):
        with self.assertRaises(ModuleNotFoundError):
            load_competition(2099)


class ActionCatalogTests(unittest.TestCase):
    def setUp(self):
        self.cat = load_competition("eurobot_2026").actions

    def test_ids(self):
        # 2026 replaced actionner_down/up with a height slider.
        self.assertEqual(
            self.cat.ids(),
            ["play_sound", "recalibration", "actionner_height"],
        )

    def test_legacy_actionner_down_up_map_to_height(self):
        self.assertEqual(
            self.cat.normalize("actionner_down"),
            ("actionner_height", {"height": 0}),
        )
        self.assertEqual(
            self.cat.normalize("actionner_up"),
            ("actionner_height", {"height": 30}),
        )

    def test_int_param_round_trip_keeps_int_type(self):
        aid, args = self.cat.normalize(("actionner_height", {"height": 15}))
        self.assertEqual(aid, "actionner_height")
        self.assertIsInstance(args["height"], int)
        self.assertEqual(args["height"], 15)

    def test_int_param_out_of_range_falls_back_to_default(self):
        # 50 cm is above the 30 cm max — should snap to default (0).
        _, args = self.cat.normalize(("actionner_height", {"height": 99}))
        self.assertEqual(args["height"], 0)

    def test_int_param_float_input_coerces_to_int(self):
        _, args = self.cat.normalize(("actionner_height", {"height": 12.0}))
        self.assertEqual(args["height"], 12)
        self.assertIsInstance(args["height"], int)

    def test_to_c_actionner_height_converts_cm_to_mm(self):
        out = self.cat.to_c("actionner_height", {"height": 25})
        self.assertIn("actionner_set_height(250)", out)
        self.assertIn("25 cm", out)

    def test_legacy_alias_normalizes(self):
        # `coin_sound` -> `play_sound` with sample=coin.
        self.assertEqual(
            self.cat.normalize("coin_sound"),
            ("play_sound", {"sample": "coin"}),
        )

    def test_normalize_fills_defaults(self):
        # missing args fall back to the first enum choice.
        self.assertEqual(
            self.cat.normalize(("recalibration", {})),
            ("recalibration", {"direction": "NORTH"}),
        )

    def test_normalize_rejects_invalid_enum(self):
        # invalid choice silently swapped for the default.
        aid, args = self.cat.normalize(("recalibration", {"direction": "SOUTHWEST"}))
        self.assertEqual(aid, "recalibration")
        self.assertEqual(args["direction"], "NORTH")

    def test_normalize_drops_unknown_id(self):
        self.assertIsNone(self.cat.normalize("not_a_real_action"))

    def test_normalize_list_skips_invalid(self):
        out = self.cat.normalize_list([
            "coin_sound",
            "not_a_real_action",
            ("recalibration", {"direction": "EAST"}),
        ])
        self.assertEqual(len(out), 2)
        self.assertEqual(out[0][0], "play_sound")
        self.assertEqual(out[1], ("recalibration", {"direction": "EAST"}))

    def test_format_includes_param_values(self):
        self.assertIn("coin", self.cat.format("play_sound", {"sample": "coin"}))
        self.assertEqual(
            self.cat.format("actionner_height", {"height": 12}),
            "actionner height (12)",
        )

    def test_to_c_recalibration_uses_waypoint_and_direction(self):
        out = self.cat.to_c(
            "recalibration", {"direction": "EAST"}, wp=(100.0, 200.0, 1.5),
        )
        self.assertIn("wall_recalibration", out)
        self.assertIn("recal_dir_e", out)
        self.assertIn("100.000f", out)
        self.assertIn("200.000f", out)

    def test_to_c_play_sound_uses_sample(self):
        out = self.cat.to_c("play_sound", {"sample": "windows_start"})
        self.assertIn("SOUND_WINDOWS_START", out)


class CompetitionMetadataTests(unittest.TestCase):
    def test_assets_exist(self):
        for cid in list_competitions():
            c = load_competition(cid)
            self.assertTrue(
                os.path.exists(c.bg_image_path),
                f"{cid}: missing bg at {c.bg_image_path}",
            )
            for team, path in c.robot_images.items():
                self.assertTrue(
                    os.path.exists(path),
                    f"{cid}: missing team {team} robot at {path}",
                )

    def test_start_positions_have_both_sides(self):
        for cid in list_competitions():
            c = load_competition(cid)
            self.assertIn("blue", c.start_positions)
            self.assertIn("yellow", c.start_positions)
            self.assertTrue(c.start_positions["blue"])
            self.assertTrue(c.start_positions["yellow"])


class StateIoTests(unittest.TestCase):
    """One file per competition (`state.<id>.json`). Migration from old
    formats happens lazily on read: very-old flat blob → 2025; shared
    multi-bucket blob → split per competition on demand."""

    def test_round_trip_per_competition_file(self):
        import json
        import tempfile
        from io_utils import state_load, state_save

        with tempfile.TemporaryDirectory() as d:
            state_save(d, "eurobot_2025", "blue", [(1.0, 2.0, 0.0, [])], ["a"])
            state_save(d, "eurobot_2026", "yellow", [(10.0, 20.0, 0.5, [])], ["z"])
            self.assertTrue(os.path.exists(os.path.join(d, "state.eurobot_2025.json")))
            self.assertTrue(os.path.exists(os.path.join(d, "state.eurobot_2026.json")))
            self.assertFalse(os.path.exists(os.path.join(d, "state.json")))
            self.assertEqual(state_load(d, "eurobot_2025")["side"], "blue")
            self.assertEqual(state_load(d, "eurobot_2026")["side"], "yellow")
            # Saving 2026 must not touch 2025.
            state_save(d, "eurobot_2026", "blue", [], [])
            self.assertEqual(state_load(d, "eurobot_2025")["side"], "blue")
            self.assertEqual(state_load(d, "eurobot_2026")["side"], "blue")

    def test_legacy_flat_state_falls_back_to_2025(self):
        import json
        import tempfile
        from io_utils import state_load

        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "state.json"), "w") as f:
                json.dump(
                    {"side": "blue", "waypoints": [[1.0, 2.0, 0.0, []]], "labels": ["a"]},
                    f,
                )
            self.assertEqual(state_load(d, "eurobot_2025")["side"], "blue")
            self.assertIsNone(state_load(d, "eurobot_2026"))

    def test_legacy_multi_bucket_state_splits_by_id(self):
        import json
        import tempfile
        from io_utils import state_load

        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "state.json"), "w") as f:
                json.dump({
                    "active": "eurobot_2026",
                    "competitions": {
                        "eurobot_2025": {"side": "blue", "waypoints": [], "labels": []},
                        "eurobot_2026": {"side": "yellow", "waypoints": [], "labels": []},
                    },
                }, f)
            self.assertEqual(state_load(d, "eurobot_2025")["side"], "blue")
            self.assertEqual(state_load(d, "eurobot_2026")["side"], "yellow")
            self.assertIsNone(state_load(d, "eurobot_2099"))

    def test_per_competition_file_takes_priority_over_legacy(self):
        import json
        import tempfile
        from io_utils import state_load

        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "state.json"), "w") as f:
                json.dump({"side": "blue", "waypoints": [], "labels": []}, f)
            with open(os.path.join(d, "state.eurobot_2025.json"), "w") as f:
                json.dump({"side": "yellow", "waypoints": [], "labels": []}, f)
            # per-competition file wins.
            self.assertEqual(state_load(d, "eurobot_2025")["side"], "yellow")


class ModalInstantiationTests(unittest.TestCase):
    """Regression tests for the WaypointModal/ActionConfigModal callsites.
    Bug history: `_open_config_for_add` passed args in a different shape than
    `_open_config_for_queued`, so a search-and-replace on the constructor
    signature missed one — and only crashed at runtime when the user clicked
    'Add action'. These tests instantiate both paths."""

    def setUp(self):
        import pygame as pg
        pg.init()
        pg.display.set_mode((640, 480))
        self.pg = pg

        from path_planner import PathPlanner
        catalog = load_competition("eurobot_2026").actions
        self.planner = PathPlanner(catalog=catalog)
        self.planner.insert_waypoint(0, 0.0, 0.0, 0.0, [], label="wp0")

    def tearDown(self):
        self.pg.quit()

    def test_modal_instantiation_and_both_action_config_paths(self):
        from waypoint_modal import ActionConfigModal, WaypointModal

        modal = WaypointModal(
            self.planner, 0, anchor_screen_pos=(100, 100), on_close=lambda: None,
        )

        # Direct: queued-action edit path (action with params).
        modal._sub_modal = None
        self.planner.set_actions(0, [("recalibration", {"direction": "NORTH"})])
        modal._open_config_for_queued(0)
        self.assertIsInstance(modal._sub_modal, ActionConfigModal)

        # The other path: "Add action" with params — this was the broken one.
        modal._sub_modal = None
        modal._open_config_for_add("recalibration")
        self.assertIsInstance(modal._sub_modal, ActionConfigModal)


class SimulatorTests(unittest.TestCase):
    """The simulator reads from `planner.waypoints` live, so edits during
    playback are picked up. goto_index/step_prev/step_next snap the robot
    to a specific waypoint."""

    def setUp(self):
        from path_planner import PathPlanner
        from path_simulator import PathSimulator
        from world import World

        self.world = World()
        catalog = load_competition("eurobot_2026").actions
        self.planner = PathPlanner(catalog=catalog)
        for i, (x, y, a) in enumerate(
            [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (200.0, 0.0, 0.0), (300.0, 0.0, 0.0)]
        ):
            self.planner.insert_waypoint(i, x, y, a, [], label=f"w{i}")
        self.sim = PathSimulator(self.world, self.planner)

    def test_goto_index_snaps_pose_and_pauses(self):
        self.sim.goto_index(2)
        self.assertFalse(self.sim.running)
        self.assertEqual(self.sim.wp_index, 3)  # next target is i+1
        self.assertAlmostEqual(self.sim.robot.pos[0], 200.0)

    def test_step_prev_goes_one_back(self):
        self.sim.goto_index(2)  # at wp2, next=3
        self.sim.step_prev()
        # after prev: was at wp2 → back to wp1
        self.assertAlmostEqual(self.sim.robot.pos[0], 100.0)
        self.assertEqual(self.sim.wp_index, 2)

    def test_step_prev_clamps_at_zero(self):
        self.sim.goto_index(0)
        self.sim.step_prev()
        self.assertAlmostEqual(self.sim.robot.pos[0], 0.0)

    def test_step_next_snaps_forward(self):
        self.sim.goto_index(0)  # at wp0, next=1
        self.sim.step_next()
        self.assertAlmostEqual(self.sim.robot.pos[0], 100.0)

    def test_live_edit_retargets_robot(self):
        """Moving the current target waypoint while running should make
        the simulator pick the new position up on the next step."""
        self.sim.play_pause()
        # robot starts at wp0, targeting wp1 (100,0).
        self.sim.step(0.01)
        orig_x = self.sim.robot.pos[0]
        # Move wp1 to (-500, 0) — robot should now head left.
        self.planner.waypoints[1] = (-500.0, 0.0, 0.0, [])
        self.sim.step(0.01)
        self.assertLess(self.sim.robot.pos[0], orig_x,
                        "robot should reverse course after target moved left")

    def test_step_clamps_wp_index_when_waypoints_deleted(self):
        self.sim.play_pause()
        self.sim.wp_index = 3
        # Delete all but one
        self.planner.waypoints = self.planner.waypoints[:1]
        self.sim.step(0.01)
        self.assertLessEqual(self.sim.wp_index, len(self.planner.waypoints))


if __name__ == "__main__":
    unittest.main()
