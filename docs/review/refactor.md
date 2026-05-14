# strat_visualizer refactor suggestions

Observations from reading the strat_visualizer module after the
simulation pause / overlap-aware hover / gradient path / robot art / masonry
sidebar / file-split rounds. None are urgent — these are cleanups, not bug
fixes.

## 1. ~~`PokibotGameVisualizer.run()` is doing too much~~ — DONE

`run()` was split into `_build_ui`, `_build_bindings`, `_build_inputs_panel`,
`_load_state`, `_autosave_state`, `_handle_window_event`,
`_handle_preview_event`, `_handle_event`, `_handle_mouse_event`,
`_draw_frame`, and `_loop`. Action closures became real methods
(`_toggle_wps`, `_move_obstacle`, …). The whole file was also split out of
`main.py` into `app.py`, `path_planner.py`, `path_simulator.py`, `panels.py`,
`preview.py`, `game_viz.py`, `key_bindings.py`, `layout.py`, `mqtt_sim.py`,
`io_utils.py`, `world.py`, `constants.py`. `main.py` is now 10 lines.

## 2. ~~Two near-identical event loops~~ — DONE

Both loops now share `_handle_window_event(e) -> "quit" | "consumed" | None`,
and each has its own `_handle_event` / `_handle_preview_event` wrapper.

## 3. `PathSimulator` and `PoklegscomSim` duplicate waypoint-following kinematics

`PathSimulator.step` and `PoklegscomSim.movement_sim` both implement "move
toward current waypoint, advance index on arrival." The MQTT version is dumber
(no obstacle avoidance, no smoothing on angle). If they ever need to agree on
physics, factor into:

```python
def step_toward_waypoint(robot, wps, wp_index, dt, *, speed, ang_speed, pos_tol, ang_tol) -> int:
    # returns new wp_index
```

## 4. `motor_break` is plumbed via two parallel paths

- MQTT: `set_break` topic in `PoklegscomSim.process_poklegscom`.
- In-process: `PokibotGameSimulator._set_mqtt_paused` pokes
  `part.motor_break = paused` directly on every `PoklegscomSim` instance.

The direct-attribute path bypasses the messenger abstraction and depends on the
internal layout of `PokirobotSim.sim_process` (see #5). Cleaner options:

- Give `PoklegscomSim` a `set_break(paused: bool)` method and call that.
- Or publish to `set_break` locally via `MqttSimMessengerServer` — exercises
  the same code path real firmware uses.

## 5. `SimPart.sim_process` mixes `SimProcess` and nested `SimPart` instances

`PokirobotSim([PokuicomSim, PoklegscomSim, LidarSim])` stores sub-`SimPart`s
in a field literally named `sim_process` even though the items are themselves
`SimPart` objects, not `SimProcess`. The mqtt pause hook already relies on
this implicit shape via `isinstance(part, PoklegscomSim)`. Either:

- Split into `sub_parts: list[SimPart]` and `processes: list[SimProcess]`.
- Or annotate the union and add an `iter_parts()` traversal that yields only
  `SimPart` children.

## 6. Inverted callback semantics for pause/run

```python
on_user_running=lambda running: self.on_pause_changed(not running)
```

Works, but the boolean flips at every layer. Pick one orientation and stick
to it — rename the simulator hook to `on_user_paused` and pass
`not self.running` at the call sites in `PathSimulator`.

## 7. Per-frame `pg.font.SysFont`

- `PathPlanner.draw` calls `pg.font.SysFont("monospace", 14, bold=True)` every
  frame (caught in try/except for fallback). Slow on cold fontconfig caches and
  fragile on some guix setups.
- `PreviewScreen.draw` does three more `SysFont` calls per frame while the
  overlay is open.

Cache fonts once at startup; pass them through `game_viz` or a `Fonts`
dataclass.

## 8. `state.json` rewritten on every change inside the render loop

No throttling. Cheap today, but it'll hurt as more state is added. Options:

- Debounce: write at most every N seconds.
- Save only on `pg.QUIT`.

## 9. ~~Dead / dubious imports~~ — DONE

Removed during the file-split: `numpy.__config__.show`, `shapely.lib.box`,
along with the now-unused `threading`, `shutil`, `subprocess`, `time`,
`dataclass`, `json`, `LineString`, `Polygon`, `unary_union` from `main.py`.

## 10. ~~Main loop reaches into `PathPlanner._mode`~~ — DONE

`PathPlanner.is_dragging()` exists; all main-loop call sites use it.

## 11. Hover identity uses exact tuple equality

`PathPlanner.start_drag` and `delete_at` decide whether to honor the active
hovered waypoint via `self._hover_world == world_pos`. Two `screen_to_world`
calls in the same frame return identical tuples, so this works — but it breaks
silently if the world position is ever produced via a slightly different path
(rounding, snapping). Compare with a small tolerance, or store the chosen
index on hover and use it directly on click.

## 12. `PathPlanner` has grown several responsibilities

- Editing state: drag mode, undo, snap.
- Hover state: cycle through overlapping wps.
- Rendering: segments, arrows, markers, tooltip, alpha + gradient.
- Export: `to_c_string`.

The class is now ~300 lines and `draw()` takes optional `sim` and
`mouse_screen_pos` kwargs. Worth splitting render concerns (`PathRenderer`)
from interaction concerns (`PathPlanner`), with the renderer reading state
from the planner.

## 13. Masonry sidebar reflows can move panels under the cursor

`Sidebar` in `bottom` orientation places each panel into the shortest column
in declaration order. When the window resizes across a column-count
breakpoint, the assignment changes and panels visually jump. Acceptable
today, but if the panel set grows, consider remembering the previous column
assignment and reusing it unless the column count actually changed.

## 14. Robot image cache eviction is coarse

`GameZoneVisualizer._scaled_robot_img` clears the entire scaled-image dict
when it exceeds 8 entries. That's fine while we have two teams and a stable
zoom, but on a continuous resize the cache thrashes (clear + re-scale on
every quantised ratio bump). Two cheap fixes:

- LRU eviction instead of full clear.
- Quantise `rl_to_px_ratio` more aggressively (currently 4 decimals).

## 15. Alpha overlays allocate a fresh `SRCALPHA` surface per draw

`PathPlanner._draw_arrow_alpha`, `_draw_marker_alpha`, and
`_draw_segment_alpha` each create a small per-call `pg.Surface(..., SRCALPHA)`.
Small surfaces, but with N waypoints there are O(3N) allocations per frame
during simulation. A single persistent overlay (cleared and reused) would
remove the allocator pressure.

## 16. `KeyBindings.docs()` returns a fresh list every frame

`KeybindingsPanel.draw` calls `self.bindings.docs()` per frame and immediately
iterates. The list copy is trivial, but if doc rows are ever computed
dynamically (e.g. "[O/P] grid size (50 mm)" with the current value), they
should be cached or rendered to a surface only when state changes.
