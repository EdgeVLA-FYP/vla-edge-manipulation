# Decisions log

Append-only. One entry per decision that isn't obvious from reading the
code — especially anything touching `schema.py`. Never delete past entries,
even superseded ones; add a new entry that supersedes instead.

## Format

```
## YYYY-MM-DD — short title
What was decided, and why. What it invalidates (if anything).
```

## 2026-09-08 — Real arm: Seeed Studio "SO-ARM101" kit == SO-101

Confirmed via joint names (exact match to `JOINT_NAMES`) that the real hardware — Seeed's "LeRobot SO-ARM101" kit — is the SO-101 `schema.py` already targets, not a different arm. No schema change.

## 2026-09-08 — Sim backend: MuJoCo assets vendored from TheRobotStudio/SO-ARM100

Sourced from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0), `Simulation/SO101/` @ `eecbe3e0a9ebb23e25ad7b2759b03884c6660903`. Vendored directly into `assets/robotstudio_so101/` (~16MB) rather than
fetched at setup time, so a clone alone is enough to run sim tests — no network dependency in CI. Details/license/modifications: that directory's `NOTICE.md`.

## 2026-09-08 — Sim backend: radians for arm joints; gripper direction verified, not assumed

Two conventions `backends/sim.py` set, both binding on `backends/real.py`:

- Arm joints: **radians** (MuJoCo's native unit — `schema.py` doesn't fix one, and `sim.py` shipped first).
- Gripper joint: **max = closed, min = open** — inverted from both the raw numeric range and schema's 0=closed/100=open. Confirmed by     measuring mesh-to-mesh proximity at both extremes, not assumed. See `_schema_to_joint_gripper`/`_joint_to_schema_gripper`. Re-verify if the calibration file (e.g. `so101_old_calib.xml`) ever changes.

## 2026-09-09 — Sim backend: arm-joint actions validated against physical range

`send_action()` rejects an arm-joint value outside `jnt_range`, mirroring the gripper's existing check. MuJoCo's actuator only clamps *force*, not `ctrl` — an out-of-range target was silently accepted and crept to the joint's hard stop instead of erroring. Range stays backend-local (read from the model), not in `schema.py`, same reasoning as the gripper's radian range.

## 2026-09-09 — schema.py: validate_action rejects non-finite values

NaN in a non-gripper action slot passed both `validate_action()` and `sim.py`'s range check (NaN compares False to everything, so neither `<`/`>` flagged it) and would have reached MuJoCo's `ctrl` uncaught. Added directly to `schema.py`, not backend-local — a non-finite command is invalid for every backend, not just sim, and matters most for real hardware. No existing datasets to invalidate.

## 2026-09-09 — Pick-cube task workspace added to the sim scene, built as a "digital twin" (not matched to real hardware yet)

Added a 2cm cube (free body) and an open-top blue bin to `so101.xml`, per `configs/task_pickcube.yaml.example`'s instruction and `success_criteria: cube_center_within_box_bounds`. Cube sits at the center of `robot_sim.yaml.example`'s `cube_area_cm` spawn region.

Cube size is a **first-pass estimate, not a verified fit** — exact gripper aperture couldn't be pinned down reliably by mesh geometry (two
different heuristics gave contradictory results), so it's grounded in a physics contact test instead (a real 2cm cube registered 13 contacts when the gripper closed on it, with no explosion/NaN) rather than a geometric measurement. Actual graspability — and, if needed, resizing — gets proven when the scripted pick-place controller is built (next PR).

## 2026-09-10 — `scripts/view_sim.sh` for manual sim inspection

One shell script, not an installed console-script entry point — a
standalone `[project.scripts]` command needed a whole new importable
package plus a reinstall just to wrap one function call. Added
`MuJoCoBackend.launch_interactive_viewer()` (wraps `mujoco.viewer.launch`)
so the script reuses `connect()`'s config loading and workspace overrides
rather than building its own model — the interactive view can't drift from
what `get_observation()` returns. Needs a real GLFW/OpenGL display, not the
offscreen EGL path the rest of the backend uses — `MUJOCO_GL=egl` is for
camera rendering only, don't set it for this script.