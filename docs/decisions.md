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

Confirmed via joint names (exact match to `JOINT_NAMES`) that the real
hardware — Seeed's "LeRobot SO-ARM101" kit — is the SO-101 `schema.py`
already targets, not a different arm. No schema change.

## 2026-09-08 — Sim backend: MuJoCo assets vendored from TheRobotStudio/SO-ARM100

Sourced from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(Apache-2.0), `Simulation/SO101/` @ `eecbe3e0a9ebb23e25ad7b2759b03884c6660903`.
Vendored directly into `assets/robotstudio_so101/` (~16MB) rather than
fetched at setup time, so a clone alone is enough to run sim tests — no
network dependency in CI. Details/license/modifications: that directory's
`NOTICE.md`.

## 2026-09-08 — Sim backend: radians for arm joints; gripper direction verified, not assumed

Two conventions `backends/sim.py` set, both binding on `backends/real.py`:

- Arm joints: **radians** (MuJoCo's native unit — `schema.py` doesn't fix
  one, and `sim.py` shipped first).
- Gripper joint: **max = closed, min = open** — inverted from both the raw
  numeric range and schema's 0=closed/100=open. Confirmed by measuring
  mesh-to-mesh proximity at both extremes, not assumed. See
  `_schema_to_joint_gripper`/`_joint_to_schema_gripper`. Re-verify if the
  calibration file (e.g. `so101_old_calib.xml`) ever changes.

## 2026-09-09 — Sim backend: arm-joint actions validated against physical range

`send_action()` rejects an arm-joint value outside `jnt_range`, mirroring
the gripper's existing check. MuJoCo's actuator only clamps *force*, not
`ctrl` — an out-of-range target was silently accepted and crept to the
joint's hard stop instead of erroring. Range stays backend-local (read from
the model), not in `schema.py`, same reasoning as the gripper's radian range.

## 2026-09-09 — schema.py: validate_action rejects non-finite values

NaN in a non-gripper action slot passed both `validate_action()` and
`sim.py`'s range check (NaN compares False to everything, so neither
`<`/`>` flagged it) and would have reached MuJoCo's `ctrl` uncaught. Added
directly to `schema.py`, not backend-local — a non-finite command is
invalid for every backend, not just sim, and matters most for real
hardware. No existing datasets to invalidate.

## 2026-09-09 — Pick-cube task workspace added to the sim scene

Added a 2cm cube (free body) and an open-top blue bin to `so101.xml`, per
`configs/task_pickcube.yaml.example`'s instruction and
`success_criteria: cube_center_within_box_bounds`. Cube sits at the center
of `robot_sim.yaml.example`'s `cube_area_cm` spawn region; per-episode
randomization within that region is the recording script's job, not baked
into the static scene.

Cube size is a **first-pass estimate, not a verified fit** — exact
gripper aperture couldn't be pinned down reliably by mesh geometry (two
different heuristics gave contradictory results), so it's grounded in a
physics contact test instead (a real 2cm cube registered 13 contacts when
the gripper closed on it, with no explosion/NaN) rather than a geometric
measurement. Actual graspability — and, if needed, resizing — gets proven
when the scripted pick-place controller is built (next PR).

Also reworked the `front` camera: the previous framing (from the sim
backend PR, before task objects existed) pointed at an empty floor and is
now fully occluded by the arm's own body once the cube/bin are in the
scene. Repositioned as a look-at camera aimed at the workspace midpoint.

## 2026-09-09 — Workspace built as a "digital twin, standard-case defaults" — not matched to real hardware yet

No real hardware exists to measure yet, so the workspace (table colour,
cube size/colour, bin colour, front camera mount) uses plausible common-case
values instead of guessed-but-unlabeled ones, with every one of those
values overridable from `configs/robot_sim.yaml`'s new `workspace:` block —
applied by `MuJoCoBackend.connect()` onto the loaded model (same mechanism
already used for `physics_timestep`), not by hand-editing `so101.xml`. When
real hardware exists, recalibrating is a config edit, not a scene rebuild.
Table's checker-pattern debug texture replaced with a plain overridable
colour for the same reason — it was never meant to represent anything real.

**Supersedes** part of the previous entry: cube-position randomization is
no longer the recording script's job. It now lives in
`MuJoCoBackend.reset_to_home()` itself — sim resets the cube to a fresh
random position within `cube_area_cm` (centered on the gripper's home XY)
every time it's called, alongside resetting the arm. Kept out of the shared
`RobotBackend` interface (rule 2: no sim/real branching above the backend
layer) — for a real backend, `reset_to_home()`'s equivalent "workspace
reset" is a human physically moving the cube, which needs no code at all.
`MuJoCoBackend` also now takes an optional `seed` for reproducible episodes.

## 2026-09-09 — connect() now rejects a cube spawn area that overlaps the bin; friction is real config

A review pass (see commit `8a35783` for the full list of fixes, including
ones not worth a decisions.md entry) surfaced two things future
contributors need to know that aren't obvious from skimming the code:

- Widening `randomization.cube_area_cm` far enough makes the cube's spawn
  region (centered on the *live* gripper xpos, not a fixed point) overlap
  the bin. `connect()` now checks this — against the bin's actual wall
  geometry, not a hardcoded number — and raises `ValueError` rather than
  letting it happen silently.
- `workspace.table_friction`/`cube_friction` are real, applied config
  fields, not just cosmetic (colour/size were already config-driven;
  friction is now too, for anyone tuning grasp behavior).
