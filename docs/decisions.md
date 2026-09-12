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
different heuristics gave contradictory results), so it's grounded in a physics contact test instead (a real 2cm cube registered 13 contacts when the gripper closed on it, with no explosion/NaN) rather than a geometric measurement.

## 2026-09-10 — `scripts/view_sim.sh` for manual sim inspection

One shell script. Added `MuJoCoBackend.launch_interactive_viewer()` (wraps `mujoco.viewer.launch`) so the script reuses `connect()`'s config loading and workspace overrides rather than building its own model — the interactive view can't drift from what `get_observation()` returns. Needs a real GLFW/OpenGL display, not the offscreen EGL path the rest of the backend uses — `MUJOCO_GL=egl` is for camera rendering only, don't set it for this script.

## 2026-09-12 — CORRECTS 2026-09-08 entry: gripper direction was actually inverted

The 2026-09-08 entry's "max = closed, min = open" is **wrong** — verified by tracking the actual fingertip mesh vertices (farthest point from each geom's own origin) across the joint's raw range, not just checking whether whole-mesh bounding regions overlap: raw `jnt_range` **min** gives a ~6mm fingertip gap (closed) and **max** gives ~141mm (open). The earlier "mesh-to-mesh proximity" check was measuring proximity near the hinge pivot, which stays close regardless of angle, not the fingertip gap that actually matters. `_schema_to_joint_gripper`/`_joint_to_schema_gripper` (`backends/sim.py`) now map schema 0→lo, 100→hi directly (no reversal) — matches `test_gripper_min_is_closed_and_max_is_open`, updated to assert the corrected direction. Any dataset or calibration note written against the old mapping has the gripper's open/closed sense backwards.

## 2026-09-12 — Phase 1 controller: IK and grasp-approach geometry

Implements `MuJoCoBackend.get_body_pose()` (ground-truth pose of any named body) and `.solve_ik()` (damped-least-squares IK for the gripper's tool-centre-point, soft top-down-orientation regularizer), plus `PickPlaceController` (`pick_place_controller.py`), the home→approach→grasp→lift→bin→home state machine. Per issue #7.

Non-obvious decisions future controller/backend work should know:

- `solve_ik()` holds `wrist_roll` fixed at 0 rather than solving it — free, it barely moves the TCP but drifts between solves and swings the offset fingertip sideways.
- `_GRIPPER_TCP_OFFSET` is the fixed jaw's own fingertip mesh vertex, not the vendored MJCF's "gripperframe" site (checked and found not to track the moving jaw).
- The controller's descend/slide-in waypoints stop short of the object's centre (~1.2× half-size) instead of driving to it, and retreat radially before descending instead of going straight down — both avoid the fingertip pushing the object before the grasp.
- Supplementary grip pads were added to the vendored gripper mesh (`assets/robotstudio_so101/NOTICE.md`) — the bare mesh's near-pointed jaw tips only produced a glancing contact that couldn't survive a lift.

## 2026-09-12 — Phase 1 controller: grasp-reliability investigation, 0% → 93%

Measured via `scripts/measure_pickplace_success.sh` throughout (N=40-150 per step; final confirmed number uses `MuJoCoBackend(seed=42)`, N=100). Each step found by classifying actual failures (contact status, cube trajectory, per-episode instrumentation), not by sweeping parameters:

1. **0% → 38%: contact "slow slip".** Grip force decayed smoothly to zero over the lift instead of holding — MuJoCo's default pyramidal friction cone is a known weak case for stable grasping ([docs: "Preventing slip"](https://mujoco.readthedocs.io/en/latest/modeling.html#preventing-slip), [maintainer discussion #2309](https://github.com/google-deepmind/mujoco/discussions/2309)), independently confirmed for SO-101 specifically ([ggando.com/blog/so101-rl-lift](https://ggando.com/blog/so101-rl-lift/)). Fixed with `cone="elliptic" impratio="10" noslip_iterations="3"` (`so101.xml`) plus correctly-sized grip pads (2.5mm box, not a sphere — a sphere-on-flat-face contact is always a single point regardless of size).
2. **38% → 77%: release-sequencing bug.** The release step opened the gripper *while the arm was still moving* to the release position, flinging the cube sideways. Fixed by reaching the release position, settling, then opening as a separate held action.
3. **77% → 86%: torsional/rolling friction was silently inert.** MuJoCo's `condim` defaults to 3 (normal + sliding friction only); the cube/pad geoms' torsional/rolling friction coefficients had never been active. Set `condim="6"` on the cube geom (contacts inherit `max(condim)` of the pair automatically — no pad-geom edit needed). This is the sim equivalent of the SO-101 community's compliant/Fin-Ray gripper mod ([EmbodiedAI-Group/SO-ARM101-6DoF](https://github.com/EmbodiedAI-Group/SO-ARM101-6DoF)), which resists torque via contact area rather than friction coefficient.
4. **86% → 93%: two more geometry bugs.** (a) Release height put the cube's bottom face below the bin wall top, clipping it instead of clearing it — fixed by deriving release height from wall height + a real clearance margin (`_BIN_WALL_HEIGHT`, `_RELEASE_CLEARANCE`). (b) The transport reach's single large *horizontal* joint-space jump transiently shook the grip loose (the equally large *vertical* lift jump, tested identically, is stable) — fixed by interpolating the transport reach only (`_TRANSPORT_WAYPOINTS`).

**Current confirmed state: 93/100.** Remaining failures are a torque-imbalanced-grasp rotational-drift mechanism: cube angular velocity while gripped either damps to zero (success) or slowly diverges until contact is lost (failure), anywhere from ~1 to ~600+ steps later — reduced by condim=6, not eliminated.

**For whoever picks this up next:**
- An early-warning classifier (angular-velocity trend + per-pad contact-force imbalance) was tried against this mechanism and did not generalize: strong in-sample (52-67% recall, ~0-4% false-positive rate on 50 training trials) but degraded sharply on a held-out sample (recall down to 15-70%, false positives up to 20%). Don't add retry logic without validating on a much larger, held-out sample first.
- A "single large joint-space jump is safe" conclusion is specific to the contact-solver configuration it was tested under — step 4(b) above is a jump that was tested and ruled out safe *before* `condim=6` existed, then became a real bug after. Re-check jump safety after any contact-model change.
- Same-seed paired comparisons (identical cube-position sequence, one variable changed) are the standard here for validating a change — much stronger evidence than comparing independent-seed runs.
