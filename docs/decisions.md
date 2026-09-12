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

Implements `MuJoCoBackend.get_body_pose()`/`.solve_ik()` (damped-least-squares IK for the gripper's TCP, soft top-down-orientation regularizer) and `PickPlaceController` (`controllers/pick_place_controller.py`), the home→approach→grasp→lift→bin→home state machine, per issue #7. `solve_ik()` holds `wrist_roll` fixed rather than solving it (free, it barely moves the TCP but drifts between solves and swings the offset fingertip sideways), and its TCP offset is the fixed jaw's own fingertip mesh vertex, not the vendored MJCF's "gripperframe" site (checked, doesn't track the moving jaw). The controller retreats radially before descending and stops short of the object's centre while sliding in, instead of driving straight through it; two supplementary grip pads were added to the vendored gripper mesh (`assets/robotstudio_so101/NOTICE.md`) since the bare jaw tips only produced a glancing contact.

## 2026-09-12 — Phase 1 controller: grasp-reliability investigation, 0% → 93%

Measured via `scripts/measure_pickplace_success.sh` (`MuJoCoBackend(seed=42)`, N=100 for the final number); each step found by classifying actual failures, not sweeping parameters. Contact "slow slip" (grip force decaying to zero over the lift, a known weakness of MuJoCo's default pyramidal friction cone — see [MuJoCo docs](https://mujoco.readthedocs.io/en/latest/modeling.html#preventing-slip)) took it from 0% to 38% via `cone="elliptic" impratio="10" noslip_iterations="3"` and correctly-sized grip pads (2.5mm box, not sphere). A release-sequencing bug — the gripper opening while the arm was still moving to the release position — took it to 77%. `condim` defaulting to 3 had silently disabled the torsional/rolling friction already set on the cube/pad geoms; setting `condim="6"` on the cube geom took it to 86%. A release height that clipped the bin wall instead of clearing it, and a single large horizontal transport jump that shook the grip loose (the equally large vertical lift jump is stable), took it to the current 93%. Remaining failures are a torque-imbalanced-grasp rotational-drift mechanism; an early-warning classifier for it didn't generalize to a held-out sample (see `docs/ARCHITECTURE.md`). Note: the transport-jump fix reverses a conclusion reached in an earlier round *before* `condim=6` existed (that a single big jump was safe) — re-verify trajectory-safety assumptions after any contact-model change.

## 2026-09-12 — `solve_ik()` warns on non-convergence; found a real reachability limit

`solve_ik()`'s DLS loop had no post-loop convergence check — it silently returned a best-effort joint solution even when the position error never reached `_IK_TOL`, with no signal to the caller (found in PR review). Instrumenting it across real episodes found this isn't rare: ~14% of calls during the lift/transport phase don't converge, all stuck against `wrist_flex`'s joint limit, with residuals up to 15mm — the arm's kinematics can't sustain a fixed height and top-down orientation through the full transport swing for some cube start positions. Added a `warnings.warn()` on non-convergence (`backends/sim.py`) so this is visible instead of silent; the underlying reachability limit itself is not fixed by this PR and is a plausible contributor to the remaining rotational-drift-labeled failures above — worth investigating directly (e.g. relaxing the orientation regularizer or lowering the transport height) before further grasp-mechanism work.

## 2026-09-12 — `solve_ik()` retry-with-relaxed-orientation fixes the wrist_flex reachability gap

Following up on the previous entry: `solve_ik()`/`_solve_ik_dls()` now retries with the orientation weight progressively relaxed (half, then zero), only when the default oriented solve fails to converge. First attempt (straight to `orient_weight=0`) measured 91/100 vs. the prior 93/100, initially judged as noise (independent-sample SE ≈ 2.5% at this rate) — **wrong methodology**: `MuJoCoBackend(seed=X)` is deterministic, so two same-seed runs aren't independent samples, they're a paired comparison, and the right check is which specific episodes changed. Doing that properly found 3 real success→fail regressions against only 1 fail→success rescue — a genuine effect, not noise. Root cause: dropping the orientation weight straight to 0 lets the redundant DOF drift to a *different* joint configuration that still satisfies position but sets up worse downstream transport dynamics (confirmed on one regressed episode: identical peak tilt to a gentler retry, but a different final configuration). Halving the weight first, escalating to 0 only if that still doesn't converge, keeps the solution closer to the original and fixes 2 of the 3 regressions; the third (episode 49 at seed=42) still needs the full escalation and still regresses. Measured: 94/100 (`MuJoCoBackend(seed=42)`, N=100; paired against the no-retry baseline: 1 regression, 2 rescues), with non-convergence eliminated entirely (0 warnings across 100 episodes, vs. ~14% of calls before). A weight-0.01-only retry (no escalation) measures higher still (95/100, 0 regressions) but leaves ~40% of the original non-convergence unresolved — rejected in favor of the two-stage version, since eliminating silent ground-truth corruption for future dataset recording was the actual point of this fix, not the success-rate metric itself. **Current confirmed state: 94/100.**
