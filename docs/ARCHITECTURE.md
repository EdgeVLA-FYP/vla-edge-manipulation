# Architecture

Living document — reflects current system design, not history. Edit it in place as decisions change during implementation. For a dated log of *why* a decision was made, add an entry to `decisions.md` instead; this file should never accumulate "previously we did X" — just the current picture.

## The golden rule

Everything above the robot backend is identical for sim and real. Only the backend swaps:

```
SHARED (dataset format, policy I/O, training, evaluation)
        |
  RobotBackend (schema.py-shaped observation/action contract)
        |
  +-----------+-----------+
  |           |           |
 sim        real        mock
```

If a change requires touching anything outside `backends/` to make sim and real behave consistently, the abstraction has leaked — fix the backend, not the shared layer.

## Current state

- **Built**: `schema.py` (joint order, dims, camera keys, gripper convention, `validate_action`/`validate_observation`); `backends/base.py` (`RobotBackend` ABC); `backends/mock.py`; `backends/sim.py` (MuJoCo, SO-101, plus sim-only `get_body_pose()`/`solve_ik()` ground-truth accessors); `controllers/pick_place_controller.py` (`PickPlaceController` — scripted IK pick-place state machine, a ground-truth "expert" for dataset recording, not a `RobotBackend`); `scripts/view_sim.sh` for manual sim inspection.
- **Built, partially working**: `PickPlaceController` succeeds 93/100 (`scripts/measure_pickplace_success.sh`, seed=42). Remaining failures are a torque-imbalanced-grasp rotational-drift mechanism (cube angular velocity damps to zero or diverges until contact is lost) — see `docs/decisions.md` (2026-09-12) for the full fix history and for why a retry classifier wasn't added.
- **Not yet**: `backends/real.py` (SO-101 hardware), dataset recording, training, evaluation — Phase 2 (dataset recording, issue #8) is blocked on the grasp being reliable enough.

## Key decisions already reflected in code

- `ACTION_DIM`/`STATE_DIM` derive from `len(JOINT_NAMES)` — one list is the only place joint count can drift.
- Task instruction/definition lives in `configs/task_*.yaml`, not `schema.py` — schema is I/O shape and units only, never task-specific.
- `MockBackend` shipped before any real backend, so recording/eval/training code can be built and tested with zero hardware or simulator dependency.
- Gripper convention fixed once (LeRobot: 0 = closed, 100 = open) rather than left to each backend to decide.
- Arm-joint state/action values are in radians (MuJoCo's native unit), set by `backends/sim.py` as the first backend to ship; `backends/real.py` must convert to it. See `docs/decisions.md`.
- SO-101 sim scene/meshes are vendored under `assets/robotstudio_so101/`, not fetched at setup — see that directory's `NOTICE.md` and `docs/decisions.md`.
- The pick-cube task workspace (cube + target bin) lives in `so101.xml` itself, matching `configs/task_pickcube.yaml.example` — not a separate scene variant. Colours/sizes/camera mount are "standard-case" defaults, all overridable from `configs/robot_sim.yaml`'s `workspace:` block via `MuJoCoBackend.connect()` — recalibrating to real hardware is a config edit, not a scene edit.
- `MuJoCoBackend.reset_to_home()` re-randomizes the cube's position (within `cube_area_cm`, centered on the gripper's home XY) every call — this is sim-only behavior inside one backend's implementation, not a new `RobotBackend` method, per the golden rule above.
- Gripper raw-joint direction: `jnt_range` **min = closed** (~6mm fingertip gap), **max = open** (~141mm) — matches schema's 0=closed/100=open directly. (A 2026-09-08 check got this backwards; corrected 2026-09-12 — see `docs/decisions.md`.)
- `MuJoCoBackend.get_body_pose()`/`.solve_ik()` are sim-only ground-truth accessors (no `RobotBackend` equivalent — real hardware has no ground-truth object pose or Jacobian) used by `PickPlaceController`, the scripted "expert" that will drive Phase 2 dataset recording once its grasp is reliable. `solve_ik()` holds `wrist_roll` fixed and regularizes toward a top-down approach; see `docs/decisions.md` for why.
- The vendored gripper mesh has two small supplementary grip-pad geoms added (`assets/robotstudio_so101/NOTICE.md`) — the bare mesh's pointed jaw tips couldn't hold a small cube through a lift.

## Deliberately not built yet

- A multi-robot registry — SO-101 is the only target; add one only when a second robot is real, not speculatively.
- A custom training loop — `lerobot-train` once `lerobot` is added (see `CLAUDE.md`'s dependency philosophy).
- Photorealistic rendering — not needed for the pipeline-rehearsal/evaluation role sim plays here.
- RL/reward design — imitation learning only.
- Grasp-verification/retry logic in `PickPlaceController` — a classifier for this was built and found not to generalize (see `docs/decisions.md`); don't re-add retry logic without first validating a detector on a large (several-hundred-trial), held-out sample.
