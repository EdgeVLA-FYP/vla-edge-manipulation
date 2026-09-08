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

- **Built**: `schema.py` (joint order, dims, camera keys, gripper convention, `validate_action`/`validate_observation`); `backends/base.py` (`RobotBackend` ABC); `backends/mock.py`.
- **Not yet**: `backends/sim.py` (MuJoCo), `backends/real.py` (SO-101 hardware), dataset recording, training, evaluation, config-loading code.

## Key decisions already reflected in code

- `ACTION_DIM`/`STATE_DIM` derive from `len(JOINT_NAMES)` — one list is the only place joint count can drift.
- Task instruction/definition lives in `configs/task_*.yaml`, not `schema.py` — schema is I/O shape and units only, never task-specific.
- `MockBackend` shipped before any real backend, so recording/eval/training code can be built and tested with zero hardware or simulator dependency.
- Gripper convention fixed once (LeRobot: 0 = closed, 100 = open) rather than left to each backend to decide.

## Deliberately not built yet

- A multi-robot registry — SO-101 is the only target; add one only when a second robot is real, not speculatively.
- A custom training loop — `lerobot-train` once `lerobot` is added (see `CLAUDE.md`'s dependency philosophy).
- Photorealistic rendering — not needed for the pipeline-rehearsal/evaluation role sim plays here.
- RL/reward design — imitation learning only.
