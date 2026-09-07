# vla-edge-manipulation

Vision-Language-Action model (SmolVLA) driving an SO-101 arm, developed
against a MuJoCo simulation and deployed to an NVIDIA Jetson Orin Nano.
Final Year Project.

## Layout

- `src/vla_edge_manipulation/schema.py` — the shared contract (joint order,
  dimensions, units, camera keys). Everything else imports it.
- `src/vla_edge_manipulation/backends/` — `RobotBackend` interface plus
  implementations (`mock` now; `sim`/`real` land as those phases start).
- `configs/` — copy each `*.yaml.example` to `*.yaml` (gitignored) and fill
  in machine-specific values (ports, paths, camera indices).
- `tests/` — schema and backend conformance tests.
- `docs/decisions.md`, `docs/experiments.md`, `docs/sessions.md` —
  append-only logs.
- `.claude/skills/` — Claude Code skills for recurring project actions
  (recording a dataset, changing the schema, adding a backend, training,
  evaluation, Jetson benchmarking).

## Setup

Uses the `lerobot` conda env described in the parent FYP `CLAUDE.md`. From
this repo:

```bash
conda activate lerobot
uv pip install --python "$CONDA_PREFIX/bin/python" -e ".[dev]"
pytest tests/ -v
```

## Status

Repo skeleton in progress. See `docs/decisions.md` for what's been decided
so far.
