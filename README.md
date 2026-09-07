# vla-edge-manipulation

Vision-Language-Action model (VLA) driving an SO-101 arm, developed
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

## Setup

## Local environment

Conda env `lerobot` (Python 3.12), created via:
```bash
conda create -y -n lerobot python=3.12
conda activate lerobot
conda activate lerobot
uv pip install --python "$CONDA_PREFIX/bin/python" -e ".[dev]"
pytest tests/ -v
```

## Status

Repo skeleton in progress. See `docs/decisions.md` for what's been decided
so far.
