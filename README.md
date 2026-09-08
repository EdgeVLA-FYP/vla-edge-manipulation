# vla-edge-manipulation

Vision-Language-Action model (VLA) driving an SO-101 arm, developed
against a MuJoCo simulation and deployed to an NVIDIA Jetson Orin Nano.
Final Year Project.

## Layout

- `src/vla_edge_manipulation/schema.py` — the shared contract (joint order,
  dimensions, units, camera keys). Everything else imports it.
- `src/vla_edge_manipulation/backends/` — `RobotBackend` interface plus
  implementations (`mock`, `sim`; `real` lands once hardware is available).
- `assets/robotstudio_so101/` — vendored MuJoCo scene/meshes for the sim
  backend (see that directory's `NOTICE.md` for source and license).
- `configs/` — copy each `*.yaml.example` to `*.yaml` (gitignored) and fill
  in machine-specific values (ports, paths, camera indices).
- `tests/` — schema and backend conformance tests.
- `docs/decisions.md`, `docs/experiments.md`, `docs/sessions.md` —
  append-only logs.

## Setup

Conda env `vla-edge` (Python 3.12), created via:
```bash
conda create -y -n vla-edge python=3.12
conda activate vla-edge
uv pip install -e ".[dev]"        # add `sim` for MuJoCo work: -e ".[dev,sim]"
cp configs/robot_sim.yaml.example configs/robot_sim.yaml  # only needed for `sim`
pre-commit install
pytest tests/ -v
```

MuJoCo needs a headless rendering backend on a machine with no GPU/display —
`MUJOCO_GL=egl` (software EGL via Mesa) works without sudo or a GPU on
Linux, including under WSL2.

## Status

Sim backend (MuJoCo) is up alongside the mock backend. See
`docs/decisions.md` for what's been decided so far and
`docs/ARCHITECTURE.md` for current state.
