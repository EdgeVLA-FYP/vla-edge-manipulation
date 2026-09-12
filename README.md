# vla-edge-manipulation

Vision-Language-Action model (VLA) driving an SO-101 arm, developed
against a MuJoCo simulation and deployed to an NVIDIA Jetson Orin Nano.
Final Year Project.

## Layout

- `src/vla_edge_manipulation/schema.py` — the shared contract (joint order,
  dimensions, units, camera keys). Everything else imports it.
- `src/vla_edge_manipulation/config.py` — shared YAML config loading, used
  by any module that reads a `configs/*.yaml`.
- `src/vla_edge_manipulation/backends/` — `RobotBackend` interface plus
  implementations (`mock`, `sim`; `real` lands once hardware is available).
- `src/vla_edge_manipulation/controllers/` — scripted controllers used as
  ground-truth "experts" for dataset recording (`pick_place_controller.py`).
  Grasp reliability is a known open problem — see `docs/decisions.md`.
- `src/vla_edge_manipulation/recording.py` — records a controller's
  episodes into `LeRobotDataset` format and validates them.
- `assets/robotstudio_so101/` — vendored MuJoCo scene/meshes for the sim
  backend (see that directory's `NOTICE.md` for source and license).
- `configs/` — copy each `*.yaml.example` to `*.yaml` (gitignored) and fill
  in machine-specific values (ports, paths, camera indices).
- `scripts/` — standalone dev-convenience scripts, not part of the installed package.
- `tests/` — schema and backend conformance tests.
- `docs/decisions.md`, `docs/experiments.md`, `docs/sessions.md` —
  append-only logs.

## Setup

Conda env `vla-edge` (Python 3.12), created via:
```bash
conda create -y -n vla-edge python=3.12
conda activate vla-edge
uv pip install -e ".[dev]"        # add `sim` for MuJoCo work, `recording` for dataset recording
cp configs/robot_sim.yaml.example configs/robot_sim.yaml    # only needed for `sim`
cp configs/task_pickcube.yaml.example configs/task_pickcube.yaml  # only needed for `recording`
pre-commit install
pytest tests/ -v
```

MuJoCo needs a headless rendering backend on a machine with no GPU/display — `MUJOCO_GL=egl` (software EGL via Mesa) works without sudo or a GPU on
Linux, including under WSL2. That's for offscreen camera rendering (`get_observation()`, tests, CI) only — don't set it for the interactive viewer below, which needs a real GLFW/OpenGL window instead.

## Running the simulation

With the `sim` extra installed (see Setup):
```bash
bash scripts/view_sim.sh
```
Opens an interactive window with joint/control sliders for manually posing the SO-101 arm and watching physics — the scene includes the pick-cube task workspace (cube + target bin). Close the window to exit.

To reproduce the pick-place controller's measured grasp success rate:
```bash
bash scripts/measure_pickplace_success.sh 25
```

## Recording a dataset

With the `recording` extra installed (see Setup):
```bash
bash scripts/record_dataset.sh --episodes 20 --repo-id EdgeVLA/so101_pickcube_sim_v1_20ep --push
```
Retries failed attempts rather than including them (see `docs/decisions.md`), validates the result, and (with `--push`) publishes it to the Hub. Log the `repo_id` in `docs/sessions.md` afterward.

## Status

Sim backend (MuJoCo) is up alongside the mock backend. A scripted IK
pick-place controller succeeds 94/100 (`docs/decisions.md`) and a
recording pipeline (`docs/ARCHITECTURE.md`) turns its successful episodes
into a published `LeRobotDataset`.
