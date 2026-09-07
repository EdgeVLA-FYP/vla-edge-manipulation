# vla-edge-manipulation

Vision-Language-Action model (SmolVLA) driving an SO-101 arm, developed
against a MuJoCo simulation and deployed to an NVIDIA Jetson Orin Nano.
University Final Year Project, team org `EdgeVLA-FYP`.

This file is project-wide context for anyone (or any agent) working in this
repo, on any machine. It intentionally does not encode a phase-by-phase
roadmap — the plan changes as the project learns things; see "Where things
actually stand" below for where the live version of that lives.

## The five rules that carry most of the value

1. **One schema file, owned deliberately.** `src/vla_edge_manipulation/schema.py`
   defines joint order, dimensions, units, camera keys, gripper convention.
   Everything else imports it; it imports nothing project-specific. Changing
   it can silently invalidate every dataset recorded before the change — see
   the `schema-change` skill (or just: grep for usages, add a round-trip
   test if it's a unit change, run the full test suite, log it in
   `docs/decisions.md`).
2. **One backend interface, multiple implementations.** `backends/base.py`'s
   `RobotBackend` is the only thing the rest of the code may depend on —
   never branch on "is this sim or real" above that layer. `backends/mock.py`
   exists so anyone can develop against the interface with no hardware and
   no simulator installed.
3. **Configuration in files, never hardcoded.** Ports, camera indices,
   calibration paths, and task definitions belong in `configs/*.yaml`
   (copied from the committed `*.yaml.example` templates and gitignored) —
   never inline in source. Different machines have different serial ports;
   hardcoding one breaks it for everyone else.
4. **Nobody pushes to `main` directly.** Branch per task, open a PR, get at
   least a skim review before merging. Keep branches short-lived.
5. **Install dependencies when the work in front of you actually needs
   them, not upfront.** See below — this one has already saved real time on
   this project.

## Dependency philosophy: pull things in only when needed

Don't install `lerobot`, training extras, or anything GPU/CUDA-related
speculatively "because the roadmap will need it eventually." Concretely:

- Early sim work (`backends/sim.py`, loading the MuJoCo model, verifying
  joints/gripper conversion) needs only `mujoco` — already declared as this
  project's `sim` extra in `pyproject.toml`. It does not need `lerobot`.
- Add `lerobot` as a normal pinned dependency only once you're actually
  touching `LeRobotDataset`, loading a policy checkpoint, or running
  `lerobot-train` — not before. Treat it as a library dependency of this
  project (pinned version in `pyproject.toml`), not as a cloned/editable
  side project, unless you specifically need to patch its internals.
- On a CPU-only dev machine, watch for PyPI's default Linux `torch` wheel
  pulling in CUDA runtime packages even with no GPU present. Install with
  `uv`'s `--torch-backend cpu` (or pin the `https://download.pytorch.org/whl/cpu`
  index) to avoid a multi-GB unusable download.
- Reasoning: the roadmap and architecture *will* shift as the project
  learns things (they already have once). Installing the full heavy stack
  up front means reinstalling/undoing it every time the plan changes.
  Installing incrementally also builds a better mental model of what each
  phase of the project actually depends on, instead of an opaque pile of
  transitive dependencies nobody can account for.

## Repo layout

```
src/vla_edge_manipulation/
  schema.py          the shared contract — read this first
  backends/
    base.py          RobotBackend interface
    mock.py          fake backend, no hardware/sim required
    sim.py           (lands with the sim-backend work)
    real.py          (lands once hardware is available)
configs/              *.yaml.example templates; real *.yaml is gitignored
tests/                 schema validation + backend conformance tests
docs/
  decisions.md         append-only decision log — the schema/architecture
                        source of truth as it evolves
  experiments.md       append-only checkpoint/training-run registry
  sessions.md          append-only dataset-recording session log
```

## Environment

Each contributor's machine may differ; don't assume a specific OS/GPU.
What should stay consistent regardless of machine:

- A dedicated conda/virtualenv for this project (this repo's own
  convention so far: an env named `vla-edge`, Python 3.12). Install
  packages into it explicitly (e.g. `uv pip install --python
  "$CONDA_PREFIX/bin/python" ...`) rather than relying on ambient
  detection, since a stray local venv can silently outrank the intended
  environment.
- `pip install -e ".[dev]"` (add `sim` once doing MuJoCo work) from the
  repo root gets this project's own code importable and testable.
- `pytest tests/ -v` must pass before merging any change.
- MuJoCo needs a headless rendering backend on a machine with no GPU/no
  display — `MUJOCO_GL=egl` (software EGL via Mesa) works without sudo or a
  GPU on Linux, including under WSL2.

## Naming conventions

Keep these consistent so results and artifacts are traceable across the
team:

| Artefact | Pattern | Example |
|---|---|---|
| Dataset | `so101_{task}_{source}_{version}_{n}ep` | `so101_pickcube_sim_v1_50ep` |
| Training run | `{policy}_{task}_{data}_{variant}` | `smolvla_pickcube_real_v1_bf16` |
| Checkpoint | `{run_name}@{step}` | `smolvla_pickcube_real_v1@14000` |
| Eval result | `{checkpoint}_{protocol}_{date}` | `smolvla_v1@14000_realeval20_20261110` |

## Where things actually stand

This file deliberately doesn't say "we are on Phase N." For the current
state of the project, what's been decided and why, and what's next, read
(in this order):
1. `docs/decisions.md` — architectural/schema decisions, most recent last.
2. `docs/experiments.md` / `docs/sessions.md` — what's actually been run.
3. Open PRs and issues on the `EdgeVLA-FYP/vla-edge-manipulation` repo.

## Local Claude Code skills

If you use Claude Code, `.claude/skills/` may contain personal, gitignored
skills for recurring actions (recording a dataset, changing the schema,
adding a backend, training, evaluation, Jetson benchmarking). They aren't
shared via git — each contributor can create their own, or ask a teammate
for theirs.
