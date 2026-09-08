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

The real hardware is the Seeed Studio "AI Robotic Arm Kit — LeRobot SO-ARM101"
(servo motors kit variant). Confirmed this is the same design `schema.py`
already targets, not a different arm: its official simulation assets (see
below) use joint names `shoulder_pan`/`shoulder_lift`/`elbow_flex`/`wrist_flex`/
`wrist_roll`/`gripper`, an exact match for `JOINT_NAMES`. No schema change.

## 2026-09-08 — Sim backend: MuJoCo assets vendored from TheRobotStudio/SO-ARM100

`backends/sim.py` needed an SO-101 MuJoCo model. Used the official one from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(Apache-2.0), `Simulation/SO101/`, pinned at commit
`eecbe3e0a9ebb23e25ad7b2759b03884c6660903`. Vendored directly into
`assets/robotstudio_so101/` (~16MB of MJCF + STL meshes) rather than fetched
at setup time: this repo is small enough that the extra history is cheap,
and direct vendoring means `git clone` alone is enough to run the sim
conformance tests — no network dependency in CI, no failure mode where
upstream becomes unavailable or renames a file. See that directory's
`NOTICE.md` for exactly what was taken and the (minimal, documented) local
modifications, license, and attribution.

## 2026-09-08 — Sim backend: arm-joint units are radians; gripper direction verified, not assumed

Two unit decisions in `backends/sim.py`, both binding on any future backend:

- The five arm joints are exchanged in **radians** (MuJoCo's native unit,
  set by the vendored scene's `<compiler angle="radian">`) — `schema.py`
  doesn't fix a unit for them beyond shape, and `sim.py` is the first
  backend to ship, so it sets the convention. `backends/real.py`, when it
  lands, converts to radians rather than this project switching to degrees.
- The vendored gripper joint's **maximum** (`+1.7453` rad) is the physically
  **closed** position and its **minimum** (`-0.1745` rad) is **open** — the
  opposite of what the joint's numeric range would suggest, and opposite of
  `schema.py`'s `GRIPPER_MIN`(=closed)/`GRIPPER_MAX`(=open) ordering. Found
  by measuring actual mesh-to-mesh proximity between the moving jaw and the
  fixed finger at both extremes (min distance shrinks toward the joint's
  max), not by assuming the naive mapping. `MuJoCoBackend` inverts the
  interpolation accordingly — see `_schema_to_joint_gripper`/
  `_joint_to_schema_gripper`. Any contributor swapping in a different
  calibration file (e.g. `so101_old_calib.xml`) must re-verify this before
  trusting the gripper in recorded data.
