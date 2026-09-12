# Source

Vendored from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
`Simulation/SO101/`, commit `eecbe3e0a9ebb23e25ad7b2759b03884c6660903`.

Licensed under Apache License 2.0 (see `LICENSE` in this directory). The
robot matches the real arm this project targets — 6-DOF, joint names
`shoulder_pan`/`shoulder_lift`/`elbow_flex`/`wrist_flex`/`wrist_roll`/`gripper`,
identical to `schema.JOINT_NAMES` — sold by Seeed Studio as the "LeRobot
SO-ARM101" kit.

Files taken: `so101_new_calib_camera.xml` (the camera-equipped variant, new
calibration) and its referenced mesh assets. `so101_old_calib.xml`,
`so101_new_calib.xml` (no-camera variant), and the `.urdf` files were not
needed and are not vendored.

## Local modifications (Apache 2.0 §4b)

- `so101_new_calib_camera.xml`: `<compiler meshdir>` changed from `assets`
  to `meshes` to match this directory's layout. Added a `<camera name="wrist">`
  inside the `wrist_camera` body — upstream models the camera housing but
  not a MuJoCo camera sensor. Mount pose chosen empirically (best framing of
  6 axis-aligned candidates), not a verified optical calibration — re-check
  against the real wrist camera once hardware arrives.
- `so101_new_calib_camera.xml`: added two small supplementary collision boxes (`fixed_jaw_pad`, `moving_jaw_pad`), one at each jaw's fingertip, with higher-friction material and stiff contact parameters (`solref`/`solimp`) than the bare mesh. The vendored jaws are thin, near-pointed tips with a very large single-jaw sweep; without a pad, IK-driven grasp attempts on a small cube only ever produced a glancing top-corner contact that couldn't survive a lift (see docs/decisions.md, which also covers the MuJoCo contact-solver settings in `so101.xml` this pairs with). Pad size/position (2.5mm box) matches a published, tested SO-101-in-MuJoCo grasping setup (github.com/ggand0/pick-101), not a guess. This represents a real, common practical modification (rubber grip pads) a builder would add before running pick-place on the real gripper — not a purely simulated shortcut — but it has **not** been validated against real hardware; re-check pad size/position/friction once the physical gripper is available, or remove this modification if real grip pads aren't fitted.
- `meshes/*.stl`: unmodified.

## Not vendored from upstream

`so101.xml` (this directory) is authored by this project, not upstream — it
includes the vendored robot file and adds the floor/lighting/`front` camera
that `scene.xml` in the upstream repo provides for its own demo, since we
need our own camera placement to match `schema.CAMERA_KEYS`.
