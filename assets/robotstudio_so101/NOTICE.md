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
  element inside the `wrist_camera` body — upstream models the camera's
  physical housing but does not itself define a MuJoCo camera sensor. The
  mesh doesn't expose which face is the lens, so the mount pose was chosen
  empirically (rendered all 6 axis-aligned candidates, picked the one that
  frames the gripper) — treat it as a best-effort default, not a verified
  optical calibration. Re-check against the real wrist camera once hardware
  arrives.
- `meshes/*.stl`: unmodified.

## Not vendored from upstream

`so101.xml` (this directory) is authored by this project, not upstream — it
includes the vendored robot file and adds the floor/lighting/`front` camera
that `scene.xml` in the upstream repo provides for its own demo, since we
need our own camera placement to match `schema.CAMERA_KEYS`.
