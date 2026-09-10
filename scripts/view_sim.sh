#!/usr/bin/env bash
# Opens MuJoCo's interactive viewer on the SO-101 sim scene for manual
# inspection/control (joint + control sliders, drag the arm, watch physics).
# Needs a real GLFW/OpenGL display, so run this from your own terminal, not
# through remote automation. Don't set MUJOCO_GL=egl for this — that's for
# offscreen camera rendering (get_observation(), tests, CI) only.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

python -c "
from vla_edge_manipulation.backends.sim import MuJoCoBackend

backend = MuJoCoBackend()
backend.connect()
try:
    backend.launch_interactive_viewer()
finally:
    backend.disconnect()
"
