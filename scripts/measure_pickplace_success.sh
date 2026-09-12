#!/usr/bin/env bash
# Runs PickPlaceController across N randomized episodes and reports the
# measured grasp+place success rate. See docs/decisions.md (2026-09-12
# entries) for why this is currently near 0% and what's been ruled out.
#
# Usage: bash scripts/measure_pickplace_success.sh [N]  (default N=25)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

N="${1:-25}"

MUJOCO_GL=egl python -c "
import sys
from vla_edge_manipulation.backends.sim import MuJoCoBackend
from vla_edge_manipulation.controllers.pick_place_controller import PickPlaceController

n = int(sys.argv[1])
backend = MuJoCoBackend(seed=42)
backend.connect()
try:
    controller = PickPlaceController(backend)
    successes = sum(controller.run_episode() for _ in range(n))
    print(f'{successes}/{n} = {successes / n * 100:.0f}% success')
finally:
    backend.disconnect()
" "$N"
