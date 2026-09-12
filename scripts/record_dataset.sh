#!/usr/bin/env bash
# Records PickPlaceController demonstrations into LeRobotDataset format and
# validates them (issue #8). See docs/decisions.md for the fix history this
# depends on and docs/sessions.md for past recording sessions.
#
# Usage: bash scripts/record_dataset.sh --episodes N --repo-id ORG/NAME [--push] [--root DIR]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

MUJOCO_GL=egl python -m vla_edge_manipulation.recording "$@"
