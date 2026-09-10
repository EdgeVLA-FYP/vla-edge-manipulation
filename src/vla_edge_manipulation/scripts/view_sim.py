"""Interactive MuJoCo viewer for the SO-101 sim scene. Installed as `vla-sim-view`.

Opens a window with joint/control sliders for manually posing the arm and
watching physics — needs a real GLFW/OpenGL display, so run this from your
own terminal, not through scripted automation. Close the window to exit.
"""

from __future__ import annotations

import argparse

from vla_edge_manipulation.backends.sim import MuJoCoBackend


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Connect and validate the scene, then exit without opening a window.",
    )
    args = parser.parse_args()

    backend = MuJoCoBackend()
    backend.connect()
    try:
        if args.dry_run:
            print("Scene loaded and validated OK.")
            return
        print("Opening the interactive viewer — close the window to exit.")
        backend.launch_interactive_viewer()
    finally:
        backend.disconnect()


if __name__ == "__main__":
    main()
