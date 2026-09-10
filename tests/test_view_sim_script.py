"""Smoke test for the vla-sim-view console-script entry point
(src/vla_edge_manipulation/scripts/view_sim.py) — catches the entry point
itself breaking (argparse wiring, MuJoCoBackend constructor drift), not the
interactive viewer window, which needs a real display and isn't testable here.
"""

import pytest

pytest.importorskip("mujoco")
pytest.importorskip("yaml")

from vla_edge_manipulation.scripts.view_sim import main  # noqa: E402


def test_dry_run_connects_and_exits_without_opening_a_window(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["vla-sim-view", "--dry-run"])
    main()
    assert "validated OK" in capsys.readouterr().out
