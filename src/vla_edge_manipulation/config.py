"""Shared YAML config loading. Every `configs/*.yaml.example` template has a
gitignored, machine-specific `configs/*.yaml` copy — loaded and validated
against its own template the same way, regardless of which module needs it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_config_path(name: str) -> Path:
    """`configs/{name}.yaml` — the gitignored copy of `{name}.yaml.example`."""
    return repo_root() / "configs" / f"{name}.yaml"


def _check_required_keys(
    config: dict[str, Any],
    required: dict[str, Any],
    path: Path,
    example_path: Path,
    prefix: str = "",
) -> None:
    """Recursively checks config has every key `required` (the parsed *.yaml.example) declares."""
    for key, example_value in required.items():
        full_key = f"{prefix}{key}"
        if key not in config:
            raise KeyError(f"{path}: missing {full_key!r} — see {example_path.name} for all fields")
        if isinstance(example_value, dict):
            _check_required_keys(
                config[key], example_value, path, example_path, prefix=f"{full_key}."
            )


def load_config(path: Path) -> dict[str, Any]:
    """Loads `path`, validated against the repo's canonical `{path.stem}.yaml.example`
    template — regardless of where `path` itself lives (e.g. a test's tmp_path copy)."""
    example_path = repo_root() / "configs" / f"{path.stem}.yaml.example"
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found — copy {example_path.name} to {path.name} and adjust it"
        )
    with path.open() as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict):
        raise ValueError(f"{path}: expected a YAML mapping, got {type(config).__name__}")

    with example_path.open() as f:
        example_config = yaml.safe_load(f)
    _check_required_keys(config, example_config, path, example_path)
    return config
