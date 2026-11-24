"""Locate and import Spencer's DroneNavPPO package.

Spencer's files are the single source of truth for the observation contract.
This module only puts his directory on ``sys.path`` -- nothing there is copied,
modified or shadowed, so constants such as ``DT``, ``SPEED`` and ``RADIUS`` can
never silently drift out of sync with the trained policies.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

#: Directories searched for ``navigation.py``, in priority order.
_CANDIDATES = (
    "DroneNavPPO/DroneNavPPO",
    "DroneNavPPO",
    "../DroneNavPPO/DroneNavPPO",
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]      # .../webots_dns
WORKSPACE_ROOT = PROJECT_ROOT.parent                     # .../drone_navigation_system


def find_ppo_dir() -> Path:
    """Return the directory holding Spencer's ``navigation.py``.

    ``DNS_PPO_DIR`` overrides the search when the ZIP is extracted elsewhere.
    """
    override = os.environ.get("DNS_PPO_DIR")
    if override:
        path = Path(override).expanduser().resolve()
        if not (path / "navigation.py").is_file():
            raise FileNotFoundError(f"DNS_PPO_DIR={path} has no navigation.py")
        return path

    for relative in _CANDIDATES:
        path = (WORKSPACE_ROOT / relative).resolve()
        if (path / "navigation.py").is_file():
            return path
    raise FileNotFoundError(
        "Could not find Spencer's navigation.py. Looked under "
        + ", ".join(str((WORKSPACE_ROOT / c).resolve()) for c in _CANDIDATES)
        + ". Set DNS_PPO_DIR to the directory containing navigation.py."
    )


def install() -> Path:
    """Put Spencer's directory on ``sys.path`` and return it. Idempotent."""
    path = find_ppo_dir()
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)
    return path


install()

# Re-exported so the rest of the package never hard-codes these numbers.
from navigation import (          # noqa: E402  (import must follow path setup)
    DT,
    GOAL_TOL,
    HORIZON,
    OBS_DIM,
    RADIUS,
    RANGE,
    SCHEMA,
    SECTORS,
    SPEED,
    Frame,
    Navigation,
    observation,
    pack_scan,
    validate,
)

__all__ = [
    "DT", "GOAL_TOL", "HORIZON", "OBS_DIM", "RADIUS", "RANGE", "SCHEMA",
    "SECTORS", "SPEED", "Frame", "Navigation", "observation", "pack_scan",
    "validate", "find_ppo_dir", "install", "PROJECT_ROOT", "WORKSPACE_ROOT",
]
