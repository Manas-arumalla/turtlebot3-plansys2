"""Locate packaged PDDL files both in installed and source-tree layouts."""

from __future__ import annotations

import os


def pddl_dir() -> str:
    """Return the directory containing the packaged PDDL files.

    Prefers the installed share directory (via ament_index) and falls back to
    the source tree so the pure-Python tools work without a ROS install.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('washbot_planning')
        candidate = os.path.join(share, 'pddl')
        if os.path.isdir(candidate):
            return candidate
    except Exception:  # noqa: BLE001 - any failure means "not installed"
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'pddl')


def find_pddl(name: str) -> str:
    """Resolve a PDDL file name (e.g. ``domain_strips.pddl``) to a full path."""
    base = pddl_dir()
    for candidate in (os.path.join(base, name),
                      os.path.join(base, 'problems', name)):
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f'PDDL file "{name}" not found under {base}')


def read_pddl(name: str) -> str:
    with open(find_pddl(name), 'r', encoding='utf-8') as handle:
        return handle.read()
