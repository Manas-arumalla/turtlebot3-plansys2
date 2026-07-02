"""Matplotlib plots for benchmark CSVs (matplotlib is a dev-only dependency)."""

from __future__ import annotations

import csv
import os
import statistics
from typing import Dict, List

_SERIES_STYLE = {
    'internal-gbfs': {'color': '#1f77b4', 'marker': 'o'},
    'internal-astar': {'color': '#d62728', 'marker': 's'},
    'popf': {'color': '#2ca02c', 'marker': '^'},
}


def _load(csv_path: str) -> List[Dict]:
    with open(csv_path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _median_by_backend(rows: List[Dict], value_key: str,
                       solved_only: bool = True) -> Dict[str, List[tuple]]:
    groups: Dict[tuple, List[float]] = {}
    for row in rows:
        if solved_only and row['solved'] != '1':
            continue
        key = (row['backend'], int(row['locations']))
        groups.setdefault(key, []).append(float(row[value_key]))
    series: Dict[str, List[tuple]] = {}
    for (backend, locations), values in sorted(groups.items()):
        series.setdefault(backend, []).append((locations, statistics.median(values)))
    return series


def _style(backend: str) -> Dict:
    return _SERIES_STYLE.get(backend, {'marker': 'd'})


def plot_all(csv_path: str, out_dir: str) -> List[str]:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rows = _load(csv_path)
    os.makedirs(out_dir, exist_ok=True)
    written: List[str] = []

    figures = [
        ('wall_time_s', 'Median planning time (s)', 'benchmark_time.png', True),
        ('plan_length', 'Median plan length (actions)', 'benchmark_plan_length.png', False),
        ('expanded', 'Median states expanded', 'benchmark_expanded.png', True),
    ]
    for value_key, ylabel, filename, log_scale in figures:
        series = _median_by_backend(rows, value_key)
        fig, ax = plt.subplots(figsize=(6.4, 4.2), dpi=140)
        for backend, points in series.items():
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            ax.plot(xs, ys, label=backend, linewidth=1.8, markersize=5,
                    **_style(backend))
        if log_scale:
            ax.set_yscale('log')
        ax.set_xlabel('World size (locations)')
        ax.set_ylabel(ylabel)
        ax.grid(True, which='both', alpha=0.3)
        ax.legend(frameon=False)
        fig.tight_layout()
        path = os.path.join(out_dir, filename)
        fig.savefig(path)
        plt.close(fig)
        written.append(path)
    return written
