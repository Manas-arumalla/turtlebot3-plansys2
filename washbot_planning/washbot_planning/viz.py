"""Render the navigation map with the waypoint graph overlaid.

Produces the world figures used in the documentation directly from the real
artifacts: the SLAM map (PGM + YAML) and the executor's world model YAML.
matplotlib is a dev-only dependency, imported lazily.
"""

from __future__ import annotations

import os
from typing import Dict, List, Tuple

import yaml


def read_pgm(path: str) -> Tuple[int, int, int, bytes]:
    """Minimal binary PGM (P5) reader: returns (width, height, maxval, pixels)."""
    with open(path, 'rb') as handle:
        data = handle.read()

    fields: List[bytes] = []
    index = 0
    while len(fields) < 4 and index < len(data):
        # Skip whitespace and comment lines in the header.
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            while index < len(data) and data[index] != 0x0A:
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        fields.append(data[start:index])
    if fields[0] != b'P5':
        raise ValueError(f'{path} is not a binary PGM (P5) file')
    width, height, maxval = int(fields[1]), int(fields[2]), int(fields[3])
    pixels = data[index + 1:index + 1 + width * height]
    return width, height, maxval, pixels


def load_world(world_yaml: str) -> Dict:
    with open(world_yaml, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    # The executor's config nests the world under ros__parameters-style keys
    # when loaded as a ROS param file; accept both layouts.
    if 'world' in config:
        return config['world']
    for value in config.values():
        if isinstance(value, dict) and 'world' in value:
            return value['world']
    raise ValueError(f'no "world" section found in {world_yaml}')


def render_world(map_yaml: str, world_yaml: str, out_path: str,
                 title: str = 'WashBot world model',
                 path_csv: str = '') -> str:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with open(map_yaml, 'r', encoding='utf-8') as handle:
        map_meta = yaml.safe_load(handle)
    image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)),
                              map_meta['image'])
    width, height, maxval, pixels = read_pgm(image_path)
    resolution = float(map_meta['resolution'])
    origin_x, origin_y = float(map_meta['origin'][0]), float(map_meta['origin'][1])

    # Map extent in world coordinates (row 0 of the PGM is the top of the map).
    extent = (origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution)
    grid = [
        [pixels[row * width + col] / maxval for col in range(width)]
        for row in range(height)
    ]

    world = load_world(world_yaml)
    locations: Dict[str, Dict] = world['locations']
    edges: List[List[str]] = world.get('edges', [])
    charger = world.get('charger', '')

    fig, ax = plt.subplots(figsize=(7.2, 6.6), dpi=150)
    ax.imshow(grid, cmap='gray', origin='upper', extent=extent,
              vmin=0.0, vmax=1.0, interpolation='nearest')

    for a, b in edges:
        pa, pb = locations[a], locations[b]
        ax.plot([pa['x'], pb['x']], [pa['y'], pb['y']],
                color='#1f77b4', linewidth=1.6, alpha=0.85, zorder=2)

    for name, pose in locations.items():
        dirty = bool(pose.get('dirty', False)) or bool(pose.get('deep_dirty', False))
        if name == charger:
            color, label_color = '#2ca02c', '#1a6b1a'
        elif dirty:
            color, label_color = '#d62728', '#8f1b1c'
        else:
            color, label_color = '#1f77b4', '#12507e'
        ax.scatter([pose['x']], [pose['y']], s=110, color=color, zorder=3,
                   edgecolors='white', linewidths=1.2)
        ax.annotate(name, (pose['x'], pose['y']),
                    textcoords='offset points', xytext=(8, 8),
                    fontsize=9, fontweight='bold', color=label_color, zorder=4)

    handles = [
        plt.Line2D([], [], marker='o', linestyle='', color='#2ca02c',
                   label='dock / charger'),
        plt.Line2D([], [], marker='o', linestyle='', color='#d62728',
                   label='cleanable fixture'),
        plt.Line2D([], [], marker='o', linestyle='', color='#1f77b4',
                   label='waypoint'),
        plt.Line2D([], [], color='#1f77b4', label='traversable edge'),
    ]

    # Optional overlay: an executed trajectory recorded by pose_recorder.
    if path_csv:
        times, xs, ys = [], [], []
        with open(path_csv, 'r', encoding='utf-8') as handle:
            next(handle)  # header
            for line in handle:
                t, x, y = line.strip().split(',')
                times.append(float(t))
                xs.append(float(x))
                ys.append(float(y))
        if xs:
            scatter = ax.scatter(xs, ys, c=times, cmap='viridis', s=7,
                                 zorder=5, linewidths=0)
            colorbar = fig.colorbar(scatter, ax=ax, shrink=0.75, pad=0.02)
            colorbar.set_label('mission time (s)')
            handles.append(plt.Line2D([], [], marker='o', linestyle='',
                                      color='#3b528b', label='executed path'))
    ax.legend(handles=handles, loc='upper right', fontsize=8, frameon=True)
    ax.set_title(title)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path
