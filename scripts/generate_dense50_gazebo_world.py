#!/usr/bin/env python3
from pathlib import Path


X_CELLS = 20
Y_CELLS = 20
RESOLUTION = 1.0
TARGET_RATIO = 0.50


def occupied_cells():
    protected = {(0, 0), (1, 0), (0, 1), (19, 19), (18, 19), (19, 18)}
    corridor_keep_free = set()
    for i in range(X_CELLS):
        if i % 4 != 0:
            corridor_keep_free.add((i, i))
        if i % 5 != 0:
            corridor_keep_free.add((X_CELLS - 1 - i, i))
    cells = []
    for iy in range(Y_CELLS):
        for ix in range(X_CELLS):
            if (ix, iy) in protected or (ix, iy) in corridor_keep_free:
                continue
            if (ix + 2 * iy) % 4 in (0, 1):
                cells.append((ix, iy))
            if len(cells) >= round(X_CELLS * Y_CELLS * TARGET_RATIO):
                return cells
    for iy in range(Y_CELLS):
        for ix in range(X_CELLS):
            if (ix, iy) in protected or (ix, iy) in corridor_keep_free or (ix, iy) in cells:
                continue
            cells.append((ix, iy))
            if len(cells) >= round(X_CELLS * Y_CELLS * TARGET_RATIO):
                return cells
    return cells


def world_xy(ix, iy):
    return ((ix - X_CELLS / 2) * RESOLUTION + 0.5, (iy - Y_CELLS / 2) * RESOLUTION + 0.5)


def model(ix, iy, n):
    x, y = world_xy(ix, iy)
    height = 0.7 + ((ix * 3 + iy * 5) % 5) * 0.45
    sx = 0.82 if n % 7 else 1.6
    sy = 0.82 if n % 11 else 1.45
    if n % 13 == 0:
        sx, sy = 2.4, 0.35
    name = f"dense50_obstacle_{n:03d}"
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {height/2:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box></geometry>
          <material><ambient>0.65 0.16 0.08 1</ambient><diffuse>0.85 0.22 0.10 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def make_world(path):
    cells = occupied_cells()
    ratio = len(cells) / (X_CELLS * Y_CELLS)
    models = "\n".join(model(ix, iy, i) for i, (ix, iy) in enumerate(cells))
    text = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='dense50_ground_footprint'>
    <include><uri>model://sun</uri></include>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>24 24 0.05</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>24 24 0.05</size></box></geometry>
          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material>
        </visual>
      </link>
    </model>
{models}
  </world>
</sdf>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    print(f"ground_total_cells={X_CELLS * Y_CELLS}")
    print(f"ground_occupied_footprint_cells={len(cells)}")
    print(f"measured_ground_footprint_occupancy_ratio={ratio:.6f}")
    if abs(ratio - TARGET_RATIO) > 0.03:
        raise SystemExit("dense50 footprint ratio outside tolerance")


if __name__ == "__main__":
    out = Path("worlds/dense50_ground_footprint.world")
    make_world(out)
    share = Path("src/aerial_exploration_planner/worlds/dense50_ground_footprint.world")
    share.write_text(out.read_text())
