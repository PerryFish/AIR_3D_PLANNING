#!/usr/bin/env python3
"""Extract an edge/outline cloud from the TARE garage preview point cloud."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PLY = Path(
    "/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply"
)
WORLD = ROOT / "src/virtual_env/garage_v1/worlds/garage_v1.world"
OUT_DIR = ROOT / "src/virtual_env/garage_v1/maps/tare_reference"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE_PLY)
    parser.add_argument("--world", type=Path, default=WORLD)
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--xy-resolution", type=float, default=0.45)
    parser.add_argument("--z-resolution", type=float, default=0.45)
    parser.add_argument("--min-z", type=float, default=-0.2)
    parser.add_argument("--max-z", type=float, default=5.0)
    parser.add_argument("--max-points", type=int, default=80000)
    return parser.parse_args()


def garage_pose(world: Path) -> tuple[float, float, float]:
    root = ET.parse(world).getroot()
    for include in root.findall(".//include"):
        uri = include.findtext("uri", "").strip()
        name = include.findtext("name", "").strip()
        if uri == "model://garage" or name == "garage":
            vals = include.findtext("pose", "0 0 0 0 0 0").split()
            return float(vals[0]), float(vals[1]), float(vals[2])
    return 0.0, 0.0, 0.0


def read_binary_ply(path: Path) -> list[tuple[float, float, float]]:
    with path.open("rb") as f:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = f.readline()
            if not line:
                raise RuntimeError("PLY ended before end_header")
            header += line
        text = header.decode("ascii", errors="strict")
        if "format binary_little_endian 1.0" not in text:
            raise RuntimeError("Only binary_little_endian PLY is supported")
        match = re.search(r"element vertex (\d+)", text)
        if not match:
            raise RuntimeError("Could not parse PLY vertex count")
        count = int(match.group(1))
        points = []
        for _ in range(count):
            raw = f.read(12)
            if len(raw) != 12:
                break
            points.append(struct.unpack("<fff", raw))
        return points


def align(points: list[tuple[float, float, float]], pose: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    return [(x + pose[0], y + pose[1], z + pose[2]) for x, y, z in points]


def bounds(points: list[tuple[float, float, float]]) -> dict[str, float]:
    return {
        "x_min": min(p[0] for p in points),
        "x_max": max(p[0] for p in points),
        "y_min": min(p[1] for p in points),
        "y_max": max(p[1] for p in points),
        "z_min": min(p[2] for p in points),
        "z_max": max(p[2] for p in points),
    }


def extract_edges(
    points: list[tuple[float, float, float]],
    xy_res: float,
    z_res: float,
    min_z: float,
    max_z: float,
    max_points: int,
) -> tuple[list[tuple[float, float, float]], dict]:
    b = bounds(points)
    x0 = math.floor(b["x_min"] / xy_res) * xy_res
    y0 = math.floor(b["y_min"] / xy_res) * xy_res
    z0 = math.floor(b["z_min"] / z_res) * z_res
    cells: dict[tuple[int, int, int], list[tuple[float, float, float]]] = {}
    for p in points:
        x, y, z = p
        if z < min_z or z > max_z:
            continue
        ix = int(math.floor((x - x0) / xy_res))
        iy = int(math.floor((y - y0) / xy_res))
        iz = int(math.floor((z - z0) / z_res))
        cells.setdefault((ix, iy, iz), []).append(p)

    occupied = set(cells)
    edge_cells = []
    for cell in occupied:
        ix, iy, iz = cell
        is_edge = False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            if (ix + dx, iy + dy, iz) not in occupied:
                is_edge = True
                break
        if is_edge:
            edge_cells.append(cell)

    edge_cells.sort(key=lambda c: (c[2], c[0], c[1]))
    edge_points = []
    for ix, iy, iz in edge_cells:
        pts = cells[(ix, iy, iz)]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        edge_points.append((cx, cy, cz))

    if len(edge_points) > max_points:
        stride = math.ceil(len(edge_points) / max_points)
        edge_points = edge_points[::stride]

    metrics = {
        "xy_resolution": xy_res,
        "z_resolution": z_res,
        "occupied_cells": len(occupied),
        "edge_cells": len(edge_cells),
        "cell_edge_ratio": len(edge_cells) / max(1, len(occupied)),
        "filtered_z_min": min_z,
        "filtered_z_max": max_z,
    }
    return edge_points, metrics


def write_xyz(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w") as f:
        f.write("# TARE garage edge/outline cloud transformed to AIR garage_v1 map frame\n")
        for x, y, z in points:
            f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    source_points = read_binary_ply(args.source)
    pose = garage_pose(args.world)
    aligned = align(source_points, pose)
    edge_points, metrics = extract_edges(
        aligned,
        args.xy_resolution,
        args.z_resolution,
        args.min_z,
        args.max_z,
        args.max_points,
    )
    pcd = args.out_dir / "garage_edge_cloud.pcd"
    xyz = args.out_dir / "garage_edge_cloud.xyz"
    meta_path = args.out_dir / "garage_edge_cloud_bounds.json"
    write_pcd(pcd, edge_points)
    write_xyz(xyz, edge_points)
    result = {
        "source": str(args.source),
        "frame_id": "map",
        "method": "z-sliced XY occupancy boundary from TARE preview pointcloud",
        "surface_point_count": len(source_points),
        "edge_point_count": len(edge_points),
        "edge_density_ratio": len(edge_points) / max(1, len(source_points)),
        "transform": {"x": pose[0], "y": pose[1], "z": pose[2], "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "surface_bounds": bounds(aligned),
        "edge_bounds": bounds(edge_points),
        "outputs": {"pcd": str(pcd), "xyz": str(xyz)},
        **metrics,
    }
    meta_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
