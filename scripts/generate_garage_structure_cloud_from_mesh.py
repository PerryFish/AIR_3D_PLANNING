#!/usr/bin/env python3
"""Generate garage_v1 structure point clouds aligned to the Gazebo world pose."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARE_PLY = Path(
    "/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply"
)
DEFAULT_DAE = ROOT / "src/virtual_env/garage_v1/models/garage/meshes/garage.dae"
DEFAULT_WORLD = ROOT / "src/virtual_env/garage_v1/worlds/garage_v1.world"
DEFAULT_MAP_DIR = ROOT / "src/virtual_env/garage_v1/maps"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("tare", "mesh"), default="tare")
    parser.add_argument("--tare-ply", type=Path, default=DEFAULT_TARE_PLY)
    parser.add_argument("--dae", type=Path, default=DEFAULT_DAE)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--output-prefix", type=Path, default=DEFAULT_MAP_DIR / "garage_structure_from_tare")
    parser.add_argument("--target-points", type=int, default=160000)
    return parser.parse_args()


def garage_pose(world: Path) -> tuple[float, float, float]:
    root = ET.parse(world).getroot()
    for include in root.findall(".//include"):
        uri = include.findtext("uri", "")
        name = include.findtext("name", "")
        if uri.strip() == "model://garage" or name.strip() == "garage":
            raw = include.findtext("pose", "0 0 0 0 0 0").split()
            return (float(raw[0]), float(raw[1]), float(raw[2]))
    raise RuntimeError(f"garage include pose not found in {world}")


def read_tare_ply(path: Path, target_points: int) -> list[tuple[float, float, float]]:
    with path.open("rb") as f:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = f.readline()
            if not line:
                raise RuntimeError("PLY ended before end_header")
            header += line
        text = header.decode("ascii", errors="strict")
        if "format binary_little_endian 1.0" not in text:
            raise RuntimeError("only binary_little_endian PLY is supported")
        match = re.search(r"element vertex (\d+)", text)
        if not match:
            raise RuntimeError("PLY vertex count not found")
        count = int(match.group(1))
        stride = max(1, count // max(1, target_points))
        points = []
        for i in range(count):
            raw = f.read(12)
            if len(raw) != 12:
                break
            if i % stride == 0:
                points.append(struct.unpack("<fff", raw))
    return points


def read_dae_vertices(path: Path) -> list[tuple[float, float, float]]:
    text = path.read_text(errors="ignore")
    match = re.search(
        r'<float_array id="Garage-mesh-positions-array" count="\d+">(.*?)</float_array>',
        text,
        re.S,
    )
    if not match:
        raise RuntimeError("Garage mesh position float_array not found")
    values = [float(v) for v in match.group(1).split()]
    return list(zip(values[0::3], values[1::3], values[2::3]))


def densify_vertices(points: list[tuple[float, float, float]], target_points: int) -> list[tuple[float, float, float]]:
    if len(points) >= target_points:
        stride = max(1, len(points) // target_points)
        return points[::stride]
    out = list(points)
    step = max(1, int(math.sqrt(len(points))))
    for i in range(0, len(points) - step, step):
        a = points[i]
        b = points[i + step]
        for t in (0.25, 0.5, 0.75):
            out.append(tuple(a[j] * (1.0 - t) + b[j] * t for j in range(3)))
            if len(out) >= target_points:
                return out
    return out


def transform(points: list[tuple[float, float, float]], xyz: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    return [(x + xyz[0], y + xyz[1], z + xyz[2]) for x, y, z in points]


def bounds(points: list[tuple[float, float, float]]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return (
        tuple(min(p[i] for p in points) for i in range(3)),
        tuple(max(p[i] for p in points) for i in range(3)),
    )


def write_xyz(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# garage_v1 structure cloud aligned with garage_v1.world model://garage pose\n")
        for x, y, z in points:
            f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def main() -> None:
    args = parse_args()
    pose = garage_pose(args.world)
    if args.source == "tare":
        points = read_tare_ply(args.tare_ply, args.target_points)
    else:
        points = densify_vertices(read_dae_vertices(args.dae), args.target_points)
    aligned = transform(points, pose)
    write_xyz(args.output_prefix.with_suffix(".xyz"), aligned)
    write_pcd(args.output_prefix.with_suffix(".pcd"), aligned)
    mn, mx = bounds(aligned)
    print(f"source={args.source}")
    print(f"point_count={len(aligned)}")
    print(f"garage_pose={pose}")
    print(f"bounds_min={mn}")
    print(f"bounds_max={mx}")
    print(f"wrote={args.output_prefix.with_suffix('.xyz')}")
    print(f"wrote={args.output_prefix.with_suffix('.pcd')}")


if __name__ == "__main__":
    main()
