#!/usr/bin/env python3
"""Prepare a downsampled TARE garage preview cloud aligned to AIR garage_v1."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
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
    parser.add_argument("--target-points", type=int, default=120000)
    parser.add_argument("--copy-original", action="store_true", default=True)
    parser.add_argument("--no-copy-original", action="store_false", dest="copy_original")
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


def read_binary_ply_downsample(path: Path, target_points: int) -> tuple[list[tuple[float, float, float]], dict]:
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
        stride = max(1, count // max(1, target_points))
        points = []
        raw_min = [float("inf")] * 3
        raw_max = [float("-inf")] * 3
        for i in range(count):
            raw = f.read(12)
            if len(raw) != 12:
                break
            x, y, z = struct.unpack("<fff", raw)
            vals = (x, y, z)
            for j, value in enumerate(vals):
                raw_min[j] = min(raw_min[j], value)
                raw_max[j] = max(raw_max[j], value)
            if i % stride == 0:
                points.append(vals)
        return points, {
            "source_vertex_count": count,
            "sample_stride": stride,
            "raw_bounds": {"min": raw_min, "max": raw_max},
        }


def transform(points: list[tuple[float, float, float]], xyz: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    return [(x + xyz[0], y + xyz[1], z + xyz[2]) for x, y, z in points]


def bounds(points: list[tuple[float, float, float]]) -> dict:
    mn = [min(p[i] for p in points) for i in range(3)]
    mx = [max(p[i] for p in points) for i in range(3)]
    return {"x_min": mn[0], "x_max": mx[0], "y_min": mn[1], "y_max": mx[1], "z_min": mn[2], "z_max": mx[2]}


def write_xyz(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w") as f:
        f.write("# TARE garage preview pointcloud downsampled and transformed to AIR garage_v1 map frame\n")
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
    original_out = args.out_dir / "garage_preview_pointcloud_original.ply"
    pcd_out = args.out_dir / "garage_preview_pointcloud_downsampled.pcd"
    xyz_out = args.out_dir / "garage_preview_pointcloud_downsampled.xyz"
    bounds_out = args.out_dir / "garage_preview_pointcloud_bounds.json"

    if args.copy_original and not original_out.exists():
        shutil.copy2(args.source, original_out)

    pose = garage_pose(args.world)
    sampled, meta = read_binary_ply_downsample(args.source, args.target_points)
    aligned = transform(sampled, pose)
    write_pcd(pcd_out, aligned)
    write_xyz(xyz_out, aligned)
    result = {
        "source": str(args.source),
        "copied_original": str(original_out) if original_out.exists() else None,
        "downsampled_pcd": str(pcd_out),
        "downsampled_xyz": str(xyz_out),
        "frame_id": "map",
        "point_count": len(aligned),
        "transform": {"x": pose[0], "y": pose[1], "z": pose[2], "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "scale": 1.0},
        "bounds": bounds(aligned),
        **meta,
    }
    bounds_out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
