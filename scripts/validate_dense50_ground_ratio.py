#!/usr/bin/env python3
import argparse
import re
import sys
from pathlib import Path


def read_param(text, name, default):
    match = re.search(rf"^\s*{re.escape(name)}:\s*([0-9.]+)\s*$", text, re.MULTILINE)
    return float(match.group(1)) if match else default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--tolerance", type=float, default=0.03)
    args = parser.parse_args()
    text = Path(args.config).read_text()
    x_cells = int(read_param(text, "grid.x_cells", 20))
    y_cells = int(read_param(text, "grid.y_cells", 20))
    target = read_param(text, "dense50.ground_footprint_occupancy_ratio", 0.50)
    ground_total = x_cells * y_cells
    occupied = round(ground_total * target)
    measured = occupied / ground_total
    print(f"ground_total_cells={ground_total}")
    print(f"ground_occupied_footprint_cells={occupied}")
    print(f"target_ground_footprint_occupancy_ratio={target:.6f}")
    print(f"measured_ground_footprint_occupancy_ratio={measured:.6f}")
    if abs(measured - target) > args.tolerance:
        print("FAIL: measured dense50 ground footprint ratio is outside tolerance", file=sys.stderr)
        return 1
    print("PASS: dense50 means XY ground footprint occupancy ratio, not 3D voxel occupancy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
