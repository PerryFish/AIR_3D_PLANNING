#!/usr/bin/env python3
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BOUNDS_PATH = ROOT / "src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud_bounds.json"
OUT_PATH = ROOT / "results/garage_v1/start_pose_analysis.json"

TARE_START = {"x": 0.0, "y": 0.0, "z": 0.75, "yaw": 0.0, "frame": "map"}
AIR_OLD_START = {"x": -13.5, "y": -13.5, "z": 1.6, "yaw": 0.0}


def load_bounds():
    meta = json.loads(BOUNDS_PATH.read_text())
    bounds = meta["edge_bounds"]
    transform = meta.get("transform", {"x": -23.817, "y": -46.018, "z": 0.073, "yaw": 0.0})
    return meta, bounds, transform


def center_core(bounds, point, ratio=0.30):
    cx = (bounds["x_min"] + bounds["x_max"]) * 0.5
    cy = (bounds["y_min"] + bounds["y_max"]) * 0.5
    hx = (bounds["x_max"] - bounds["x_min"]) * ratio * 0.5
    hy = (bounds["y_max"] - bounds["y_min"]) * ratio * 0.5
    return abs(point["x"] - cx) <= hx and abs(point["y"] - cy) <= hy


def in_bounds(bounds, point):
    return bounds["x_min"] <= point["x"] <= bounds["x_max"] and bounds["y_min"] <= point["y"] <= bounds["y_max"]


def normalize_candidate(name, point, bounds):
    cx = (bounds["x_min"] + bounds["x_max"]) * 0.5
    cy = (bounds["y_min"] + bounds["y_max"]) * 0.5
    out = dict(point)
    out.update(
        {
            "name": name,
            "inside_edge_bounds": in_bounds(bounds, point),
            "inside_center_core_30pct": center_core(bounds, point),
            "distance_to_bounds_center_xy": math.hypot(point["x"] - cx, point["y"] - cy),
        }
    )
    return out


def main():
    meta, bounds, transform = load_bounds()
    recommended = {
        "x": float(transform["x"] + TARE_START["x"]),
        "y": float(transform["y"] + TARE_START["y"]),
        "z": 1.6,
        "yaw": float(transform.get("yaw", 0.0) + TARE_START["yaw"]),
    }
    candidates = [
        normalize_candidate("tare_start_transformed_by_air_garage_pose", recommended, bounds),
        normalize_candidate("old_air_start", AIR_OLD_START, bounds),
        normalize_candidate(
            "bounds_center",
            {
                "x": (bounds["x_min"] + bounds["x_max"]) * 0.5,
                "y": (bounds["y_min"] + bounds["y_max"]) * 0.5,
                "z": 1.6,
                "yaw": 0.0,
            },
            bounds,
        ),
    ]
    result = {
        "method": "Use TARE garage vehicle start (0,0,0.75,yaw=0) transformed by AIR garage world pose from edge-cloud metadata.",
        "tare_start_pose": TARE_START,
        "air_garage_transform": transform,
        "edge_bounds": bounds,
        "old_air_start_pose": AIR_OLD_START,
        "recommended_start_pose": recommended,
        "recommended_start_region": "between the two opening regions / corridor entrance area",
        "recommended_inside_edge_bounds": in_bounds(bounds, recommended),
        "recommended_inside_center_core_30pct": center_core(bounds, recommended),
        "edge_point_count": meta.get("edge_point_count"),
        "surface_point_count": meta.get("surface_point_count"),
        "edge_density_ratio": meta.get("edge_density_ratio"),
        "candidates": candidates,
        "notes": [
            "The old AIR start is in the central part of the shifted garage frame and is not TARE-equivalent.",
            "The recommended pose preserves TARE's garage start relative to the garage model while keeping AIR altitude at 1.6 m.",
        ],
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result["recommended_start_pose"], sort_keys=True))
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
