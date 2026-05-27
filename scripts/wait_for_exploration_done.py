#!/usr/bin/env python3
import argparse
import csv
import time
from pathlib import Path


def latest_row(csv_path):
    if not csv_path.exists():
        return None
    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="results/metrics_dense50.csv")
    parser.add_argument("--summary", default="results/test_summary.md")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--coverage-threshold", type=float, default=0.93)
    args = parser.parse_args()
    csv_path = Path(args.csv)
    start = time.time()
    row = None
    while time.time() - start < args.timeout:
        row = latest_row(csv_path)
        if row:
            coverage = float(row["coverage"])
            done = row["done"] == "True"
            if coverage >= args.coverage_threshold and done:
                write_summary(Path(args.summary), row, True)
                print(f"PASS: final_coverage={coverage:.6f} done=True")
                return 0
        time.sleep(0.5)
    if row:
        print(f"FAIL: timeout final_coverage={row['coverage']} done={row['done']}")
        write_summary(Path(args.summary), row, False)
    else:
        print("FAIL: timeout waiting for metrics CSV rows")
        write_summary(Path(args.summary), {}, False)
    return 1


def write_summary(path, row, passed):
    path.parent.mkdir(parents=True, exist_ok=True)
    coverage = row.get("coverage", "0.0")
    done = row.get("done", "False")
    failed_goals = row.get("failed_goals", "unknown")
    stuck_events = row.get("stuck_events", "unknown")
    ground_ratio = row.get("ground_footprint_occupancy_ratio", "unknown")
    path.write_text(
        "\n".join(
            [
                "# Dense50 Aerial Exploration Test Summary",
                "",
                f"- passed: {passed}",
                f"- final_coverage: {coverage}",
                f"- done: {done}",
                f"- failed_goals: {failed_goals}",
                f"- stuck_events: {stuck_events}",
                f"- measured_ground_footprint_occupancy_ratio: {ground_ratio}",
                "- coverage_rule: done is true only when coverage >= 0.93",
                "- dense50_definition: XY ground obstacle footprint occupancy ratio, not 3D voxel occupancy",
                "",
            ]
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
