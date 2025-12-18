import argparse
from plot import plot_arc_errors
from typing import Dict, Tuple, List
import os
import numpy as np
from plot import calc_gap
import csv
from collections import defaultdict

INPUT_ROOT = "sim2real_results"
OUTPUT = "sim2real_results/gap_table.csv"

def main() -> None:
    gaps_by_name: Dict[str, Dict[int, float]] = defaultdict(dict)
    # ap = argparse.ArgumentParser()
    # ap.add_argument("--name", required=True)
    # args = ap.parse_args()
    for dirpath, dirnames, filenames in os.walk(INPUT_ROOT):
        folder_name = os.path.basename(dirpath)
        if "COMBINED" in folder_name:
            continue

        npz_path = os.path.join(dirpath, "raw_metrics.npz")
        if not os.path.isfile(npz_path):
            continue

        # Expect folder_name like "log1_RAMP_A"
        try:
            prefix, name = folder_name.split("_", 1)  # "log1", "RAMP_A"
            trial_idx = int(prefix.replace("log", ""))  # 1, 2, 3, ...
        except ValueError:
            # Skip folders that don't match the pattern
            continue

        metrics = np.load(npz_path, allow_pickle=True)
        arc = metrics["arc"].item()
        if not arc.get("ok", False):
            continue

        u = arc["_u"]
        pos_err = arc["_pos_err"]

        gap, _ = calc_gap(pos_err, u)
        gaps_by_name[name][trial_idx] = gap

    # Build rows: one per unique name
    rows = []
    for name, trials in gaps_by_name.items():
        row = {
            "name": name,
            "trial1": trials.get(1, ""),
            "trial2": trials.get(2, ""),
            "trial3": trials.get(3, ""),
        }
        rows.append(row)

    fieldnames = ["name", "trial1", "trial2", "trial3"]
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    with open(OUTPUT, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

if __name__ == "__main__":
    main()