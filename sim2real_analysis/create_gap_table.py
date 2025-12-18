import argparse
from plot import plot_arc_errors
from typing import Dict, Tuple, List
import os
import numpy as np
from plot import calc_gap
import csv
from collections import defaultdict
import matplotlib.pyplot as plt

INPUT_ROOT = "sim2real_results"
OUTPUT = "sim2real_results/gap_table.csv"
import re

def format_name(s: str) -> str:
    pattern = r"^(.*?)(?:_with)?_max0_(\d+(?:_\d+)?)_min0_(\d+(?:_\d+)?)$"
    m = re.match(pattern, s)
    if not m:
        print("format does not match")
        return s  # fallback if format doesn't match

    name, maxv, minv = m.groups()

    # Replace underscores with spaces in name
    name = name.replace("_", " ")

    # Convert underscore decimals: 3_5 -> 0.35
    maxf = "0." + maxv.replace("_", "")
    minf = "0." + minv.replace("_", "")

    return f"{name} [{minf}, {maxf}]"

def plot_gaps(rows: List[Dict[str, object]], save_path: str = "sim2real_results/gap_table.png") -> None:
    names = [format_name(r["name"]) for r in rows]

    # Convert blanks -> NaN so matplotlib skips those bars cleanly
    def to_float(x):
        return float(x) if x != "" else np.nan

    t1 = np.array([to_float(r["trial1"]) for r in rows], dtype=float)
    t2 = np.array([to_float(r["trial2"]) for r in rows], dtype=float)
    t3 = np.array([to_float(r["trial3"]) for r in rows], dtype=float)

    x = np.arange(len(names))
    w = 0.28

    plt.figure(figsize=(max(10, 0.45 * len(names)), 6))
    plt.bar(x - w, t1, width=w, label="trial1")
    plt.bar(x,      t2, width=w, label="trial2")
    plt.bar(x + w,  t3, width=w, label="trial3")

    plt.xticks(x, names, rotation=90)
    plt.ylabel("sim2real_gap")
    plt.title("Sim2Real Gap by Trial")
    plt.legend()
    plt.ylim(0.0, 0.15)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.show()

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
    plot_gaps(rows)

if __name__ == "__main__":
    main()