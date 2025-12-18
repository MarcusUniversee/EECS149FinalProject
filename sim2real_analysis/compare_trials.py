import argparse
from plot import plot_arc_errors
from typing import Dict, Tuple, List
import os
import numpy as np

INPUT_ROOT = "sim2real_results"

def main() -> None:
    metrics_list = []
    labels_list = []
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    args = ap.parse_args()
    for dirpath, dirnames, filenames in os.walk(INPUT_ROOT):
        folder_name = os.path.basename(dirpath)
        if args.name not in folder_name:
            continue

        npz_path = os.path.join(dirpath, "raw_metrics.npz")
        if os.path.isfile(npz_path):
            metrics = np.load(npz_path, allow_pickle=True)
            metrics_list.append(metrics["arc"].item())
            labels_list.append(folder_name)
    plot_arc_errors(os.path.join(INPUT_ROOT, "COMBINED_" + args.name), metrics_list, labels_list, args.name)

if __name__ == "__main__":
    main()