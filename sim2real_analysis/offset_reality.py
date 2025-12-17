import pandas as pd
import numpy as np
import os

# ---- config ----
INPUT_ROOT  = "data"          # directory with CSVs
OUTPUT_ROOT = "corrected_data"
SUFFIX     = ""

DX_BODY = 0.03   # meters (tag -> robot front)
DY_BODY = 0.0

yaw_in_degrees = True  # set False if yaw already in radians

YAW_IN_DEGREES = True        # set False if yaw already radians
X_COL, Y_COL, YAW_COL = "x", "y", "yaw"
T_COL = "t"
os.makedirs(OUTPUT_ROOT, exist_ok=True)


# ---- load ----
for root, _, files in os.walk(INPUT_ROOT):
    for fname in files:
        if "sim" in fname.lower() or not fname.lower().endswith(".csv"):
            continue
        in_path = os.path.join(root, fname)

        # preserve subfolder structure
        rel_dir = os.path.relpath(root, INPUT_ROOT)
        out_dir = os.path.join(OUTPUT_ROOT, rel_dir)
        os.makedirs(out_dir, exist_ok=True)

        df = pd.read_csv(in_path)

        # sanity check
        required = {X_COL, Y_COL, YAW_COL, T_COL}
        if not required.issubset(df.columns):
            print(f"Skipping {in_path}: missing required columns")
            continue

        t0 = df[T_COL].iloc[0]
        df[T_COL] = df[T_COL] - t0

        yaw = df[YAW_COL].to_numpy()
        if YAW_IN_DEGREES:
            yaw = np.deg2rad(yaw)

        c = np.cos(yaw)
        s = np.sin(yaw)

        df["x"] = df[X_COL] + c * DX_BODY - s * DY_BODY
        df["y"] = df[Y_COL] + s * DX_BODY + c * DY_BODY

        base = os.path.splitext(fname)[0]
        out_path = os.path.join(out_dir, base + SUFFIX + ".csv")

        df.to_csv(out_path, index=False)
        print(f"Wrote {out_path}")