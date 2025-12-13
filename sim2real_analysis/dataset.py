import numpy as np
from dataclasses import dataclass
import pandas as pd
from utils import wrap_to_pi

REQUIRED_COLS = ["t", "x", "y", "z", "roll", "pitch", "yaw", "desired_x", "desired_y"]
@dataclass
class RunData:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    desired_x: np.ndarray
    desired_y: np.ndarray

def load_csv(path: str) -> RunData:
    df = pd.read_csv(path)
    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")

    # sort by time, drop duplicate timestamps (keep last)
    df = df.sort_values("t", kind="mergesort").drop_duplicates(subset=["t"], keep="last")

    t = df["t"].to_numpy(float)
    x = df["x"].to_numpy(float)
    y = df["y"].to_numpy(float)
    yaw_raw = df["yaw"].to_numpy(float)
    desired_x = df["desired_x"].to_numpy(float)
    desired_y = df["desired_y"].to_numpy(float)

    yaw = wrap_to_pi(np.deg2rad(yaw_raw))

    return RunData(t=t, x=x, y=y, yaw=yaw, desired_x=desired_x, desired_y=desired_y)