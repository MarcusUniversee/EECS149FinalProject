import math
import numpy as np
from typing import Dict, Tuple

def wrap_to_pi(a: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

def unwrap_angle(a: np.ndarray) -> np.ndarray:
    """Unwrap angles for differentiation (continuous)."""
    return np.unwrap(a)

def safe_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    dy = np.full_like(y, np.nan, dtype=float)
    if y.size < 2:
        return dy

    # ensure strictly increasing time for derivative
    if np.any(np.diff(t) <= 0):
        order = np.argsort(t, kind="mergesort")
        y2 = y[order]
        t2 = t[order]
        dy2 = safe_diff(y2, t2)
        dy[order] = dy2
        return dy

    if y.size == 2:
        dy[:] = (y[1] - y[0]) / (t[1] - t[0])
        return dy

    dy[1:-1] = (y[2:] - y[:-2]) / (t[2:] - t[:-2])
    dy[0] = (y[1] - y[0]) / (t[1] - t[0])
    dy[-1] = (y[-1] - y[-2]) / (t[-1] - t[-2])
    return dy

def cumulative_arc_length(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx * dx + dy * dy)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    return s


def interp_linear_with_nan(x: np.ndarray, y: np.ndarray, x_new: np.ndarray) -> np.ndarray:
    """
    Linear interpolation on x (must be increasing).
    Returns NaN outside bounds.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x_new = np.asarray(x_new, dtype=float)
    out = np.interp(x_new, x, y, left=np.nan, right=np.nan)
    out[(x_new < x[0]) | (x_new > x[-1])] = np.nan
    return out

def rms(a: np.ndarray) -> float:
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(a * a)))


def mean(a: np.ndarray) -> float:
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.mean(a))


def maxabs(a: np.ndarray) -> float:
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.max(np.abs(a)))


def nanmax(a: np.ndarray) -> float:
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.max(a))

def speed_and_yawrate(run_r: Dict[str, np.ndarray], t: np.ndarray) -> Dict[str, np.ndarray]:
    vx = safe_diff(run_r["x"], t)
    vy = safe_diff(run_r["y"], t)
    v = np.sqrt(vx * vx + vy * vy)
    yaw_rate = safe_diff(run_r["yaw"], t)
    return {"v": v, "yaw_rate": yaw_rate}

def desired_heading(run_r: Dict[str, np.ndarray]) -> np.ndarray:
    dx = run_r["desired_x"] - run_r["x"]
    dy = run_r["desired_y"] - run_r["y"]
    return np.arctan2(dy, dx)


def xcorr_delay(a: np.ndarray, b: np.ndarray, dt: float, max_lag_s: float) -> Tuple[float, float]:
    """
    Normalized cross-correlation delay estimate between signals a and b.
    Positive lag => b lags a (responds later).
    """
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    a = a[m]
    b = b[m]
    if a.size < 20:
        return (float("nan"), float("nan"))

    a = a - np.mean(a)
    b = b - np.mean(b)
    sa = np.std(a)
    sb = np.std(b)
    if sa < 1e-9 or sb < 1e-9:
        return (float("nan"), float("nan"))
    a = a / sa
    b = b / sb

    max_lag = int(round(max_lag_s / dt))
    corr_full = np.correlate(a, b, mode="full")
    lags_full = np.arange(-len(b) + 1, len(a))
    center = len(corr_full) // 2

    i0 = max(center - max_lag, 0)
    i1 = min(center + max_lag + 1, corr_full.size)
    corr = corr_full[i0:i1]
    lags = lags_full[i0:i1]

    k = int(np.argmax(corr))
    best_lag = float(lags[k] * dt)
    peak = float(corr[k] / len(a))  # comparable scalar; not strict Pearson
    return best_lag, peak

