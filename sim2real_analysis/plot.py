import numpy as np
import matplotlib.pyplot as plt
import os
from dataset import RunData
from typing import Dict

def ensure_dir(d: str) -> None:
    os.makedirs(d, exist_ok=True)

# =========================
# Helper plotters (one per dict)
# Each helper is responsible for:
#  - generating its plots
#  - calling note(fname, desc)
# =========================

def plot_raw_overlay(out_dir: str, sim: RunData, real: RunData, note) -> None:
    # Raw XY overlay
    plt.figure()
    plt.plot(sim.x, sim.y, label="sim")
    plt.plot(real.x, real.y, label="real")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory overlay (raw)")
    plt.legend()
    plt.tight_layout()
    fname = "traj_overlay_raw.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Raw XY paths (no alignment). Quick sanity check for gross offsets, scale issues, or coordinate-frame mismatch.")


def plot_arc(out_dir: str, arc: Dict, note) -> None:
    if not arc.get("ok", False):
        return

    u = arc["_u"]
    pos_err = arc["_pos_err"]
    yaw_err = arc["_yaw_err"]

    sim2real_gap_m = float(np.nanmax(pos_err))
    gap_u = float(u[int(np.nanargmax(pos_err))]) if np.any(np.isfinite(pos_err)) else float("nan")

    # 1) Arc-length aligned overlay
    plt.figure()
    plt.plot(arc["_sim_xu"], arc["_sim_yu"], label="sim (arc aligned)")
    plt.plot(arc["_real_xu"], arc["_real_yu"], label="real (arc aligned)")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory overlay (arc-length aligned)")
    plt.legend()
    plt.tight_layout()
    fname = "traj_overlay_arclength.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "XY overlay after arc-length alignment (removes pacing differences). Use to judge geometric similarity of paths.")

    # 2) Error vs arc-length + annotate gap
    plt.figure()
    plt.plot(u, pos_err, label="pos error")
    if np.isfinite(gap_u):
        plt.axvline(gap_u, linestyle="--", label=f"gap at u={gap_u:.3f}")
    plt.xlabel("normalized arc length u")
    plt.ylabel("pos error (m)")
    plt.title(f"Position error vs arc length (Sim2Real gap = {sim2real_gap_m:.4f} m)")
    plt.legend()
    plt.tight_layout()
    fname = "pos_err_vs_arclength_gap.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(
        fname,
        "Primary sim2real plot: pointwise XY deviation vs normalized arc length u. "
        "Sim2Real gap is computed as max(pos_err(u)) and annotated by a vertical line.",
    )

    # 3) Yaw error vs arc-length
    plt.figure()
    plt.plot(u, yaw_err)
    plt.xlabel("normalized arc length u")
    plt.ylabel("yaw error (rad)")
    plt.title("Yaw error vs arc length")
    plt.tight_layout()
    fname = "yaw_err_vs_arclength.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Yaw mismatch vs normalized arc length (geometric alignment). Helps separate heading disagreement from path deviation.")

    # 4) Error distribution (hist + CDF) for pos error
    pe = pos_err[np.isfinite(pos_err)]
    if pe.size > 0:
        plt.figure()
        plt.hist(pe, bins=60)
        plt.xlabel("pos error (m)")
        plt.ylabel("count")
        plt.title("Arc-length pos error histogram")
        plt.tight_layout()
        fname = "pos_err_arclength_hist.png"
        plt.savefig(os.path.join(out_dir, fname), dpi=200)
        plt.close()
        note(fname, "Distribution of arc-length position errors. Shows whether the gap is a rare spike or broadly elevated.")

        plt.figure()
        pe_sorted = np.sort(pe)
        cdf = np.linspace(0.0, 1.0, pe_sorted.size)
        plt.plot(pe_sorted, cdf)
        plt.xlabel("pos error (m)")
        plt.ylabel("CDF")
        plt.title("Arc-length pos error CDF")
        plt.tight_layout()
        fname = "pos_err_arclength_cdf.png"
        plt.savefig(os.path.join(out_dir, fname), dpi=200)
        plt.close()
        note(fname, "CDF of arc-length position error. Read percentiles (e.g., 90% of the path is below X meters error).")

    # 5) Spatial localization: overlay points colored by error magnitude (scatter)
    sx, sy = arc["_sim_xu"], arc["_sim_yu"]
    rx, ry = arc["_real_xu"], arc["_real_yu"]
    m = np.isfinite(sx) & np.isfinite(sy) & np.isfinite(pos_err)
    if np.any(m):
        plt.figure()
        plt.plot(rx, ry, label="real (arc aligned)")
        sc = plt.scatter(sx[m], sy[m], c=pos_err[m], s=8, label="sim (colored by pos err)")
        plt.axis("equal")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("Arc-length aligned: error localized along sim path")
        plt.legend()
        plt.colorbar(sc, label="pos error (m)")
        plt.tight_layout()
        fname = "traj_arclength_error_colored.png"
        plt.savefig(os.path.join(out_dir, fname), dpi=200)
        plt.close()
        note(fname, "Arc-length aligned path with error magnitude shown spatially (where the mismatch is happening).")


def plot_cp(out_dir: str, cp: Dict, note) -> None:
    if not cp.get("ok", False):
        return

    plt.figure()
    plt.plot(cp["_weighted_dist"])
    plt.xlabel("real sample index")
    plt.ylabel("min weighted distance (m)")
    plt.title(
        f"Closest-point (time-windowed): weighted dist  "
        f"(Δt={cp['time_diff_s']}s, yw={cp['yaw_weight_m_per_rad']} m/rad)"
    )
    plt.tight_layout()
    fname = "closest_point_weighted_distance.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Secondary metric: for each real sample, best-matching sim sample within ±Δt window using weighted (XY+yaw) distance.")

    plt.figure()
    plt.plot(cp["_pos_dist"])
    plt.xlabel("real sample index")
    plt.ylabel("pos distance at match (m)")
    plt.title("Closest-point: position distance at matched sim sample")
    plt.tight_layout()
    fname = "closest_point_pos_distance.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Closest-point diagnostic: XY-only distance at the selected match (useful even when yaw weight > 0).")

    plt.figure()
    plt.plot(cp["_yaw_err_abs"])
    plt.xlabel("real sample index")
    plt.ylabel("|yaw error| at match (rad)")
    plt.title("Closest-point: yaw error at matched sim sample")
    plt.tight_layout()
    fname = "closest_point_yaw_error.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Closest-point diagnostic: absolute yaw error at the selected match (does not imply causal timing).")

    w = cp["_weighted_dist"]
    missing = ~np.isfinite(w)
    plt.figure()
    plt.plot(missing.astype(float))
    plt.xlabel("real sample index")
    plt.ylabel("missing match (0/1)")
    plt.title(f"Closest-point: missing matches (fraction={cp['fraction_missing']:.3f})")
    plt.tight_layout()
    fname = "closest_point_missing_matches.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Indicates where no sim samples existed within ±Δt. If frequent, increase time window or check logging rates.")


def plot_rs(out_dir: str, rs: Dict, note) -> None:
    if not rs.get("ok", False):
        return

    t = rs["t"] - rs["t"][0]
    sim_r = rs["sim"]
    real_r = rs["real"]

    plt.figure()
    plt.plot(sim_r["x"], sim_r["y"], label="sim (overlap)")
    plt.plot(real_r["x"], real_r["y"], label="real (overlap)")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory overlay (time overlap window)")
    plt.legend()
    plt.tight_layout()
    fname = "traj_overlay_overlap.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "XY overlay restricted to the absolute-time overlap interval used for dynamics comparisons (prevents extrapolation).")

    plt.figure()
    plt.plot(t, sim_r["yaw"], label="sim yaw (unwrapped)")
    plt.plot(t, real_r["yaw"], label="real yaw (unwrapped)")
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("yaw (rad, unwrapped)")
    plt.title("Yaw over overlap window")
    plt.legend()
    plt.tight_layout()
    fname = "yaw_overlap.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Unwrapped yaw over the overlap interval (absolute-time aligned). Useful to see drift/bias beyond yaw-rate alone.")

    pos_err_t = np.sqrt((sim_r["x"] - real_r["x"]) ** 2 + (sim_r["y"] - real_r["y"]) ** 2)
    plt.figure()
    plt.plot(t, pos_err_t)
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("pos error (m)")
    plt.title("Position error over overlap window (time-aligned)")
    plt.tight_layout()
    fname = "pos_err_overlap_time.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Time-aligned XY error over overlap. Interpretable only when sim/real clocks are roughly comparable.")


def plot_dyn(out_dir: str, dyn: Dict, note) -> None:
    if not dyn.get("ok", False):
        return

    t = dyn["_t"] - dyn["_t"][0]

    plt.figure()
    plt.plot(t, dyn["_sim_v"], label="sim")
    plt.plot(t, dyn["_real_v"], label="real")
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("speed (m/s)")
    plt.title("Speed profile (time-aligned overlap)")
    plt.legend()
    plt.tight_layout()
    fname = "speed_profile.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Speed vs time over overlap interval. Differences indicate pacing/dynamics mismatch (not geometric mismatch).")

    plt.figure()
    plt.plot(t, dyn["_sim_yaw_rate"], label="sim")
    plt.plot(t, dyn["_real_yaw_rate"], label="real")
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("yaw rate (rad/s)")
    plt.title("Yaw-rate profile (time-aligned overlap)")
    plt.legend()
    plt.tight_layout()
    fname = "yaw_rate_profile.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Yaw-rate vs time over overlap interval. Used for response-lag and turning-dynamics comparisons.")

    plt.figure()
    plt.plot(t, dyn["_v_err"])
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("speed error (sim - real) (m/s)")
    plt.title("Speed error")
    plt.tight_layout()
    fname = "speed_error.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Pointwise speed error over overlap. Indicates where the real system speeds up/slows down relative to sim.")

    plt.figure()
    plt.plot(t, dyn["_yaw_rate_err"])
    plt.xlabel("time (s) [overlap]")
    plt.ylabel("yaw-rate error (sim - real) (rad/s)")
    plt.title("Yaw-rate error")
    plt.tight_layout()
    fname = "yaw_rate_error.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Pointwise yaw-rate error over overlap. Highlights turning-rate mismatch and potential saturation/friction effects.")

    ve = dyn["_v_err"]
    ye = dyn["_yaw_rate_err"]
    ve = ve[np.isfinite(ve)]
    ye = ye[np.isfinite(ye)]

    if ve.size > 0:
        plt.figure()
        plt.hist(ve, bins=60)
        plt.xlabel("speed error (m/s)")
        plt.ylabel("count")
        plt.title("Speed error histogram")
        plt.tight_layout()
        fname = "speed_error_hist.png"
        plt.savefig(os.path.join(out_dir, fname), dpi=200)
        plt.close()
        note(fname, "Distribution of speed errors over overlap. Useful summary when time series are noisy.")

    if ye.size > 0:
        plt.figure()
        plt.hist(ye, bins=60)
        plt.xlabel("yaw-rate error (rad/s)")
        plt.ylabel("count")
        plt.title("Yaw-rate error histogram")
        plt.tight_layout()
        fname = "yaw_rate_error_hist.png"
        plt.savefig(os.path.join(out_dir, fname), dpi=200)
        plt.close()
        note(fname, "Distribution of yaw-rate errors over overlap. Useful summary of turning-dynamics mismatch.")


def plot_tau(out_dir: str, tau: Dict, note) -> None:
    if not tau.get("ok", False):
        return

    tt = tau["_tau"]

    plt.figure()
    plt.plot(tt, tau["_pos_err"])
    plt.xlabel("normalized time τ")
    plt.ylabel("pos error (m)")
    plt.title("Position error vs normalized time")
    plt.tight_layout()
    fname = "pos_err_vs_norm_time.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Phase-of-execution comparison: XY error vs normalized time (0→1). Highlights early/mid/late differences regardless of absolute duration.")

    plt.figure()
    plt.plot(tt, tau["_yaw_err"])
    plt.xlabel("normalized time τ")
    plt.ylabel("yaw error (rad)")
    plt.title("Yaw error vs normalized time")
    plt.tight_layout()
    fname = "yaw_err_vs_norm_time.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "Phase-of-execution yaw mismatch vs normalized time (0→1). Not a dynamics/latency metric; it is an execution-phase metric.")

    plt.figure()
    plt.plot(tau["_sim_x"], tau["_sim_y"], label="sim (τ-aligned)")
    plt.plot(tau["_real_x"], tau["_real_y"], label="real (τ-aligned)")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory overlay (normalized time aligned)")
    plt.legend()
    plt.tight_layout()
    fname = "traj_overlay_norm_time.png"
    plt.savefig(os.path.join(out_dir, fname), dpi=200)
    plt.close()
    note(fname, "XY overlay after normalized-time alignment. Useful to see how execution-phase alignment differs from arc-length alignment.")


# =========================
# Main function (unchanged behavior; now delegates)
# =========================

def save_plots(
    out_dir: str,
    sim: RunData,
    real: RunData,
    arc: Dict,
    cp: Dict,
    rs: Dict,
    dyn: Dict,
    tau: Dict,
) -> None:
    """
    Same plots as before, plus a non-intrusive description log written to:
      <out_dir>/PLOT_DESCRIPTIONS.md

    The descriptions do NOT overlay the plots (so they don't block data).
    """
    ensure_dir(out_dir)

    plot_notes = []  # list of (filename, description)

    def note(fname: str, desc: str) -> None:
        plot_notes.append((fname, desc))

    def write_notes() -> None:
        md_path = os.path.join(out_dir, "PLOT_DESCRIPTIONS.md")
        with open(md_path, "w", encoding="utf-8") as f:
            f.write("# Plot descriptions\n\n")
            f.write("This file documents what each saved plot is intended to show.\n\n")
            for fname, desc in plot_notes:
                f.write(f"- **{fname}**: {desc}\n")

    # Delegate to helpers (same behavior as original blocks)
    plot_raw_overlay(out_dir, sim, real, note)
    plot_arc(out_dir, arc, note)
    plot_cp(out_dir, cp, note)
    plot_rs(out_dir, rs, note)
    plot_dyn(out_dir, dyn, note)
    plot_tau(out_dir, tau, note)

    # Write the descriptions file last
    #write_notes()