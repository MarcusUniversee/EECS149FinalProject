import numpy as np
from dataset import RunData, load_csv
from utils import cumulative_arc_length, interp_linear_with_nan, unwrap_angle, wrap_to_pi
from utils import mean, rms, nanmax, maxabs
from utils import speed_and_yawrate, xcorr_delay, desired_heading, safe_diff
from typing import Dict, Tuple
import os
import json
import argparse
from plot import save_plots

def arclength_align(sim: RunData, real: RunData, n: int = 2000) -> Dict:
    """
    Reparameterizes everything by arc length for calculations. this makes
    everything completely independent of time, and only dependent on the path.

    we should use this as our main sim2real gap
    """
    #first calculate an array of accumulating arc length (cumsum of differences)
    s_sim = cumulative_arc_length(sim.x, sim.y)
    s_real = cumulative_arc_length(real.x, real.y)

    #get the final cumulative sum
    L_sim = float(s_sim[-1]) if s_sim.size else 0.0
    L_real = float(s_real[-1]) if s_real.size else 0.0
    if L_sim <= 1e-9 or L_real <= 1e-9:
        return {"ok": False, "reason": "degenerate trajectory length", "L_sim": L_sim, "L_real": L_real}

    #normalize the arc lengths from 0 to 1
    u_sim = s_sim / L_sim
    u_real = s_real / L_real
    #create sample indices
    u = np.linspace(0.0, 1.0, n)

    #Get the positions at the intervals
    #samples the normalized arc lengths n times evenly and gets the x/y values at those points
    #these values may have variyng time intervals, but there is a fixed number of samples for x/y
    sim_xu = interp_linear_with_nan(u_sim, sim.x, u)
    sim_yu = interp_linear_with_nan(u_sim, sim.y, u)
    real_xu = interp_linear_with_nan(u_real, real.x, u)
    real_yu = interp_linear_with_nan(u_real, real.y, u)

    #get the total euclidean error from these x/y values
    pos_err = np.sqrt((sim_xu - real_xu) ** 2 + (sim_yu - real_yu) ** 2)

    # Yaw: unwrap before interpolating for proper differentiation. The sampling is the same as the positions
    # basically unwrapping makes it so that adjacent values are never above pi, so like -170 -> 170
    # will end up having a difference of 20 with the new array
    sim_yaw_u = interp_linear_with_nan(u_sim, unwrap_angle(sim.yaw), u)
    real_yaw_u = interp_linear_with_nan(u_real, unwrap_angle(real.yaw), u)
    #make it go back to (-pi, pi)
    yaw_err = wrap_to_pi(sim_yaw_u - real_yaw_u)

    # Endpoint (time-end) errors
    final_pos_err = float(np.hypot(sim.x[-1] - real.x[-1], sim.y[-1] - real.y[-1]))
    final_yaw_err = float(abs(wrap_to_pi(unwrap_angle(sim.yaw)[-1] - unwrap_angle(real.yaw)[-1])))

    return {
        "ok": True,
        "L_sim_m": L_sim,
        "L_real_m": L_real,
        "pos_err_mean_m": mean(pos_err),
        "pos_err_rms_m": rms(pos_err),
        "pos_err_max_m": nanmax(pos_err),
        "yaw_err_mean_rad": mean(yaw_err),
        "yaw_err_rms_rad": rms(yaw_err),
        "yaw_err_max_rad": maxabs(yaw_err),
        "final_pos_err_m": final_pos_err,
        "final_yaw_err_rad": final_yaw_err,
        # arrays for plotting
        "_u": u,
        "_sim_xu": sim_xu, "_sim_yu": sim_yu,
        "_real_xu": real_xu, "_real_yu": real_yu,
        "_pos_err": pos_err, "_yaw_err": yaw_err,
    }

def closest_point_metrics(sim: RunData, real: RunData, time_diff: float = 1, yaw_weight_m_per_rad: float = 0) -> Dict:
    """
    For each real point, compute min Euclidean distance to any sim point within
    time_diff time
    This is point-set distance (orthogonal in the "closest point" sense),
    not requiring monotonic motion or any single direction.

    if fraction missing is large, you should probably incrase the time_diff

    we should use this for secondary analysis, as it does not take into account yaw
    and can compare multiple points to the same point
    """
    real_t = np.asarray(real.t, float)
    real_x = np.asarray(real.x, float)
    real_y = np.asarray(real.y, float)
    real_yaw = np.asarray(real.yaw, float)

    sim_t = np.asarray(sim.t, float)
    sim_x = np.asarray(sim.x, float)
    sim_y = np.asarray(sim.y, float)
    sim_yaw = np.asarray(sim.yaw, float)
    if real_t.size == 0 or sim_t.size == 0:
        return {"ok": False, "reason": "empty sim or real data"}

    # Ensure sim is sorted by time (should already be, but enforce)
    order = np.argsort(sim_t, kind="mergesort")
    sim_t = sim_t[order]
    sim_x = sim_x[order]
    sim_y = sim_y[order]
    sim_yaw = sim_yaw[order]

    n = real_t.shape[0]
    d_weighted = np.full((n,), np.nan, dtype=float)
    d_pos = np.full((n,), np.nan, dtype=float)
    yaw_err_abs = np.full((n,), np.nan, dtype=float)  # |wrap(yaw_err)| at argmin
    match_sim_index = np.full((n,), -1, dtype=int)
    for i in range(real_t.shape[0]):
        t0 = real_t[i] - time_diff
        t1 = real_t[i] + time_diff

        j0 = np.searchsorted(sim_t, t0, side="left")
        j1 = np.searchsorted(sim_t, t1, side="right")

        if j1 <= j0:
            continue  # no sim samples in the time window

        dx = sim_x[j0:j1] - real_x[i]
        dy = sim_y[j0:j1] - real_y[i]
        pos = np.sqrt(dx * dx + dy * dy)

        yaw_err = wrap_to_pi(sim_yaw[j0:j1] - real_yaw[i])
        yaw_term = yaw_weight_m_per_rad * yaw_err

        d2 = pos * pos + yaw_term * yaw_term
        k = int(np.argmin(d2))  # index within window slice

        d_weighted[i] = float(np.sqrt(d2[k]))
        d_pos[i] = float(pos[k])
        yaw_err_abs[i] = float(abs(yaw_err[k]))
        match_sim_index[i] = int(j0 + k)
    return {
        "ok": True,
        "time_diff_s": float(time_diff),
        "yaw_weight_m_per_rad": float(yaw_weight_m_per_rad),
        "weighted_dist_mean": mean(d_weighted),
        "weighted_dist_rms": rms(d_weighted),
        "weighted_dist_max": nanmax(d_weighted),
        "pos_dist_mean": mean(d_pos),
        "pos_dist_rms": rms(d_pos),
        "pos_dist_max": nanmax(d_pos),
        "yaw_err_mean_rad": mean(yaw_err_abs),
        "yaw_err_rms_rad": rms(yaw_err_abs),
        "yaw_err_max_rad": nanmax(yaw_err_abs),
        "fraction_missing": float(np.mean(~np.isfinite(d_weighted))),
        "_weighted_dist": d_weighted,
        "_pos_dist": d_pos,
        "_yaw_err_abs": yaw_err_abs,
        "_match_sim_index": match_sim_index,
    }
    
def resample_overlap(sim: RunData, real: RunData, dt: float) -> Dict:
    """
    Resample both runs onto uniform time grid over the overlap window.
    Requires only that time is increasing; motion can be arbitrary.

    So if real run went from 1 to 6, and the sim run went from 0 to 4,
    we would just take the data from 1 to 4
    """
    t0 = max(sim.t[0], real.t[0])
    t1 = min(sim.t[-1], real.t[-1])
    if t1 <= t0 + 5 * dt:
        return {"ok": False, "reason": "insufficient overlap"}

    t = np.arange(t0, t1, dt)

    # yaw: unwrap before interpolation so derivative is meaningful
    sim_yaw_u = unwrap_angle(sim.yaw)
    real_yaw_u = unwrap_angle(real.yaw)

    sim_r = {
        "x": interp_linear_with_nan(sim.t, sim.x, t),
        "y": interp_linear_with_nan(sim.t, sim.y, t),
        "yaw": interp_linear_with_nan(sim.t, sim_yaw_u, t),
        "desired_x": interp_linear_with_nan(sim.t, sim.desired_x, t),
        "desired_y": interp_linear_with_nan(sim.t, sim.desired_y, t),
    }
    real_r = {
        "x": interp_linear_with_nan(real.t, real.x, t),
        "y": interp_linear_with_nan(real.t, real.y, t),
        "yaw": interp_linear_with_nan(real.t, real_yaw_u, t),
        "desired_x": interp_linear_with_nan(real.t, real.desired_x, t),
        "desired_y": interp_linear_with_nan(real.t, real.desired_y, t),
    }

    return {"ok": True, "t": t, "sim": sim_r, "real": real_r}

def timing_dynamics_metrics(sim: RunData, real: RunData, dt: float, max_lag_s: float) -> Dict:
    """
    This gives us diagnostic data based on absolute time and discards all other time
    We mainly just want the latency, speed, and control response, since these
    require using absolute time rather than parameterized by something else
    """
    rs = resample_overlap(sim, real, dt)
    if not rs["ok"]:
        return {"ok": False, "reason": rs.get("reason", "resample failed")}

    t = rs["t"]
    sim_r = rs["sim"]
    real_r = rs["real"]

    sim_kin = speed_and_yawrate(sim_r, t)
    real_kin = speed_and_yawrate(real_r, t)

    v_err = sim_kin["v"] - real_kin["v"]
    yaw_rate_err = sim_kin["yaw_rate"] - real_kin["yaw_rate"]

    # within-run "command intent": desired heading rate
    sim_dh = unwrap_angle(desired_heading(sim_r))
    real_dh = unwrap_angle(desired_heading(real_r))
    sim_dh_rate = safe_diff(sim_dh, t)
    real_dh_rate = safe_diff(real_dh, t)

    sim_delay, sim_peak = xcorr_delay(sim_dh_rate, sim_kin["yaw_rate"], dt, max_lag_s)
    real_delay, real_peak = xcorr_delay(real_dh_rate, real_kin["yaw_rate"], dt, max_lag_s)

    sim_duration = float(sim.t[-1] - sim.t[0])
    real_duration = float(real.t[-1] - real.t[0])

    return {
        "ok": True,
        "overlap_start_t": float(t[0]),
        "overlap_end_t": float(t[-1]),
        "dt_s": float(dt),
        # timing (supporting)
        "sim_duration_s": sim_duration,
        "real_duration_s": real_duration,
        "duration_ratio_real_over_sim": real_duration / sim_duration if sim_duration > 1e-9 else float("nan"),
        # dynamics (supporting)
        "speed_mean_sim_mps": mean(sim_kin["v"]),
        "speed_mean_real_mps": mean(real_kin["v"]),
        "speed_rms_error_mps": rms(v_err),
        "speed_max_error_mps": maxabs(v_err),
        "yaw_rate_rms_error_radps": rms(yaw_rate_err),
        "yaw_rate_max_error_radps": maxabs(yaw_rate_err),
        # delay (supporting)
        "sim_cmd_to_yawrate_delay_s": sim_delay,
        "sim_cmd_to_yawrate_corr_peak": sim_peak,
        "real_cmd_to_yawrate_delay_s": real_delay,
        "real_cmd_to_yawrate_corr_peak": real_peak,
        # arrays for plotting
        "_t": t,
        "_sim_v": sim_kin["v"],
        "_real_v": real_kin["v"],
        "_v_err": v_err,
        "_sim_yaw_rate": sim_kin["yaw_rate"],
        "_real_yaw_rate": real_kin["yaw_rate"],
        "_yaw_rate_err": yaw_rate_err,
    }

def normalized_time_align(sim: RunData, real: RunData, n: int = 2000) -> Dict:
    """
    Compare trajectories vs normalized time tau in [0,1]:
      tau = (t - t0) / (t1 - t0)

    Returns tau-aligned position and yaw errors (RMS/mean/max) + arrays for plotting.
    This is NOT for dynamics (no velocities/delays), only for phase-of-execution comparison.

    Basically if real goes from 0 to 7, and sim goes from 0 to 5, we normalize both of them
    so that it is 0 to 1.
    """
    # durations
    sim_T = float(sim.t[-1] - sim.t[0])
    real_T = float(real.t[-1] - real.t[0])
    if sim_T <= 1e-9 or real_T <= 1e-9:
        return {"ok": False, "reason": "degenerate duration", "sim_T": sim_T, "real_T": real_T}

    # normalized time axes
    sim_tau = (sim.t - sim.t[0]) / sim_T
    real_tau = (real.t - real.t[0]) / real_T
    tau = np.linspace(0.0, 1.0, n)

    # x,y interpolation
    sim_x = interp_linear_with_nan(sim_tau, sim.x, tau)
    sim_y = interp_linear_with_nan(sim_tau, sim.y, tau)
    real_x = interp_linear_with_nan(real_tau, real.x, tau)
    real_y = interp_linear_with_nan(real_tau, real.y, tau)
    pos_err = np.sqrt((sim_x - real_x) ** 2 + (sim_y - real_y) ** 2)

    # yaw interpolation (unwrap first)
    sim_yaw = interp_linear_with_nan(sim_tau, unwrap_angle(sim.yaw), tau)
    real_yaw = interp_linear_with_nan(real_tau, unwrap_angle(real.yaw), tau)
    yaw_err = wrap_to_pi(sim_yaw - real_yaw)

    return {
        "ok": True,
        "sim_duration_s": sim_T,
        "real_duration_s": real_T,
        "pos_err_mean_m": mean(pos_err),
        "pos_err_rms_m": rms(pos_err),
        "pos_err_max_m": nanmax(pos_err),
        "yaw_err_mean_rad": mean(yaw_err),
        "yaw_err_rms_rad": rms(yaw_err),
        "yaw_err_max_rad": maxabs(yaw_err),
        # arrays for plotting
        "_tau": tau,
        "_pos_err": pos_err,
        "_yaw_err": yaw_err,
        "_sim_x": sim_x, "_sim_y": sim_y,
        "_real_x": real_x, "_real_y": real_y,
    }

def strip_arrays(d: Dict) -> Dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            continue
        out[k] = v
    return out

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True)
    ap.add_argument("--real", required=True)
    ap.add_argument("--out", default="sim2real_out")

    #sample rate for normalized comparisons
    ap.add_argument("--n", type=int, default=2000) 

    #interval between samples for timing metrics
    ap.add_argument("--dt", type=float, default=0.02) 

    #yaw weight for closest pt metric
    #0 means yaw not considered. 1 means difference in yaw contributes equally with x and y
    ap.add_argument("--yw", type=float, default=0.0) 

    # for analyzing time dependent metrics, the time window to search over
    ap.add_argument("--max_lag", type=float, default=1.0)

    args = ap.parse_args()

    sim = load_csv(args.sim)
    real = load_csv(args.real)

    arc = arclength_align(sim, real, n=args.n)
    cp = closest_point_metrics(sim, real, time_diff=args.max_lag, yaw_weight_m_per_rad=args.yw)
    dyn = timing_dynamics_metrics(sim, real, dt=args.dt, max_lag_s=args.max_lag)
    tau = normalized_time_align(sim, real, n=args.n)
    rs = resample_overlap(sim, real, dt=args.dt)
    save_plots(args.out, sim, real, arc, cp, rs, dyn, tau)

    sim2real_gap_m = (
        float(np.nanmax(arc["_pos_err"]))
        if arc.get("ok", False) and "_pos_err" in arc
        else float("nan")
    )

    metrics = {
        "inputs": {
            "sim_csv": args.sim,
            "real_csv": args.real
        },

        "sim2real_gap_m": sim2real_gap_m,

        "arc_length_alignment": strip_arrays(arc),
        "closest_point_distance": strip_arrays(cp),
        "resample_overlap": strip_arrays(rs),
        "timing_and_dynamics": strip_arrays(dyn),
        "normalized_time_alignment": strip_arrays(tau),

        "plots_dir": args.out,
    }
    try:
        s = json.dumps(metrics, indent=2)
        print(s)

        with open(os.path.join(args.out, "results.json"), "w", encoding="utf-8") as f:
            f.write(s + "\n")
    except Exception as e:
        print(repr(e))
    
    np.savez(
        os.path.join(args.out, "raw_metrics.npz"),
        arc=arc,
        cp=cp,
        rs=rs,
        dyn=dyn,
        tau=tau
    )


if __name__ == "__main__":
    main()