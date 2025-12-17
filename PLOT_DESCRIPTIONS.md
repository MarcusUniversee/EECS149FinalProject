# Plot descriptions

## Primary metrics
- **pos_err_vs_arclength_gap.png**: Primary sim2real plot: pointwise XY deviation vs normalized arc length u. Sim2Real gap is computed as max(pos_err(u)) and annotated by a vertical line.
- **traj_overlay_arclength.png**: XY overlay after arc-length alignment (removes pacing differences). Use to judge geometric similarity of paths.

## Context metrics
- **pos_err_arclength_cdf.png**: CDF of arc-length position error. Read percentiles (e.g., 90% of the path is below X meters error).
- **traj_arclength_error_colored.png**: Arc-length aligned path with error magnitude shown spatially (where the mismatch is happening).

## Secondary
- **closest_point_weighted_distance.png**: Secondary metric: for each real sample, best-matching sim sample within ±Δt window using weighted (XY+yaw) distance.

## Supporting context

- **speed_profile.png**: Speed vs time over overlap interval. Differences indicate pacing/dynamics mismatch (not geometric mismatch).
- **yaw_rate_profile.png**: Yaw-rate vs time over overlap interval. Used for response-lag and turning-dynamics comparisons.
- **speed_error.png**: Pointwise speed error over overlap. Indicates where the real system speeds up/slows down relative to sim.
- **yaw_rate_error.png**: Pointwise yaw-rate error over overlap. Highlights turning-rate mismatch and potential saturation/friction effects.

## Other

- **traj_overlay_raw.png**: Raw XY paths (no alignment). Quick sanity check for gross offsets, scale issues, or coordinate-frame mismatch.

- **yaw_err_vs_arclength.png**: Yaw mismatch vs normalized arc length (geometric alignment). Helps separate heading disagreement from path deviation.
- **pos_err_arclength_hist.png**: Distribution of arc-length position errors. Shows whether the gap is a rare spike or broadly elevated.
- **closest_point_pos_distance.png**: Closest-point diagnostic: XY-only distance at the selected match (useful even when yaw weight > 0).
- **closest_point_yaw_error.png**: Closest-point diagnostic: absolute yaw error at the selected match (does not imply causal timing).
- **closest_point_missing_matches.png**: Indicates where no sim samples existed within ±Δt. If frequent, increase time window or check logging rates.
- **traj_overlay_overlap.png**: XY overlay restricted to the absolute-time overlap interval used for dynamics comparisons (prevents extrapolation).
- **yaw_overlap.png**: Unwrapped yaw over the overlap interval (absolute-time aligned). Useful to see drift/bias beyond yaw-rate alone.
- **pos_err_overlap_time.png**: Time-aligned XY error over overlap. Interpretable only when sim/real clocks are roughly comparable.
- **speed_error_hist.png**: Distribution of speed errors over overlap. Useful summary when time series are noisy.
- **yaw_rate_error_hist.png**: Distribution of yaw-rate errors over overlap. Useful summary of turning-dynamics mismatch.
- **pos_err_vs_norm_time.png**: Phase-of-execution comparison: XY error vs normalized time (0→1). Highlights early/mid/late differences regardless of absolute duration.
- **yaw_err_vs_norm_time.png**: Phase-of-execution yaw mismatch vs normalized time (0→1). Not a dynamics/latency metric; it is an execution-phase metric.
- **traj_overlay_norm_time.png**: XY overlay after normalized-time alignment. Useful to see how execution-phase alignment differs from arc-length alignment.
