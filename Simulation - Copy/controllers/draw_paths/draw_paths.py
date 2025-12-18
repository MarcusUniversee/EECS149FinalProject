# controllers/draw_gap_paths/draw_gap_paths.py
from controller import Supervisor
import numpy as np
import yaml
import os
from utils import build_heat_polylines, xy_to_xyz_zup, import_node, shape_thick_polyline, sphere_at

# ---------------------------
# Config
# ---------------------------
SIM_COLOR = (0.1, 0.1, 1.0)   # sim path (blue-ish)
REAL_COLOR = (0.2, 1.0, 0.4)  # real path (green-ish)
WP_COLOR = (1.0, 1.0, 0.2)    # waypoints (yellow-ish)

Z_PATH_SIM = 0.02
Z_PATH_REAL = 0.02
Z_PATH_GAP = 0.02
Z_WP = 0.06

WP_RADIUS = 0.035

# How to scale error -> color bins (meters)
# You can tune these.
LOG_CONTRAST = 5
ERR_MIN = 0.0
ERR_MAX = 0.10  # clamp heatmap at this many meters

# Number of discrete color bins for the heat polyline
N_BINS = 12
CREATED_DEFS = [
    "SIM_ARC_PATH",
    "REAL_ARC_PATH"
]

# ---------------------------
# Main
# ---------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

controller_dir = os.path.dirname(__file__)
npz_path = os.path.join(controller_dir, "../../../sim2real_results/log1_ground_with_maxvel0_3/raw_metrics.npz")     # <-- update if needed
yaml_path = os.path.join(controller_dir, "../../../experiments/path1.yaml")     # <-- update if needed

# ---- Load your raw_metrics.npz ----
npz = np.load(npz_path, allow_pickle=True)

# You saved: np.savez(... arc=arc, cp=cp, ...)
# Each entry is a 0-d object array co
arc = npz["arc"].item()

if not arc.get("ok", False):
    raise RuntimeError(f"arc is not ok: {arc.get('reason', 'unknown')}")

sim_xy = np.stack([arc["_sim_xu"], arc["_sim_yu"]], axis=1)
real_xy = np.stack([arc["_real_xu"], arc["_real_yu"]], axis=1)
pos_err = np.asarray(arc["_pos_err"], float)

# Optional downsample if n is huge (keeps visuals light)
def downsample_xy_err(xy, err, step=2):
    if len(xy) <= 2:
        return xy, err
    return xy[::step], err[::step]

sim_xy, _ = downsample_xy_err(sim_xy, pos_err, step=2)   # err not used for sim path
real_xy, _ = downsample_xy_err(real_xy, pos_err, step=2) # err not used for real path

# For heatmap, we need the same sampling for points+err
heat_xy, heat_err = downsample_xy_err(real_xy, pos_err, step=2)

sim_pts = xy_to_xyz_zup(sim_xy, z=Z_PATH_SIM)
real_pts = xy_to_xyz_zup(real_xy, z=Z_PATH_REAL)
heat_pts = xy_to_xyz_zup(heat_xy, z=Z_PATH_GAP)

# ---- Load waypoints yaml ----
with open(yaml_path, "r") as f:
    wp = yaml.safe_load(f)

if isinstance(wp, dict) and "waypoints" in wp:
    waypoints = wp["waypoints"]
elif isinstance(wp, list):
    waypoints = wp
else:
    raise ValueError("Unrecognized waypoint YAML format")

wp_xy = np.array([[w["x"], w["y"]] for w in waypoints], dtype=float)
wp_pts = xy_to_xyz_zup(wp_xy, z=Z_WP)

# ---- Draw: sim, real, heat ----
sim_node = shape_thick_polyline(sim_pts, SIM_COLOR, radius=0.006, name="SIM_ARC_PATH")
#real_node = shape_thick_polyline(real_pts, REAL_COLOR, name="REAL_ARC_PATH")
if sim_node: import_node(robot, sim_node)
#if real_node: import_node(robot, real_node)

# Heatmap path (colored by deviation). Uses real path points but colors by arc pos_err.
heat_nodes = build_heat_polylines(heat_pts, heat_err, log_constrast=LOG_CONTRAST, low=ERR_MIN, high=ERR_MAX, bins=N_BINS)
for node in heat_nodes:
    import_node(robot, node)

# ---- Draw waypoints ----
for i, p in enumerate(wp_pts):
    import_node(robot, sphere_at(p, WP_RADIUS, WP_COLOR, name=f"WP_{i}"))

CREATED_DEFS += [f"WP_{i}" for i in range(len(wp_pts))]
CREATED_DEFS += [f"HEAT_BIN_{i}" for i in range(len(heat_nodes)) ]

def cleanup(supervisor, def_names):
    for name in def_names:
        node = supervisor.getFromDef(name)
        if node is not None:
            node.remove()

# Keep alive
try:
    while robot.step(timestep) != -1:
        pass
finally:
    cleanup(robot, CREATED_DEFS)