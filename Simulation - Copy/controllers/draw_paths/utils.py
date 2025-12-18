import numpy as np
from controller import Supervisor

def xy_to_xyz_zup(xy, z=0.02):
    """Map ground (x,y) -> Webots (x,y,z) for z-up worlds."""
    xy = np.asarray(xy, float)
    out = np.zeros((xy.shape[0], 3), dtype=float)
    out[:, 0] = xy[:, 0]
    out[:, 1] = xy[:, 1]
    out[:, 2] = z
    return out

def import_node(supervisor: Supervisor, node_string: str):
    root = supervisor.getRoot()
    children = root.getField("children")
    children.importMFNodeFromString(-1, node_string)

def shape_thick_polyline(points_xyz, rgb, radius=0.006, name=None, sides=16):
    pts = np.asarray(points_xyz, float)
    if pts.shape[0] < 2:
        return None

    # Build a closed, non-self-intersecting crossSection (CCW)
    angles = np.linspace(0.0, 2.0*np.pi, sides, endpoint=False)
    circle_pts = [(radius*np.cos(a), radius*np.sin(a)) for a in angles]
    circle_pts.append(circle_pts[0])  # CLOSE THE LOOP

    cross_section = ", ".join(f"{x} {y}" for x, y in circle_pts)

    spine = "".join(f"{p[0]} {p[1]} {p[2]}, " for p in pts)

    node = f"""
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor {rgb[0]} {rgb[1]} {rgb[2]}
          emissiveColor {rgb[0]} {rgb[1]} {rgb[2]}
        }}
      }}
      geometry Extrusion {{
        spine [ {spine} ]
        crossSection [ {cross_section} ]
        beginCap TRUE
        endCap TRUE
      }}
    }}
    """
    if name:
        node = f"DEF {name} Transform {{ children [ {node} ] }}"
    return node

def sphere_at(xyz, radius, rgb, name=None):
    node = f"""
    Solid {{
        translation {xyz[0]} {xyz[1]} {xyz[2]}
        children [
            Shape {{
                castShadows FALSE
                appearance PBRAppearance {{
                    baseColor 1 0 0
                    transparency 0.5
                    roughness 1
                    metalness 0
                    emissiveColor 1 0 0
                }}
                geometry Cylinder {{
                    height 0.5
                    radius 0.02
                }}
            }}
        ]
    }}
    """
    if name:
        node = f"DEF {name} {node}"
    return node

def lerp(a, b, t):
    return a + (b - a) * t

def colormap_red_yellow_green(t):
    """
    t in [0,1] where 0=green (small error), 1=red (large error).
    4-stop piecewise: green -> cyan -> yellow -> red
    """
    t = float(np.clip(t, 0.0, 1.0))

    # Anchor colors
    green  = (0.2, 1.0, 0.2)
    cyan   = (0.2, 1.0, 1.0)
    yellow = (1.0, 1.0, 0.2)
    red    = (1.0, 0.2, 0.2)

    if t < 1.0/3.0:
        # green -> cyan
        u = t / (1.0/3.0)
        return (lerp(green[0], cyan[0], u),
                lerp(green[1], cyan[1], u),
                lerp(green[2], cyan[2], u))

    elif t < 2.0/3.0:
        # cyan -> yellow
        u = (t - 1.0/3.0) / (1.0/3.0)
        return (lerp(cyan[0], yellow[0], u),
                lerp(cyan[1], yellow[1], u),
                lerp(cyan[2], yellow[2], u))

    else:
        # yellow -> red
        u = (t - 2.0/3.0) / (1.0/3.0)
        return (lerp(yellow[0], red[0], u),
                lerp(yellow[1], red[1], u),
                lerp(yellow[2], red[2], u))

def logistic_contrast(t, k=10.0):
    # t in [0,1], k controls how steep the middle is
    t = np.clip(t, 0.0, 1.0)
    x = 2.0 * t - 1.0                 # map to [-1, 1], center at 0
    y = 1.0 / (1.0 + np.exp(-k * x))  # logistic in (0,1)
    # normalize so endpoints map exactly to 0 and 1
    y0 = 1.0 / (1.0 + np.exp(k))
    y1 = 1.0 / (1.0 + np.exp(-k))
    return (y - y0) / (y1 - y0)

def build_heat_polylines(points_xyz, err, log_constrast=10, low=0, high=0.1, bins=20):
    """
    Build multiple small polylines, one per error bin, so each segment can be colored.
    We do this by grouping contiguous segments whose average error falls in same bin.

    Returns list of node strings.
    """
    pts = np.asarray(points_xyz, float)
    err = np.asarray(err, float)
    n = min(len(pts), len(err))
    pts = pts[:n]
    err = err[:n]
    if n < 2:
        return []

    # segment error = average of endpoints
    seg_e = 0.5 * (err[:-1] + err[1:])
    seg_t = np.clip((seg_e - low) / (high - low), 0.0, 1.0)
    seg_t = logistic_contrast(seg_t, k=10.0)
    seg_bin = np.floor(seg_t * (bins - 1) + 1e-9).astype(int)

    nodes = []
    start = 0
    while start < len(seg_bin):
        b = seg_bin[start]
        end = start + 1
        while end < len(seg_bin) and seg_bin[end] == b:
            end += 1

        # This run covers segments [start, end-1], which uses points [start, end]
        run_pts = pts[start:(end + 1)]
        # pick color based on bin center t
        t_center = (b + 0.5) / bins
        rgb = colormap_red_yellow_green(t_center)

        node = shape_thick_polyline(run_pts, rgb, name=f"HEAT_BIN_{start}")
        if node:
            nodes.append(node)

        start = end

    return nodes