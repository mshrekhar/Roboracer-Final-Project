#!/usr/bin/env python3
"""
optimize.py — Shortest-Path Raceline Optimization
==================================================
Reads a centerline CSV (x, y, [heading|width|...]) and a ROS map (PGM + YAML),
solves a QP for the laterally-offset path that minimizes total length within
track-width constraints, and writes the optimized line back to the same CSV.

QP formulation
--------------
Each centerline point p_i has a unit normal n_i. We pick a lateral offset
alpha_i in [-w_R_i + margin,  w_L_i - margin]. The new point is:

    p_i' = p_i + alpha_i * n_i

The squared distance between consecutive new points is:

    |p_{i+1}' - p_i'|^2  =  |d_i + alpha_{i+1} n_{i+1} - alpha_i n_i|^2
                         =  |d_i|^2  +  2 d_i^T (alpha_{i+1} n_{i+1} - alpha_i n_i)
                                     +  |alpha_{i+1} n_{i+1} - alpha_i n_i|^2

where d_i = p_{i+1} - p_i. Expanding and stacking over all i gives a QP:

    min   0.5 alpha^T P alpha  +  q^T alpha
    s.t.  lb <= alpha <= ub

with P symmetric PSD. We solve with cvxpy/OSQP.

Usage
-----
    python3 optimize.py \\
        --map        /path/to/map.yaml \\
        --centerline /path/to/centerline.csv \\
        [--margin 0.2] [--shrink 0.1]

Centerline CSV format (matches the one centerline_only.py writes):
    col 0: x  (meters, map frame)
    col 1: y  (meters, map frame)
    col 2: heading or other (preserved/recomputed)

Output: overwrites --centerline with the same column count, optimized x/y,
and recomputed heading in column 2.
"""

import argparse
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


# ════════════════════════════════════════════════════════════════════
#  Map loading + width computation
# ════════════════════════════════════════════════════════════════════
def load_map(yaml_path: str):
    """Load a ROS map_server PGM+YAML pair. Returns (occupied_mask, res, origin)."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    res = float(meta["resolution"])
    origin = np.array(meta["origin"][:2], dtype=float)
    pgm_path = os.path.join(os.path.dirname(yaml_path), meta["image"])

    img = np.array(Image.open(pgm_path))   # 0=occupied, 254=free, 205=unknown
    # Treat unknown as occupied for safety (don't drive into the unknown).
    free = (img >= 250)
    occupied = ~free

    # PGM is stored top-row-first; ROS map y axis goes up. Flip vertically so
    # row index increases with world y.
    occupied = np.flipud(occupied)
    return occupied, res, origin


def world_to_grid(xy: np.ndarray, res: float, origin: np.ndarray):
    """xy: (N,2) in meters. Returns (N,2) integer grid (col, row)."""
    g = (xy - origin) / res
    return g.astype(int)


def compute_widths(centerline: np.ndarray,
                   normals: np.ndarray,
                   occupied: np.ndarray,
                   res: float,
                   origin: np.ndarray,
                   max_search_m: float = 3.0):
    """
    For each centerline point, ray-cast along ±normal in the occupancy grid
    and return (w_left, w_right) in meters.

    "Left" follows the +normal direction (n is rotated 90° CCW from tangent),
    "right" follows -normal. We use the distance-transform on the FREE space:
    dist_to_wall[r,c] = meters to the nearest occupied cell. Then we walk
    along the normal in small steps and find where dist_to_wall hits 0
    (or where we cross into an occupied cell).
    """
    H, W = occupied.shape
    free = ~occupied

    # Distance transform of free-space: each free cell -> distance (in cells)
    # to nearest occupied cell. Useful as a soft "headroom" map.
    dist_cells = distance_transform_edt(free)
    dist_m = dist_cells * res

    n_pts = centerline.shape[0]
    w_left = np.zeros(n_pts)
    w_right = np.zeros(n_pts)

    step_m = res * 0.5             # half a cell per step
    n_steps = int(max_search_m / step_m)

    for i in range(n_pts):
        p = centerline[i]
        n = normals[i]

        # +n direction (left)
        for k in range(1, n_steps + 1):
            q = p + n * (k * step_m)
            col = int((q[0] - origin[0]) / res)
            row = int((q[1] - origin[1]) / res)
            if row < 0 or row >= H or col < 0 or col >= W:
                w_left[i] = (k - 1) * step_m
                break
            if occupied[row, col]:
                w_left[i] = (k - 1) * step_m
                break
        else:
            w_left[i] = max_search_m

        # -n direction (right)
        for k in range(1, n_steps + 1):
            q = p - n * (k * step_m)
            col = int((q[0] - origin[0]) / res)
            row = int((q[1] - origin[1]) / res)
            if row < 0 or row >= H or col < 0 or col >= W:
                w_right[i] = (k - 1) * step_m
                break
            if occupied[row, col]:
                w_right[i] = (k - 1) * step_m
                break
        else:
            w_right[i] = max_search_m

    return w_left, w_right


# ════════════════════════════════════════════════════════════════════
#  Geometry: tangents + normals on closed loop
# ════════════════════════════════════════════════════════════════════
def compute_tangents_normals(xy: np.ndarray):
    """
    Closed-loop central differences. Returns unit tangents and unit normals.
    Normal is tangent rotated 90° CCW: n = R(90) t = (-t_y, t_x).
    """
    n = xy.shape[0]
    # Forward diff with wrap-around
    nxt = np.roll(xy, -1, axis=0)
    prv = np.roll(xy,  1, axis=0)
    t = (nxt - prv)
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    nrm = np.stack([-t[:, 1], t[:, 0]], axis=1)
    return t, nrm


# ════════════════════════════════════════════════════════════════════
#  Shortest-path QP
# ════════════════════════════════════════════════════════════════════
def build_shortest_path_qp(xy: np.ndarray,
                           normals: np.ndarray,
                           w_left: np.ndarray,
                           w_right: np.ndarray,
                           margin: float):
    """
    Build P, q, lb, ub for the shortest-path QP.

    Objective: sum_i |p_{i+1} + a_{i+1} n_{i+1} - p_i - a_i n_i|^2
             = sum_i |d_i + a_{i+1} n_{i+1} - a_i n_i|^2

    Each term, expanded:
      |d|^2  +  2 d^T (a_{i+1} n_{i+1} - a_i n_i)
             +  a_{i+1}^2 |n_{i+1}|^2  -  2 a_i a_{i+1} n_i^T n_{i+1}  +  a_i^2 |n_i|^2

    |n|^2 = 1, so quadratic part contributes:
       a_i^2  on diagonal i
       a_{i+1}^2  on diagonal i+1
       -2 a_i a_{i+1} (n_i^T n_{i+1})  on off-diagonals

    Linear part:
       -2 d_i^T n_i      on a_i
       +2 d_i^T n_{i+1}  on a_{i+1}

    Constants drop out of argmin.
    """
    N = xy.shape[0]
    d = np.roll(xy, -1, axis=0) - xy            # d_i = p_{i+1} - p_i

    # Quadratic part: assemble as N x N. Each segment i contributes to rows/cols
    # (i, i+1) with the wrap.
    P = np.zeros((N, N))
    q = np.zeros(N)

    for i in range(N):
        j = (i + 1) % N
        n_i = normals[i]
        n_j = normals[j]
        cross = float(n_i @ n_j)        # scalar n_i^T n_{i+1}

        # quadratic
        P[i, i] += 1.0
        P[j, j] += 1.0
        P[i, j] += -cross
        P[j, i] += -cross

        # linear
        q[i] += -2.0 * float(d[i] @ n_i)
        q[j] += +2.0 * float(d[i] @ n_j)

    # Convention: cvxpy uses (1/2) alpha^T P alpha + q^T alpha. We expanded the
    # full sum-of-squares which already has the factor of 2 absorbed, so multiply
    # P by 2 to match (1/2) form.
    P = 2.0 * P

    # Bounds: alpha_i in [-(w_right - margin),  +(w_left - margin)]
    lb = -(w_right - margin)
    ub = +(w_left  - margin)

    # If margin makes a corridor infeasible (very tight spot), clip to 0.
    lb = np.minimum(lb, 0.0)
    ub = np.maximum(ub, 0.0)

    return P, q, lb, ub


def solve_qp(P, q, lb, ub):
    """Solve QP with cvxpy + OSQP. Returns alpha vector."""
    import cvxpy as cp
    N = P.shape[0]
    a = cp.Variable(N)
    # Symmetrize P for PSD-ness in cvxpy
    P_sym = 0.5 * (P + P.T)
    objective = cp.Minimize(0.5 * cp.quad_form(a, cp.psd_wrap(P_sym)) + q @ a)
    constraints = [a >= lb, a <= ub]
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.OSQP, verbose=False, max_iter=20000,
               eps_abs=1e-6, eps_rel=1e-6)
    if a.value is None:
        raise RuntimeError(f"QP failed: status={prob.status}")
    return np.asarray(a.value).flatten()


# ════════════════════════════════════════════════════════════════════
#  Heading recompute
# ════════════════════════════════════════════════════════════════════
def recompute_heading(xy: np.ndarray):
    """Closed-loop heading from central differences. Returns radians in (-pi, pi]."""
    nxt = np.roll(xy, -1, axis=0)
    prv = np.roll(xy,  1, axis=0)
    t = nxt - prv
    return np.arctan2(t[:, 1], t[:, 0])


# ════════════════════════════════════════════════════════════════════
#  Main
# ════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map",        required=True, help="Path to map.yaml")
    parser.add_argument("--centerline", required=True, help="Path to centerline.csv (input)")
    parser.add_argument("--output",     default=None,
                        help="Output CSV path. If omitted, overwrites --centerline.")
    parser.add_argument("--margin",     type=float, default=0.20,
                        help="Safety margin from each wall, meters (default 0.2)")
    parser.add_argument("--shrink",     type=float, default=0.0,
                        help="Extra inward shrink on widths after sampling, meters (default 0)")
    parser.add_argument("--max-search", type=float, default=3.0,
                        help="Max ray-cast distance for width sampling, meters")
    args = parser.parse_args()

    print(f"[optimize] Loading centerline: {args.centerline}")
    raw = np.loadtxt(args.centerline, delimiter=",")
    if raw.ndim == 1:
        raw = raw[None, :]
    if raw.shape[1] < 2:
        print("[optimize] ERROR: centerline must have at least 2 columns (x, y)")
        sys.exit(1)
    xy = raw[:, :2].astype(float)
    n_extra_cols = raw.shape[1] - 2
    print(f"[optimize]   {xy.shape[0]} points, {raw.shape[1]} columns")

    # If first and last point are duplicates (closed loop written twice), drop the duplicate
    if np.linalg.norm(xy[0] - xy[-1]) < 1e-3 and xy.shape[0] > 1:
        xy = xy[:-1]
        if n_extra_cols > 0:
            raw = raw[:-1]
        print(f"[optimize]   Dropped duplicate closing point -> {xy.shape[0]} points")

    print(f"[optimize] Loading map: {args.map}")
    occupied, res, origin = load_map(args.map)
    print(f"[optimize]   grid {occupied.shape}, res={res} m, origin={origin}")

    print("[optimize] Computing tangents and normals...")
    _, normals = compute_tangents_normals(xy)

    print("[optimize] Sampling track widths...")
    w_left, w_right = compute_widths(xy, normals, occupied, res, origin,
                                     max_search_m=args.max_search)
    if args.shrink > 0:
        w_left  = np.maximum(w_left  - args.shrink, 0.0)
        w_right = np.maximum(w_right - args.shrink, 0.0)
    print(f"[optimize]   w_left:  min={w_left.min():.2f}  mean={w_left.mean():.2f}  max={w_left.max():.2f}")
    print(f"[optimize]   w_right: min={w_right.min():.2f}  mean={w_right.mean():.2f}  max={w_right.max():.2f}")

    print(f"[optimize] Building QP (margin={args.margin} m)...")
    P, q, lb, ub = build_shortest_path_qp(xy, normals, w_left, w_right, args.margin)

    print("[optimize] Solving QP...")
    alpha = solve_qp(P, q, lb, ub)
    print(f"[optimize]   alpha:   min={alpha.min():+.3f}  mean={alpha.mean():+.3f}  max={alpha.max():+.3f}")
    print(f"[optimize]   |alpha|: mean={np.abs(alpha).mean():.3f}  max={np.abs(alpha).max():.3f}")

    # Apply offsets
    new_xy = xy + alpha[:, None] * normals

    # Length report
    orig_len = float(np.sum(np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)))
    new_len  = float(np.sum(np.linalg.norm(np.roll(new_xy, -1, axis=0) - new_xy, axis=1)))
    print(f"[optimize] Length: {orig_len:.2f} m -> {new_len:.2f} m  ({100*(new_len-orig_len)/orig_len:+.2f}%)")

    # Recompute heading
    new_heading = recompute_heading(new_xy)

    # Build output: x, y, heading [, original cols 3+ preserved]
    out = np.zeros((new_xy.shape[0], max(3, raw.shape[1])))
    out[:, 0] = new_xy[:, 0]
    out[:, 1] = new_xy[:, 1]
    out[:, 2] = new_heading
    if raw.shape[1] > 3:
        out[:, 3:] = raw[:new_xy.shape[0], 3:]

    out_path = args.output if args.output else args.centerline

    # Backup original only if we're overwriting it
    if out_path == args.centerline:
        backup = args.centerline + ".pre_optimize.bak"
        if not os.path.exists(backup):
            np.savetxt(backup, raw, delimiter=",", fmt="%.6f")
            print(f"[optimize] Backed up original to: {backup}")

    np.savetxt(out_path, out, delimiter=",", fmt="%.6f")
    print(f"[optimize] Wrote optimized line: {out_path}")
    print("[optimize] Done.")


if __name__ == "__main__":
    main()