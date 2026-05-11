#!/usr/bin/env python3
import sys
from pathlib import Path
import argparse
from collections import deque

import networkx as nx
import numpy as np
import yaml
from PIL import Image
from scipy.interpolate import splprep, splev
from scipy.ndimage import distance_transform_edt, binary_closing
from scipy.spatial import cKDTree
from skimage.morphology import disk, medial_axis


# ─────────────────────────────────────────────────────────────
# LOAD
# ─────────────────────────────────────────────────────────────
def load_map(yaml_path: str):
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    img_path = Path(meta["image"])
    if not img_path.is_absolute():
        img_path = yaml_path.parent / img_path

    arr = np.array(Image.open(img_path).convert("L"), dtype=np.uint8)
    return arr, meta


def get_strict_free_mask(arr, free_thresh=250):
    return arr >= free_thresh


# ─────────────────────────────────────────────────────────────
# SKELETON
# ─────────────────────────────────────────────────────────────
def build_skeleton(work_mask, ridge_frac):
    dist = distance_transform_edt(work_mask)
    max_dist = dist.max()
    if max_dist == 0:
        return None, 0

    skel, _ = medial_axis(dist >= ridge_frac * max_dist, return_distance=True)

    rows, cols = np.where(skel)
    if len(rows) == 0:
        return None, 0

    return np.column_stack([cols, rows]).astype(float), max_dist


# ─────────────────────────────────────────────────────────────
# GRAPH (FAST)
# ─────────────────────────────────────────────────────────────
def build_pixel_graph(pts):
    tree = cKDTree(pts)
    pairs = tree.query_pairs(r=1.5)

    G = nx.Graph()
    G.add_nodes_from(range(len(pts)))
    G.add_edges_from(pairs)
    return G


# ─────────────────────────────────────────────────────────────
# PRUNE (FAST)
# ─────────────────────────────────────────────────────────────
def prune_branches(G):
    Gp = G.copy()
    q = deque([n for n in Gp.nodes if Gp.degree(n) == 1])

    while q:
        n = q.popleft()
        if n not in Gp:
            continue

        neighbors = list(Gp.neighbors(n))
        Gp.remove_node(n)

        for nb in neighbors:
            if Gp.degree(nb) == 1:
                q.append(nb)

    return Gp


# ─────────────────────────────────────────────────────────────
# ROBUST CYCLE FIND
# ─────────────────────────────────────────────────────────────
def find_cycle(Gp, min_nodes):
    best = None

    for comp in sorted(nx.connected_components(Gp), key=len, reverse=True):
        if len(comp) < min_nodes:
            continue

        sub = Gp.subgraph(comp)

        try:
            cycles = nx.cycle_basis(sub)
        except:
            continue

        if not cycles:
            continue

        largest = max(cycles, key=len)

        if len(largest) >= min_nodes:
            if best is None or len(largest) > len(best):
                best = largest

    return best


def traverse_cycle(cycle_nodes, G):
    subg = G.subgraph(set(cycle_nodes))
    start = cycle_nodes[0]

    path = [start]
    visited = {start}
    curr = start

    while True:
        nbrs = [n for n in subg.neighbors(curr) if n not in visited]
        if not nbrs:
            break
        curr = nbrs[0]
        path.append(curr)
        visited.add(curr)

    return path


# ─────────────────────────────────────────────────────────────
# PARAM SWEEP (FAST + FALLBACK)
# ─────────────────────────────────────────────────────────────
FAST_CLOSING = [0, 2, 5]
FAST_RIDGE   = [0.30, 0.25, 0.35]

FALLBACK_CLOSING = [8, 12]
FALLBACK_RIDGE   = [0.20]


def find_cycle_auto(strict_free, min_cycle_nodes=50, verbose=True):

    # FAST PASS
    for closing_r in FAST_CLOSING:
        work_mask = binary_closing(strict_free, disk(closing_r)) if closing_r > 0 else strict_free

        for ridge_frac in FAST_RIDGE:
            pts, _ = build_skeleton(work_mask, ridge_frac)
            if pts is None:
                continue

            G  = build_pixel_graph(pts)
            Gp = prune_branches(G)

            if Gp.number_of_nodes() == 0:
                continue

            cycle = find_cycle(Gp, min_cycle_nodes)
            if cycle is not None:
                if verbose:
                    print(f"[FAST] Cycle found: {len(cycle)} nodes "
                          f"(closing={closing_r}, ridge={ridge_frac})")
                path_idx = traverse_cycle(cycle, Gp)
                return pts[path_idx], closing_r, ridge_frac

    # FALLBACK PASS
    if verbose:
        print("[centerline] Fast pass failed — trying fallback...")

    for closing_r in FALLBACK_CLOSING:
        work_mask = binary_closing(strict_free, disk(closing_r))

        for ridge_frac in FALLBACK_RIDGE:
            pts, _ = build_skeleton(work_mask, ridge_frac)
            if pts is None:
                continue

            G  = build_pixel_graph(pts)
            Gp = prune_branches(G)

            cycle = find_cycle(Gp, min_cycle_nodes)
            if cycle is not None:
                if verbose:
                    print(f"[FALLBACK] Cycle found: {len(cycle)} nodes "
                          f"(closing={closing_r}, ridge={ridge_frac})")
                path_idx = traverse_cycle(cycle, Gp)
                return pts[path_idx], closing_r, ridge_frac

    raise RuntimeError("Failed to find cycle.")


# ─────────────────────────────────────────────────────────────
# SPLINE
# ─────────────────────────────────────────────────────────────
def fit_spline(path_pts, n_out=300, smoothing=1000.0):
    x = path_pts[:, 0]
    y = path_pts[:, 1]

    dx = np.diff(x)
    dy = np.diff(y)
    keep = np.hstack(([True], (dx**2 + dy**2) >= 1.0))

    x, y = x[keep], y[keep]

    x = np.append(x, x[0])
    y = np.append(y, y[0])

    tck, _ = splprep([x, y], s=smoothing, per=True, k=3)

    u = np.linspace(0, 1, n_out, endpoint=False)
    sx, sy = splev(u, tck)
    dxs, dys = splev(u, tck, der=1)

    return sx, sy, np.arctan2(dys, dxs)


# ─────────────────────────────────────────────────────────────
# SNAP + HEADINGS
# ─────────────────────────────────────────────────────────────
def snap_to_free(sx, sy, free_mask):
    sx_i = np.clip(np.round(sx).astype(int), 0, free_mask.shape[1]-1)
    sy_i = np.clip(np.round(sy).astype(int), 0, free_mask.shape[0]-1)

    outside = ~free_mask[sy_i, sx_i]
    if not outside.any():
        return sx, sy, 0

    rows, cols = np.where(free_mask)
    tree = cKDTree(np.column_stack([cols, rows]))

    _, idxs = tree.query(np.column_stack([sx[outside], sy[outside]]))

    sx[outside] = cols[idxs]
    sy[outside] = rows[idxs]

    return sx, sy, int(outside.sum())


def recompute_headings(sx, sy):
    dx = np.roll(sx, -1) - np.roll(sx, 1)
    dy = np.roll(sy, -1) - np.roll(sy, 1)
    return np.arctan2(dy, dx)


def pixels_to_world(xs, ys, meta, h):
    res = float(meta["resolution"])
    ox, oy = meta["origin"][:2]
    return ox + xs * res, oy + (h - 1 - ys) * res


# ─────────────────────────────────────────────────────────────
# CCW ENFORCEMENT
# ─────────────────────────────────────────────────────────────
def ensure_ccw(wx, wy, heading_world):
    """
    Reverse waypoint order if the path is clockwise.
    Uses the shoelace formula for signed area — positive = CCW in world
    coordinates (y-up), which is what we want after pixels_to_world flips y.
    Must be called AFTER pixels_to_world and heading_world computation.
    """
    signed_area = 0.5 * np.sum(
        wx * np.roll(wy, -1) - np.roll(wx, -1) * wy
    )
    if signed_area < 0:
        wx = wx[::-1].copy()
        wy = wy[::-1].copy()
        # Flip headings 180° and rewrap to [-π, π]
        heading_world = (heading_world[::-1] + np.pi + np.pi) % (2 * np.pi) - np.pi
        print("[centerline] Path was CW — reversed to CCW")
    else:
        print("[centerline] Path already CCW — no reversal needed")
    return wx, wy, heading_world


# ─────────────────────────────────────────────────────────────
# VISUALIZER
# ─────────────────────────────────────────────────────────────
def visualize(arr, raw_pts, sx, sy, heading, save_path="preview.png"):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].imshow(arr, cmap="gray")
    axes[0].plot(raw_pts[:, 0], raw_pts[:, 1], "r-", lw=1.5)
    axes[0].set_title("Raw Cycle")

    axes[1].imshow(arr, cmap="gray")
    axes[1].plot(sx, sy, "r-", lw=2)

    step = max(1, len(sx)//25)
    axes[1].scatter(sx[::step], sy[::step], c="yellow")

    for i in range(0, len(sx), step):
        axes[1].arrow(
            sx[i], sy[i],
            np.cos(heading[i])*10,
            np.sin(heading[i])*10,
            color="lime", head_width=2
        )

    axes[1].set_title("Smoothed Centerline")

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"[viz] Saved → {save_path}")
    plt.show()


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--output", default="centerline.csv")
    parser.add_argument("--n-waypoints", type=int, default=300)
    parser.add_argument("--smoothing", type=float, default=1000.0)
    parser.add_argument("--free-thresh", type=int, default=250)
    parser.add_argument("--min-cycle-nodes", type=int, default=50)
    parser.add_argument("--visualize", action="store_true")

    args = parser.parse_args()

    print("[centerline] Loading map...")
    arr, meta = load_map(args.map)

    strict_free = get_strict_free_mask(arr, args.free_thresh)

    raw_pts, _, _ = find_cycle_auto(strict_free, args.min_cycle_nodes)

    raw_pts = raw_pts[::-1]

    sx, sy, heading = fit_spline(
        raw_pts,
        n_out=args.n_waypoints,
        smoothing=args.smoothing
    )

    sx, sy, _ = snap_to_free(sx, sy, strict_free)

    # Recompute headings after snapping (pixel space)
    heading = recompute_headings(sx, sy)

    # Convert to world coordinates
    wx, wy = pixels_to_world(sx, sy, meta, arr.shape[0])

    # Convert pixel-space headings to world-space
    # pixels_to_world flips y, so negate the y-component of the heading
    heading_world = np.arctan2(-np.sin(heading), np.cos(heading))

    # Enforce CCW — must be called after world coordinate conversion
    wx, wy, heading_world = ensure_ccw(wx, wy, heading_world)

    np.savetxt(
        args.output,
        np.column_stack([wx, wy, heading_world]),
        delimiter=",",
        fmt="%.6f"
    )

    print(f"[centerline] Saved {len(wx)} waypoints → {args.output}")

    if args.visualize:
        visualize(arr, raw_pts, sx, sy, heading)

    print("[centerline] Done.")


if __name__ == "__main__":
    main()