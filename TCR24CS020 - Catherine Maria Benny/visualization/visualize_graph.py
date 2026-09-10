#!/usr/bin/env python3
"""
visualize_graph.py -- Render the Cartesian state graph exported by the C++
Safe Semantic Planner (see include/VisualisationData.h for the JSON schema).

This script performs NO planning of its own. It only reads the JSON that the
C++ program already computed (states, transitions, the selected path) and
draws exactly that -- per assignment section 18 ("Do NOT reimplement D* Lite
in Python. Python is only the visualization layer.").

Usage:
    python3 visualize_graph.py <graph.json> [--out output.png] [--show]

If states have 2D embeddings, their actual (x, y) Cartesian coordinates are
used directly as node positions (assignment section 17) -- NOT an arbitrary
NetworkX spring layout. For dimensionality > 2, PCA is used to project down
to 2D for plotting only; the planner itself always used the original
embeddings.
"""
import argparse
import json
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import networkx as nx
import numpy as np


def load_graph(path):
    with open(path) as f:
        return json.load(f)


def project_to_2d(embeddings):
    """Use the states' own Cartesian coordinates directly for 2D/3D (using
    the first two axes for 3D); for dimensionality > 3, fall back to PCA,
    clearly documented as a visualization-only reduction (assignment
    section 17)."""
    arr = np.array(embeddings, dtype=float)
    dim = arr.shape[1] if arr.ndim == 2 else 1
    if dim <= 2:
        if dim == 1:
            return np.column_stack([arr[:, 0], np.zeros(len(arr))]), False
        return arr, False
    if dim == 3:
        # Direct 2D projection (x, y), documented as dropping the z-axis for
        # this static plot; the planner still used the full 3D embedding.
        return arr[:, :2], False
    # dim > 3: PCA reduction, visualization-only.
    centered = arr - arr.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    reduced = centered @ vt[:2].T
    return reduced, True


def build_graph(data):
    G = nx.DiGraph()
    for s in data["states"]:
        G.add_node(s["id"], embedding=s["embedding"], bad=s["bad"], onPath=s["onPath"])
    for t in data["transitions"]:
        G.add_edge(t["from"], t["to"], **t)
    return G


def draw(data, out_path, show):
    G = build_graph(data)
    ids = list(G.nodes())
    embeddings = [G.nodes[i]["embedding"] for i in ids]
    coords2d, used_pca = project_to_2d(embeddings)
    pos = {i: tuple(coords2d[k]) for k, i in enumerate(ids)}

    init = data["initialState"]
    goal = data["goalState"]
    bad_set = set(data["badStates"])
    path_states = set(data["statePath"])
    path_transitions = set(data["transitionPath"])

    fig, ax = plt.subplots(figsize=(11, 8))

    # --- Draw edges -------------------------------------------------------
    for t in data["transitions"]:
        u, v = t["from"], t["to"]
        if u not in pos or v not in pos:
            continue
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        on_path = t["id"] in path_transitions
        unavailable = not t["available"]

        if on_path:
            style = dict(color="#1b7f3a", linewidth=3.0, alpha=0.95, zorder=5)
        elif unavailable:
            style = dict(color="#b0b0b0", linewidth=1.1, alpha=0.7, linestyle=(0, (4, 3)), zorder=2)
        else:
            style = dict(color="#8888aa", linewidth=1.0, alpha=0.45, zorder=1)

        ax.annotate(
            "", xy=(x2, y2), xytext=(x1, y1),
            arrowprops=dict(
                arrowstyle="-|>", shrinkA=14, shrinkB=14,
                color=style["color"], lw=style["linewidth"], alpha=style["alpha"],
                linestyle=style.get("linestyle", "solid"),
            ),
            zorder=style["zorder"],
        )
        if data.get("_show_edge_costs", True):
            mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            ax.text(mx, my, f"{t['cost']:.1f}", fontsize=7, color="#555555",
                     ha="center", va="center",
                     bbox=dict(boxstyle="round,pad=0.05", fc="white", ec="none", alpha=0.6))

    # --- Draw nodes ---------------------------------------------------------
    for i in ids:
        x, y = pos[i]
        is_bad = i in bad_set
        is_init = i == init
        is_goal = i == goal
        on_path = i in path_states

        if is_init:
            face, edge, size, marker = "#2f6fed", "#0b2d80", 700, "s"
        elif is_goal:
            face, edge, size, marker = "#e8b800", "#7a5c00", 800, "*"
        elif is_bad:
            face, edge, size, marker = "#d1352b", "#6e0f0a", 550, "X"
        else:
            face, edge, size, marker = "#dfe3ee", "#555577", 380, "o"

        lw = 3.0 if on_path and not (is_init or is_goal) else 1.4
        ring = "#1b7f3a" if on_path else edge
        ax.scatter([x], [y], s=size, marker=marker, c=face, edgecolors=ring,
                   linewidths=lw, zorder=10)
        ax.text(x, y, str(i), fontsize=8, ha="center", va="center",
                 zorder=11, fontweight="bold",
                 color="white" if (is_init or is_bad) else "#222222")

    # --- Legend ---------------------------------------------------------
    legend_elems = [
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#2f6fed",
               markeredgecolor="#0b2d80", markersize=13, label="Initial state"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="#e8b800",
               markeredgecolor="#7a5c00", markersize=16, label="Goal state"),
        Line2D([0], [0], marker="X", color="w", markerfacecolor="#d1352b",
               markeredgecolor="#6e0f0a", markersize=13, label="Bad state"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#dfe3ee",
               markeredgecolor="#555577", markersize=11, label="Normal state"),
        Line2D([0], [0], color="#1b7f3a", lw=3, label="Selected path"),
        Line2D([0], [0], color="#8888aa", lw=1.2, alpha=0.5, label="Other transition"),
        Line2D([0], [0], color="#b0b0b0", lw=1.2, linestyle=(0, (4, 3)), label="Unavailable transition"),
    ]
    ax.legend(handles=legend_elems, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9, frameon=True)

    title = data.get("label") or "Safe Semantic Planner -- Graph Visualization"
    subtitle_bits = []
    m = data.get("metrics", {})
    if m:
        subtitle_bits.append(f"success={m.get('success')}")
        subtitle_bits.append(f"cost={m.get('totalCost'):.2f}")
        d = m.get("minimumSafetyDistance")
        subtitle_bits.append(f"minSafetyDist={'inf (no bad states)' if d is None else round(d, 2)}")
        subtitle_bits.append(f"reliability={m.get('reliability'):.3f}")
        subtitle_bits.append(f"badVisited={m.get('badStatesVisited')}")
    pca_note = "  (2D projection via PCA -- planner used full-dimensional embeddings)" if used_pca else ""
    ax.set_title(f"{title}\n" + "   |   ".join(subtitle_bits) + pca_note, fontsize=11)
    ax.set_xlabel("x (Cartesian embedding dim 0)")
    ax.set_ylabel("y (Cartesian embedding dim 1)")
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved {out_path}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("json_path", help="Path to a graph JSON exported by --export")
    ap.add_argument("--out", default=None, help="Output PNG path (default: alongside the input)")
    ap.add_argument("--show", action="store_true", help="Also open an interactive Matplotlib window")
    args = ap.parse_args()

    data = load_graph(args.json_path)
    out = args.out or (args.json_path.rsplit(".", 1)[0] + ".png")
    draw(data, out, args.show)


if __name__ == "__main__":
    main()
