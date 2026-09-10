#!/usr/bin/env python3
"""
visualize_dynamic.py -- Side-by-side "before" and "after" comparison for
dynamic replanning demonstrations (assignment section 20).

Reads two JSON files exported by the C++ planner (produced automatically by
`--test 4/5/6 --export FILE.json`, which writes FILE_before.json and
FILE_after.json) and renders them as two panels sharing one figure, with the
previous path shown faintly on the "after" panel so the change is visually
obvious.

Usage:
    python3 visualize_dynamic.py results/graphs/test4_before.json results/graphs/test4_after.json --out results/graphs/test4_dynamic.png
"""
import argparse
import json

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from visualize_graph import build_graph, project_to_2d


def draw_panel(ax, data, prev_path_states=None, prev_path_transitions=None):
    G = build_graph(data)
    ids = list(G.nodes())
    embeddings = [G.nodes[i]["embedding"] for i in ids]
    coords2d, _ = project_to_2d(embeddings)
    pos = {i: tuple(coords2d[k]) for k, i in enumerate(ids)}

    init = data["initialState"]
    goal = data["goalState"]
    bad_set = set(data["badStates"])
    path_states = set(data["statePath"])
    path_transitions = set(data["transitionPath"])
    prev_transitions = prev_path_transitions or set()

    for t in data["transitions"]:
        u, v = t["from"], t["to"]
        if u not in pos or v not in pos:
            continue
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        on_path = t["id"] in path_transitions
        was_on_prev_path = t["id"] in prev_transitions
        unavailable = not t["available"]

        if on_path:
            color, lw, alpha, ls = "#1b7f3a", 3.2, 0.95, "solid"
        elif was_on_prev_path:
            # Previous path, now abandoned or changed -- shown as a faint
            # orange dashed reference line so the change is obvious.
            color, lw, alpha, ls = "#e07b00", 2.2, 0.55, (0, (2, 2))
        elif unavailable:
            color, lw, alpha, ls = "#b0b0b0", 1.1, 0.7, (0, (4, 3))
        else:
            color, lw, alpha, ls = "#8888aa", 1.0, 0.4, "solid"

        ax.annotate(
            "", xy=(x2, y2), xytext=(x1, y1),
            arrowprops=dict(arrowstyle="-|>", shrinkA=14, shrinkB=14,
                             color=color, lw=lw, alpha=alpha, linestyle=ls),
        )

    for i in ids:
        x, y = pos[i]
        is_bad = i in bad_set
        is_init = i == init
        is_goal = i == goal
        on_path = i in path_states

        if is_init:
            face, edge, size, marker = "#2f6fed", "#0b2d80", 650, "s"
        elif is_goal:
            face, edge, size, marker = "#e8b800", "#7a5c00", 750, "*"
        elif is_bad:
            face, edge, size, marker = "#d1352b", "#6e0f0a", 500, "X"
        else:
            face, edge, size, marker = "#dfe3ee", "#555577", 340, "o"

        ring = "#1b7f3a" if on_path else edge
        lw = 3.0 if on_path and not (is_init or is_goal) else 1.4
        ax.scatter([x], [y], s=size, marker=marker, c=face, edgecolors=ring, linewidths=lw, zorder=10)
        ax.text(x, y, str(i), fontsize=8, ha="center", va="center", zorder=11, fontweight="bold",
                 color="white" if (is_init or is_bad) else "#222222")

    ax.set_aspect("equal", adjustable="datalim")
    return path_states, path_transitions


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("before_json")
    ap.add_argument("after_json")
    ap.add_argument("--out", default="dynamic_comparison.png")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    with open(args.before_json) as f:
        before = json.load(f)
    with open(args.after_json) as f:
        after = json.load(f)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    axes[0].set_title(f"BEFORE\n{before.get('label','')}\ncost={before['metrics']['totalCost']:.2f}", fontsize=10)
    draw_panel(axes[0], before)

    prev_states, prev_transitions = set(before["statePath"]), set(before["transitionPath"])
    axes[1].set_title(f"AFTER (replanned)\n{after.get('label','')}\ncost={after['metrics']['totalCost']:.2f}", fontsize=10)
    draw_panel(axes[1], after, prev_states, prev_transitions)

    legend_elems = [
        Line2D([0], [0], color="#1b7f3a", lw=3, label="Current selected path"),
        Line2D([0], [0], color="#e07b00", lw=2.2, linestyle=(0, (2, 2)), label="Previous path (superseded)"),
        Line2D([0], [0], color="#b0b0b0", lw=1.2, linestyle=(0, (4, 3)), label="Unavailable transition"),
        Line2D([0], [0], marker="X", color="w", markerfacecolor="#d1352b", markeredgecolor="#6e0f0a",
               markersize=12, label="Bad state"),
    ]
    fig.legend(handles=legend_elems, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Dynamic Replanning: Before vs. After", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"Saved {args.out}")
    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
