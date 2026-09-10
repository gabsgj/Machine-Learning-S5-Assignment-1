#!/usr/bin/env python3
"""
plot_experiments.py -- Generate the performance plots required by assignment
section "Experimental Evaluation" from the C++ program's actual CSV output.
No values are hard-coded here; everything is read from
results/experiments.csv and results/safety_weight_sweep.csv, both produced
by `./safe_planner --experiment`.

Usage:
    python3 plot_experiments.py results/experiments.csv [--sweep results/safety_weight_sweep.csv] [--outdir results/graphs]
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt


def read_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def plot_planning_time_vs_states(rows, outdir):
    n = [r["numStates"] for r in rows]
    t = [r["planningTimeMs"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(n, t, marker="o", color="#2f6fed")
    ax.set_xlabel("Number of states")
    ax.set_ylabel("Planning time (ms)")
    ax.set_title("Plot 1: Initial Planning Time vs. Number of States")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(outdir, "plot1_planning_time_vs_states.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def plot_explored_vs_states(rows, outdir):
    n = [r["numStates"] for r in rows]
    e = [r["exploredStates"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(n, e, marker="o", color="#1b7f3a")
    ax.set_xlabel("Number of states")
    ax.set_ylabel("Number of states explored (popped from OPEN)")
    ax.set_title("Plot 2: Explored States vs. Number of States")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(outdir, "plot2_explored_vs_states.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def plot_initial_vs_replan_time(rows, outdir):
    n = [r["numStates"] for r in rows]
    init_t = [r["planningTimeMs"] for r in rows]
    replan_t = [r["replanningTimeMs"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    width = 0.35
    x = range(len(n))
    ax.bar([i - width / 2 for i in x], init_t, width=width, label="Initial planning", color="#2f6fed")
    ax.bar([i + width / 2 for i in x], replan_t, width=width, label="Replanning (1 edge disabled)", color="#e07b00")
    ax.set_xticks(list(x))
    ax.set_xticklabels([str(int(v)) for v in n])
    ax.set_xlabel("Number of states")
    ax.set_ylabel("Time (ms)")
    ax.set_title("Plot 3: Initial Planning Time vs. Replanning Time")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    path = os.path.join(outdir, "plot3_initial_vs_replan_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def plot_cost_vs_safety_weight(sweep_rows, outdir):
    gamma = [r["gamma"] for r in sweep_rows]
    cost = [r["totalCost"] for r in sweep_rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(gamma, cost, marker="o", color="#d1352b")
    ax.set_xlabel("Safety weight (gamma)")
    ax.set_ylabel("Selected path total cost")
    ax.set_title("Plot 4: Path Cost vs. Safety Weight")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(outdir, "plot4_cost_vs_safety_weight.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def plot_safety_distance_vs_weight(sweep_rows, outdir):
    gamma = [r["gamma"] for r in sweep_rows]
    dist = [r["minSafetyDistance"] for r in sweep_rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(gamma, dist, marker="o", color="#7a3fe8")
    ax.set_xlabel("Safety weight (gamma)")
    ax.set_ylabel("Minimum distance to nearest bad state, D(P)")
    ax.set_title("Plot 5: Minimum Safety Distance vs. Safety Weight")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(outdir, "plot5_safety_distance_vs_weight.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path", help="Path to results/experiments.csv")
    ap.add_argument("--sweep", default=None, help="Path to results/safety_weight_sweep.csv")
    ap.add_argument("--outdir", default="results/graphs")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    rows = read_csv(args.csv_path)
    rows.sort(key=lambda r: r["numStates"])

    plot_planning_time_vs_states(rows, args.outdir)
    plot_explored_vs_states(rows, args.outdir)
    plot_initial_vs_replan_time(rows, args.outdir)

    sweep_path = args.sweep
    if sweep_path is None:
        candidate = os.path.join(os.path.dirname(args.csv_path), "safety_weight_sweep.csv")
        if os.path.exists(candidate):
            sweep_path = candidate
    if sweep_path and os.path.exists(sweep_path):
        sweep_rows = read_csv(sweep_path)
        sweep_rows.sort(key=lambda r: r["gamma"])
        plot_cost_vs_safety_weight(sweep_rows, args.outdir)
        plot_safety_distance_vs_weight(sweep_rows, args.outdir)
    else:
        print("No safety_weight_sweep.csv found -- skipping plots 4 and 5.")


if __name__ == "__main__":
    main()
