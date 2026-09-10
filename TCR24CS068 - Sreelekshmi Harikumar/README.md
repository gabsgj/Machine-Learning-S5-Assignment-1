# Safe Semantic Planner

**PCCST503 – Machine Learning | Assignment 1**
*Design of a Safe Semantic Planner in a Finite Cartesian State Space*

A **D\* Lite** based path planner, implemented in Python, that computes safe, cost-aware paths between states embedded in a finite Cartesian state space — while avoiding bad states and efficiently replanning when the environment changes.

 **Live demo:** [safe-semantic-planner.streamlit.app](https://safe-semantic-planner-bzkdux9hn34xd8nxhkxuxd.streamlit.app/)

---

## Table of Contents

- [Overview](#overview)
- [Problem Definition](#problem-definition)
- [Features](#features)
- [Algorithm: D\* Lite](#algorithm-d-lite)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Interactive Streamlit App](#interactive-streamlit-app)
- [Test Cases](#test-cases)
- [Experimental Evaluation](#experimental-evaluation)
- [Dynamic Replanning](#dynamic-replanning)
- [Complexity Analysis](#complexity-analysis)
- [Author](#author)

---

## Overview

This project implements a generic planning algorithm that computes a **safe path** through a finite set of states `S = {s1, s2, ..., sn}` embedded in `ℝᵈ`. Every state is a vector, every transition between states carries a **cost**, **safety score**, **reliability**, and **availability flag**, and a subset of states is marked as **bad** and must never be visited.

The planner is built around **D\* Lite**, an incremental heuristic search algorithm, so that when the environment changes — the goal moves, bad states change, transitions are disabled/added — the planner can **reuse prior search effort** instead of re-solving the problem from scratch.

Although inspired by AI planning, the emphasis of this project is on:

- Graph search (D\* Lite)
- Heuristic design
- Multi-objective optimization (cost, safety, reliability)
- Software engineering (clean interfaces, incremental updates)
- Experimental evaluation

## Problem Definition

| Assignment Concept | Implementation |
|---|---|
| State `si = (x1, ..., xd)` | `State` — has an `id`, `name`, and `embedding` (coordinate vector) |
| Transition `(si, sj)` | `Transition` — has `id`, `from_state`, `to_state`, `cost`, `safety`, `reliability`, `available` |
| Bad states `B` | `PlanningProblem.bad_states` — a set of state IDs the planner must never enter |
| Initial / goal state | `PlanningProblem.initial_state` / `PlanningProblem.goal_state` |
| Planner interface | `DStarLite.plan()` / `DStarLite.replan()` returning a `PlanningResult` |

### Objective Function

The planner scores paths using a weighted combination of the assignment's objective function:

```
Score(P) = α·G − β·C + γ·D + δ·R
```

where `G` is goal completion, `C` is cumulative transition cost, `D` is the minimum safety distance to any bad state, and `R` is cumulative reliability. In this implementation, the trade-off between cost, safety, and reliability is exposed directly as tunable **weights** (`cost_weight`, `safety_weight`, `reliability_weight`) in both the planner constructor and the Streamlit sidebar, so a user can shift the planner from "cheapest path" toward "safest path" interactively.

## Features

- D\* Lite based incremental path planning
- Finite Cartesian state-space representation
- Transition cost optimization
- Safety-aware planning (maximizes minimum distance to bad states)
- Reliability-aware planning
- Hard avoidance of bad states
- Dynamic transition disabling / enabling
- Dynamic transition addition
- Dynamic goal updates
- Dynamic bad-state updates
- Incremental replanning (no full graph rebuild)
- Interactive Streamlit visualization of the state space and selected path
- Automated experimental benchmark suite covering all six assignment test cases
- Planning/replanning performance measurement (time, explored states)

## Algorithm: D\* Lite

D\* Lite was chosen over LPA\* because the assignment's dynamic-environment requirements (moving goal, changing bad states, transitions appearing/disappearing) map directly onto D\* Lite's core strength: it searches **backward from the goal** and maintains `g` and `rhs` values for every state so that, after a local change, only the affected region of the search needs to be repaired via a priority queue of inconsistent states, instead of re-running search over the whole graph.

**Key design choices:**

1. **State representation** — Each `State` stores an integer `id`, a human-readable `name`, and a `d`-dimensional `embedding` used both for the heuristic and for visualization.
2. **Heuristic function** — Euclidean distance between state embeddings, `h(s, goal) = ||embedding(s) − embedding(goal)||₂`, which is admissible and consistent for the Cartesian metric space used here.
3. **Safety computation** — For every candidate state, the minimum Euclidean distance to any bad state is computed and folded into the transition's `safety` score; the planner's `safety_weight` trades this off against raw path cost.
4. **Edge cost** — A weighted combination of a transition's `cost`, its `(1 − safety)` penalty, and its `(1 − reliability)` penalty, scaled by `cost_weight`, `safety_weight`, and `reliability_weight` respectively. Unavailable transitions are treated as having infinite cost and are never traversed.
5. **Data structures** — A priority queue (min-heap) keyed by the D\* Lite key function `[min(g, rhs) + h, min(g, rhs)]`, plus hash maps for `g`-values, `rhs`-values, states, and transitions for O(1) lookup during updates.
6. **Incremental replanning** — `update_goal()`, `update_bad_states()`, `update_transition()`, and `add_transition()` each locally mark the affected states as inconsistent and push them back onto the priority queue; `replan()` then only re-expands the states whose values actually changed, rather than restarting the search.

## Project Structure

```
safe-semantic-planner/
├── app.py                      # Streamlit UI: run/replan, visualize graph, view metrics
├── planner/
│   ├── models.py                # State, Transition, PlanningProblem, PlanningResult
│   └── dstar_lite.py             # DStarLite planner (plan, replan, update_* methods)
│   └──safety.py
│   └──heuristic.py              
├── experiments/
│   ├── test_cases.py             # Builds the six assignment test-case problems
│   └── benchmark.py               # run_benchmark(): runs all test cases, collects metrics
├── tests/                          # Unit tests for planner correctness
├── test_run.py                     # Script: run the planner on a single test case
├── test_incremental.py             # Script: demonstrate incremental replanning
├── requirements.txt
└── README.md
```

## Installation

```bash
git clone https://github.com/sreelekshmi-harikumar/safe-semantic-planner.git
cd safe-semantic-planner
python -m venv venv
source venv/bin/activate        # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

**Dependencies:** `streamlit`, `numpy`, `pandas`, `plotly`, `networkx`

## Usage

### Run a single test case from the command line

```bash
python test_run.py
```

### Demonstrate incremental replanning

```bash
python test_incremental.py
```

### Launch the interactive app locally

```bash
streamlit run app.py
```

## Interactive Streamlit App

The app lets you:

1. Select one of the six assignment test cases.
2. Tune the cost / safety / reliability weights.
3. Run the planner and view the resulting path overlaid on the Cartesian state-space graph.
4. Disable existing transitions, add new ones, change the goal state, or change the set of bad states — all from the sidebar.
5. Trigger **Incremental Replan** to update the existing D\* Lite search instead of rebuilding it.
6. Inspect planning statistics: total cost, minimum safety distance, reliability, explored states, planning time, and replanning time.
7. Run the full experimental evaluation across all test cases with one click.

In the graph view: the **initial state** is blue, the **goal state** is gold, **bad states** are red, and the **selected path** is highlighted in green.

## Test Cases

| # | Test Case | What It Verifies |
|---|---|---|
| 1 | Basic Reachability | Planner returns the unique valid path `S → A → B → G` |
| 2 | Bad State Avoidance | Planner rejects `S → A → X → G` (X is bad) and selects `S → C → D → G` |
| 3 | Safety Margin | Planner balances a cheaper-but-riskier path against a costlier-but-safer one, controlled by the safety weight |
| 4 | Dynamic Transition | When `(A, G)` becomes unavailable mid-run, the planner finds an alternative route |
| 5 | Goal Update | When the goal changes, the planner revises the path via incremental replanning rather than a full rebuild |
| 6 | Transition Addition | A newly inserted shortcut transition is discovered and used if it improves the solution |

## Experimental Evaluation

`experiments/benchmark.py` runs all six test cases and records, for each:

- Goal success rate
- Number of bad states visited (expected: zero)
- Total path cost
- Minimum distance to bad states (safety margin)
- Cumulative reliability
- Number of explored states
- Planning time (ms)
- Replanning time (ms), where applicable

These results are viewable either by running the benchmark script directly or via the **"Run All Experiments"** button in the Streamlit app.

## Dynamic Replanning

The assignment requires the planner to handle a changing environment efficiently:

| Environment Change | Planner Response |
|---|---|
| Goal state changes | `DStarLite.update_goal(new_goal)` re-keys the priority queue relative to the new goal and reuses existing `g`/`rhs` values elsewhere in the graph |
| Bad states change | `DStarLite.update_bad_states(new_bad_states)` recomputes safety penalties only for affected transitions |
| Transition becomes unavailable/available | `DStarLite.update_transition(id, available)` marks affected states inconsistent |
| New transition added | `DStarLite.add_transition(transition)` inserts the edge and re-evaluates only the states it touches |

After any of the above, calling `planner.replan()` repairs the search incrementally rather than calling `plan()` again from scratch — this is the central advantage of D\* Lite over a plain Dijkstra/A\* re-run on every update.

## Complexity Analysis

- **Time complexity:** Each priority-queue operation (insert/update/pop) costs `O(log n)` for `n` states. A full initial computation is `O(E log n)` for `E` transitions. An incremental update after a local change (goal move, single transition toggle, single bad-state change) only re-expands the states whose `g`/`rhs` values become inconsistent, which in practice is a small local neighborhood rather than the whole graph — asymptotically bounded by `O(k log n)` where `k` is the number of affected states, `k ≪ n` for local changes.
- **Space complexity:** `O(n + E)` — one `g` and `rhs` value per state, plus storage for all states and transitions, plus the priority queue which holds at most `n` entries.



---
## Author

**Sreelekshmi Harikumar**  
CSE - Third year  
PCCST503 – Machine Learning, Assignment 1  
