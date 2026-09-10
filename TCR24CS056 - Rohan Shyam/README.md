# Safe Semantic Planner

A minimal C++ implementation of a safe semantic planner using D* Lite for path planning and incremental dynamic replanning in finite Cartesian state spaces with geometric safety guarantees.

## Overview

In autonomous navigation and semantic mission planning, agents operate over discrete states embedded in continuous metric spaces. Safe path planning requires finding cost-optimal paths from an initial state to a goal while strictly avoiding forbidden "bad" states and dynamically adapting to environment changes (e.g., edge invalidation, goal shifts, or newly discovered shortcuts) without recomputing the entire search graph from scratch.

This project implements a clean, academic-focused safe semantic planner using **D\* Lite** (Koenig & Likhachev) written in standard C++17.

## Features

- **D\* Lite Path Planning**: Goal-directed, backward-search incremental heuristic planning.
- **Cartesian State Embeddings**: Supports arbitrary dimensional state spaces ($\mathbb{R}^D$).
- **Directed Transitions**: Edges with customizable cost, safety, reliability, and availability.
- **Strict Bad-State Avoidance**: Guarantees no path traverses forbidden states.
- **Euclidean Safety Distance**: Computes geometric clearance to the closest obstacle state:
  $$\text{dist}(s) = \min_{b \in \text{BadStates}} \|\mathbf{e}_s - \mathbf{e}_b\|_2$$
- **Cost-Based Path Selection**: Minimizes cumulative transition cost with optional safety margin weighting.
- **Incremental Dynamic Replanning**: Efficiently updates shortest paths when transitions change or goals update.

## Algorithm

The planner implements the standard backward-search D\* Lite algorithm:
1. **$g(s)$ and $rhs(s)$ Values**:
   - $g(s)$: current estimate of shortest path distance from state $s$ to $s_{\text{goal}}$.
   - $rhs(s)$: one-step lookahead cost based on successors:
     $$rhs(s) = \begin{cases} 0 & \text{if } s = s_{\text{goal}} \\ \min_{s' \in \text{Succ}(s)} \big(c(s, s') + g(s')\big) & \text{otherwise} \end{cases}$$
2. **Consistency**:
   - Consistent: $g(s) = rhs(s)$
   - Overconsistent: $g(s) > rhs(s)$ (cost decreased)
   - Underconsistent: $g(s) < rhs(s)$ (cost increased/invalidated)
3. **Priority Queue & Keys**:
   $$k(s) = \begin{bmatrix} \min(g(s), rhs(s)) + h(s_{\text{start}}, s) + k_m \\ \min(g(s), rhs(s)) \end{bmatrix}$$
   Inconsistent states are maintained in a min-priority queue ordered lexicographically by $k(s)$.
4. **Heuristic**:
   Arbitrary-dimension Euclidean distance:
   $$h(a, b) = \|\mathbf{e}_a - \mathbf{e}_b\|_2 = \sqrt{\sum_{i=0}^{D-1} (a_i - b_i)^2}$$
5. **Incremental Replanning**:
   When an edge $(u, v)$ changes, only affected vertices are updated via `UpdateVertex(u)`. Calling `replan()` resumes `ComputeShortestPath()` from the existing priority queue, reusing previous search effort instead of re-expanding the entire graph.

## Project Structure

```text
safe-semantic-planner/
├── CMakeLists.txt     # Build configuration (C++17)
├── README.md          # Project documentation and design analysis
├── planner.hpp        # Data structures and Planner class declarations
├── planner.cpp        # D* Lite and safety calculation implementation
├── main.cpp           # 6 demonstration scenarios with detailed metrics
└── tests.cpp          # Automated unit tests and edge cases
```

## Build

Requirements: C++17 compiler (GCC $\ge 8$, Clang $\ge 7$, or MSVC) and CMake $\ge 3.14$.

```bash
cmake -S . -B build
cmake --build build
```

## Run

### Main Demonstration
Runs the six required assignment scenarios:
```bash
./build/safe_planner
```

### Automated Unit Tests
Runs deterministic test assertions covering all scenarios and edge cases:
```bash
./build/planner_tests
```

## Test Cases

The test suite in `main.cpp` demonstrates the six canonical scenarios:

1. **Basic Reachability**:
   - Path: $S \to A \to B \to G$
   - Verifies standard path extraction and cost accumulation.
2. **Bad State Avoidance**:
   - Primary shortest route contains bad state $X$ ($S \to A \to X \to G$).
   - Planner detects $X \in \text{badStates}$, prunes transitions, and selects safe alternative $S \to C \to D \to G$.
3. **Safety Margin (Cost vs Safety Trade-off)**:
   - Evaluates two candidate paths:
     - Path 1 ($S \to N_{\text{cheap}} \to G$): Cost = 5.0, Safety = 0.2 (close to bad state).
     - Path 2 ($S \to N_{\text{safe}} \to G$): Cost = 8.0, Safety = 2.0 (far from bad state).
   - Shows pure cost minimization vs safety-aware path selection ($\lambda = 1.0$).
4. **Dynamic Transition**:
   - Initial plan: $S \to A \to G$ (Cost: 2.0).
   - Action: Transition $A \to G$ becomes unavailable.
   - Planner incrementally replans to $S \to B \to G$ (Cost: 4.0).
5. **Goal Update**:
   - Initial target: $G_1$ ($S \to A \to G_1$, Cost: 2.0).
   - Action: Goal changes to $G_2$.
   - Planner re-roots and replans to $S \to B \to G_2$ (Cost: 3.0).
6. **Transition Addition**:
   - Initial route: $S \to A \to B \to G$ (Cost: 6.0).
   - Action: Shortcut transition $S \to G$ (Cost: 2.5) is discovered and added.
   - Planner incrementally incorporates shortcut and replans to $S \to G$ in $0.5\,\mu s$ (1 expansion).

## Evaluation

The planner tracks and prints the following metrics for every run:
- **Total Path Cost**: Cumulative edge cost $\sum_{e \in \text{path}} c(e)$.
- **Safety Score**: Minimum geometric clearance to bad states across all states on the path:
  $$\text{safetyScore} = \min_{s \in \text{statePath}} \text{dist}(s)$$
- **Planning / Replanning Time**: Execution time measured in microseconds ($\mu s$) using high-resolution timers.
- **States Explored**: Total vertex expansions popped during `ComputeShortestPath()`.
- **Bad States Visited**: Number of forbidden states present in the final path (guaranteed to be 0).

## Complexity

- **Time Complexity**:
  - Initial planning: $O(|V| \log |V| + |E|)$, bounded by $A^*$ backward search.
  - Replanning: $O(|V_{\text{inconsistent}}| \log |V| + |E_{\text{affected}}|)$ where $|V_{\text{inconsistent}}| \ll |V|$ for localized environment changes.
  - Safety score calculation: $O(|P| \cdot |\text{badStates}| \cdot D)$ where $|P|$ is path length and $D$ is embedding dimension.
- **Space Complexity**:
  - $O(|V| + |E|)$ for graph adjacency, priority queue, and lookup hash maps.

## Limitations

- **Discrete Transitions**: Operates on directed graph topologies rather than continuous non-holonomic trajectory generation.
- **Heuristic Admissibility**: Requires the Euclidean heuristic to satisfy $h(u, v) \le c^*(u, v)$.
- **Re-rooting on Goal Change**: In backward D\* Lite, altering the goal state requires re-initializing the search root ($rhs(s_{\text{goal}}) = 0$).

## Future Work

- Continuous trajectory smoothing and dubins/spline safety corridor generation.
- Dynamic moving obstacle tracking with time-expanded semantic states.
- Multi-objective Pareto frontier exploration for combined cost, safety, and reliability metrics.
