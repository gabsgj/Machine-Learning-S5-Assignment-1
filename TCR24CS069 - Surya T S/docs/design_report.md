# Safe Semantic Planner — Detailed Design Report

## PCCST503 Machine Learning — Assignment

**Author:** Surya T S  
**University Registration Number:** TCR24CS069  
**Algorithm:** D* Lite (Koenig & Likhachev, 2002)  
**Language:** C++14  
**Date:** August 2026

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Problem Formulation](#2-problem-formulation)
3. [Algorithm Selection & Justification](#3-algorithm-selection--justification)
4. [State Representation](#4-state-representation)
5. [Data Structures](#5-data-structures)
6. [Heuristic Function Design](#6-heuristic-function-design)
7. [Safety Computation](#7-safety-computation)
8. [Multi-Objective Cost Function](#8-multi-objective-cost-function)
9. [D* Lite Algorithm — Detailed Description](#9-d-lite-algorithm--detailed-description)
10. [Dynamic Environment Handling](#10-dynamic-environment-handling)
11. [Software Architecture](#11-software-architecture)
12. [Time Complexity Analysis](#12-time-complexity-analysis)
13. [Space Complexity Analysis](#13-space-complexity-analysis)
14. [Test Cases & Experimental Results](#14-test-cases--experimental-results)
15. [Discussion](#15-discussion)
16. [Bonus Features](#16-bonus-features)
17. [Conclusion](#17-conclusion)
18. [References](#18-references)

---

## 1. Introduction

Path planning in dynamic environments is a fundamental problem in artificial intelligence and robotics. Given a graph where edges may appear, disappear, or change cost over time, a planner must efficiently compute safe, optimal routes from a start state to a goal state while avoiding dangerous regions.

This report presents the design and implementation of a **Safe Semantic Planner** — a C++ system that computes paths in a finite Cartesian state space R^d. The planner uses the **D\* Lite** algorithm enhanced with a multi-objective cost function that simultaneously optimizes:

- **Path cost** (minimize)
- **Safety distance from bad states** (maximize)
- **Transition reliability** (maximize)

The system supports **incremental replanning** when the environment changes, enabling efficient adaptation without rebuilding all data structures from scratch.

---

## 2. Problem Formulation

### 2.1 State Space

Let **S = {s_1, s_2, ..., s_n}** be a finite set of states embedded in Cartesian space R^d. Each state s_i has a coordinate vector:

```
s_i = (x_1, x_2, ..., x_d)
```

The dimensionality d is arbitrary (our test cases use d=2 for clarity, but the implementation is dimension-agnostic).

### 2.2 Transitions

A directed transition t = (s_i, s_j) connects state s_i to state s_j. Each transition carries four attributes:

| Attribute | Type | Description |
|-----------|------|-------------|
| `cost` | double >= 0 | Resource expenditure to traverse this edge |
| `safety` | double | Safety score of the transition itself |
| `reliability` | double [0,1] | Probability the transition completes successfully |
| `available` | bool | Whether the transition can currently be used |

### 2.3 Constraints

The planner receives:
- **Initial state s_I** — where the agent starts
- **Goal state s_G** — where the agent must arrive
- **Bad states B = {b_1, ..., b_k}** — states that must NEVER be visited

### 2.4 Objective Function

The planner maximizes:

```
Score(P) = alpha*G - beta*C + gamma*D + delta*R
```

Where:
- **G** = 1 if the goal is reached, 0 otherwise (weighted by alpha)
- **C** = sum of transition costs along the path
- **D** = min distance from any path state to the nearest bad state
- **R** = sum of transition reliabilities along the path
- **alpha, beta, gamma, delta** = tunable weight parameters

### 2.5 Dynamic Environment

The following changes may occur at any time:
1. Goal state moves to a different state
2. Bad states are added or removed
3. Transitions become available or unavailable
4. New shortcut transitions are inserted

The planner must handle all of these efficiently.

---

## 3. Algorithm Selection & Justification

### 3.1 Candidates Considered

| Algorithm | Strengths | Weaknesses |
|-----------|-----------|------------|
| **A\*** | Simple, optimal, well-understood | No incremental replanning; full re-search needed |
| **LPA\*** | Incremental; reuses search history | Forward search; goal changes require full restart |
| **D\* Lite** | Incremental; backward search supports goal changes | Slightly more complex implementation |
| **D\*** (original) | Handles dynamic environments | Complex, harder to implement correctly |

### 3.2 Why D\* Lite

**D\* Lite** was selected because:

1. **Backward search direction**: D\* Lite searches from goal toward start. This means when the agent discovers a blocked edge nearby, only local path segments need repair. This is exactly what our problem requires.

2. **Incremental replanning**: When edge costs change, D\* Lite only re-expands nodes whose optimal cost-to-goal has been affected. Unaffected portions of the search tree are preserved.

3. **Goal change support**: Since the search tree is rooted at the goal, changing the goal and re-initializing the search is natural and correct.

4. **Optimality guarantee**: D\* Lite produces provably shortest paths (adapted to our composite cost function).

5. **Proven correctness**: The algorithm has a rigorous mathematical foundation (Koenig & Likhachev, 2002) and is widely used in robotics (Mars rovers, autonomous vehicles).

---

## 4. State Representation

```cpp
class State {
public:
    uint64_t id;                    // Unique identifier
    std::vector<double> embedding;  // d-dimensional coordinate vector
};
```

### Design Decisions

- **uint64_t id**: Supports up to 2^64 unique states (more than sufficient for any practical application).
- **vector<double> embedding**: Arbitrary-dimension support. The Euclidean distance computation, heuristic function, and safety distance calculations all work generically over this vector, making the planner applicable to 2D, 3D, or higher-dimensional spaces without modification.

### State Lookup

States are indexed in an `unordered_map<uint64_t, const State*>` providing O(1) average-case lookup by ID. This is critical because the D\* Lite algorithm frequently queries state embeddings for heuristic computation.

---

## 5. Data Structures

### 5.1 Adjacency Lists

```
Forward:  adjForward_[state_id] = [indices of transitions FROM state_id]
Reverse:  adjReverse_[state_id] = [indices of transitions TO state_id]
```

Both are `unordered_map<uint64_t, vector<size_t>>`.

**Why both directions?**
- **Forward adjacency**: Used during path extraction (greedy forward trace from start to goal) and successor queries.
- **Reverse adjacency**: Used by D\* Lite for predecessor queries. Since D\* Lite searches backward (goal to start), it needs to know which states have transitions leading TO a given state.

### 5.2 Priority Queue (Open Set)

The D\* Lite priority queue requires:
- Efficient minimum-key extraction: O(log n)
- Efficient key updates (remove + re-insert): O(log n)
- Membership queries: O(1)

**Implementation**: `std::set<pair<KeyPair, uint64_t>>` with a companion `unordered_map<uint64_t, KeyPair>`.

```cpp
struct KeyPair {
    double k1;  // min(g, rhs) + h(s_start, s) + k_m
    double k2;  // min(g, rhs)
    // Lexicographic comparison
};
```

The `set` provides O(log n) insert/erase/min operations. The companion `unordered_map` provides O(1) lookup to find a state's current key for removal before re-insertion.

### 5.3 G-Values and RHS-Values

```cpp
unordered_map<uint64_t, double> g_;    // Cost-to-goal (committed)
unordered_map<uint64_t, double> rhs_;  // One-step lookahead cost-to-goal
```

**Interpretation:**
- **g(s)**: The current best known cost from state s to the goal.
- **rhs(s)**: The one-step lookahead value, computed as:
  ```
  rhs(s) = min over successors v of { effectiveCost(s,v) + g(v) }
  ```
  For the goal state: `rhs(s_G) = 0`.

**Consistency**: A state is:
- **Consistent** when g(s) = rhs(s) — its cost is optimal
- **Overconsistent** when g(s) > rhs(s) — a better path was found
- **Underconsistent** when g(s) < rhs(s) — a path became worse

### 5.4 Safety Map

```cpp
unordered_map<uint64_t, double> safetyMap_;  // state_id -> min distance to bad
```

Precomputed for all states at initialization. Updated when bad states change.

### 5.5 Bad State Set

```cpp
unordered_set<uint64_t> badStateSet_;  // O(1) membership check
```

---

## 6. Heuristic Function Design

The heuristic h(a, b) estimates the cost from state a to state b. For D\* Lite (backward search), we need h(s_start, s) — the estimated cost from the start to state s.

### Formula

```
h(a, b) = beta * ||a.embedding - b.embedding||_2 = beta * sqrt(sum_i (a_i - b_i)^2)
```

This is the Euclidean distance in R^d scaled by the cost weight $\beta$.

### Properties

1. **Admissibility**: The scaled Euclidean distance never overestimates the true composite path cost, because:
   - Base transition costs satisfy $c \ge \text{dist}$ and $\beta > 0$
   - Penalty terms (proximity $\gamma/D_{\min}$ and unreliability $\delta(1-R)$) are strictly non-negative additions
   - Therefore, $\beta \cdot \text{dist}(a, b)$ is a guaranteed lower bound on the optimal composite cost.

2. **Consistency (Monotonicity)**: For any states a, b, c:
   ```
   h(a, c) <= h(a, b) + cost_eff(b, c)
   ```
   This follows from the triangle inequality of Euclidean distance.

3. **Dimension-agnostic**: Works for any dimension $d$ without modification.

These properties guarantee that D\* Lite will produce optimal paths.

---

## 7. Safety Computation

### 7.1 Safety Distance

For each state s in S, the safety distance is:

```
D_min(s) = min_{b in B} ||s.embedding - b.embedding||_2
```

This represents the minimum Euclidean distance from state s to the nearest bad state.

### 7.2 Safety Map Construction

```
Algorithm: ComputeSafetyMap(S, B)
1. Collect embeddings of all bad states
2. For each state s in S:
     safetyMap[s.id] = min distance from s to any bad state
3. Return safetyMap
```

Time: O(|S| * |B|)  
Space: O(|S|)

### 7.3 Integration with Planning

Safety is integrated at **two levels**:

1. **Edge-level**: The effective cost of each transition includes a proximity penalty term $\gamma / D_{\min}$. Edges leading to states near bad states receive higher penalty.

2. **Path-level**: The `PlanningResult::safetyScore` reports the minimum safety distance along the entire computed path, giving a global measure of how "safe" the path is.

### 7.4 Bad State Enforcement

Bad states are **strictly forbidden** (not just penalized). This is enforced by:
- Setting `effectiveCost(t) = INFINITY` for any transition where `t.to` is a bad state
- Excluding bad states from predecessor/successor queries

---

## 8. Multi-Objective Cost Function

### 8.1 Composite Edge Cost

The four objectives are combined into an additive composite edge cost:

```
effectiveCost(t) = beta * t.cost + gamma * (1.0 / D_min(t.to)) + delta * (1.0 - t.reliability)
```

Where:
- `beta` (default 1.0): Weight on transition cost. Higher = penalize high traversal cost.
- `gamma` (default 0.5): Weight on hazard proximity penalty ($1/D_{\min}$). Higher = strongly avoid routes passing close to bad states.
- `delta` (default 0.3): Weight on unreliability ($1 - R$). Higher = prefer reliable transitions.

### 8.2 Constraints on Edge Cost

- **Strict Positivity**: The effective cost is clamped to $\epsilon = 10^{-6}$ to ensure strictly positive edge weights, satisfying D\* Lite non-negative weight requirements.
- **Infinity for forbidden states**: If `t.to` is a bad state or `t.available` is false, the effective cost is $+\infty$.

### 8.3 Why This Approach Works

Combining objectives into a scalar cost allows us to use standard single-objective D\* Lite without modification to the core algorithm. The weights provide a tunable knob:

| Weight Setting | Behavior |
|----------------|----------|
| High beta, low gamma | Prefer cheapest path (ignore safety) |
| Low beta, high gamma | Prefer safest path (ignore cost) |
| Balanced | Tradeoff between cost and safety |

---

## 9. D\* Lite Algorithm — Detailed Description

### 9.1 Overview

D\* Lite maintains two values for each state:
- **g(s)**: Committed cost-to-goal
- **rhs(s)**: One-step lookahead cost-to-goal

The algorithm works in three phases: **Initialize**, **ComputeShortestPath**, and **ExtractPath**.

### 9.2 Initialization

```
procedure Initialize():
    for all s in S:
        g(s) = INFINITY
        rhs(s) = INFINITY
    rhs(s_goal) = 0
    k_m = 0
    Insert s_goal into OpenSet with key CalculateKey(s_goal)
```

### 9.3 Key Calculation

```
procedure CalculateKey(s):
    k1 = min(g(s), rhs(s)) + h(s_start, s) + k_m
    k2 = min(g(s), rhs(s))
    return [k1, k2]
```

Keys are compared lexicographically: first by k1, then by k2 as tiebreaker.

### 9.4 UpdateVertex

```
procedure UpdateVertex(u):
    if u != s_goal:
        rhs(u) = min over successors v of { effectiveCost(u, v) + g(v) }
    
    if u is in OpenSet:
        remove u from OpenSet
    
    if g(u) != rhs(u):
        insert u into OpenSet with key CalculateKey(u)
```

### 9.5 ComputeShortestPath

```
procedure ComputeShortestPath():
    while OpenSet is not empty AND
          (TopKey() < CalculateKey(s_start) OR rhs(s_start) != g(s_start)):
        
        u = state with minimum key in OpenSet
        k_old = TopKey()
        k_new = CalculateKey(u)
        
        if k_old < k_new:
            // Key changed — reinsert with updated key
            Update u's key in OpenSet to k_new
        
        else if g(u) > rhs(u):
            // OVERCONSISTENT: a better path was found
            g(u) = rhs(u)
            Remove u from OpenSet
            for each predecessor p of u:
                UpdateVertex(p)
        
        else:
            // UNDERCONSISTENT: path became worse
            g(u) = INFINITY
            UpdateVertex(u)
            for each predecessor p of u:
                UpdateVertex(p)
```

### 9.6 Path Extraction

After ComputeShortestPath terminates, the optimal path is extracted by greedy forward traversal:

```
procedure ExtractPath():
    path = [s_start]
    current = s_start
    while current != s_goal:
        next = argmin over successors v of { effectiveCost(current, v) + g(v) }
        path.append(next)
        current = next
    return path
```

### 9.7 Termination Conditions

ComputeShortestPath terminates when BOTH conditions hold:
1. The start state is consistent: g(s_start) = rhs(s_start)
2. The minimum key in OpenSet >= CalculateKey(s_start)

If the OpenSet is empty and g(s_start) = INFINITY, no path exists.

---

## 10. Dynamic Environment Handling

### 10.1 Goal State Change

**Method**: `updateGoal(newGoal)`

When the goal changes, the backward search tree is re-rooted:
1. Store new goal ID
2. Re-initialize all g/rhs values to INFINITY
3. Set rhs(newGoal) = 0
4. Insert newGoal into OpenSet
5. Run ComputeShortestPath

This reuses the existing adjacency lists and safety map.

**Complexity**: O((n+m) log n) — same as initial plan, but adjacency list construction is skipped.

### 10.2 Bad State Changes

**Methods**: `addBadState(id)`, `removeBadState(id)`

When bad states change:
1. Update the bad state set
2. Recompute the safety map (all safety distances change)
3. For bad state addition: set g and rhs of the bad state to INFINITY
4. Update all non-bad vertices (their effective costs changed)
5. Run ComputeShortestPath

**Complexity**: O(n*b + (n+m) log n)

### 10.3 Transition Availability Toggle

**Method**: `setTransitionAvailability(transId, available)`

When a transition is toggled:
1. Update the availability flag
2. Re-initialize the search state
3. Run ComputeShortestPath

**Complexity**: O((n+m) log n)

### 10.4 Transition Addition (Shortcut)

**Method**: `addTransition(t)`

When a new transition is added:
1. Append to transition list
2. Update forward and reverse adjacency lists
3. Call UpdateVertex on the source state (may have a better path now)
4. Run ComputeShortestPath

This is truly incremental — only affected nodes are re-expanded.

**Complexity**: O(k log n) where k = number of affected states.

---

## 11. Software Architecture

![System Architecture](images/system_architecture.png)

### 11.1 Class Diagram

```
                    +----------------+
                    |    Planner     |  (Abstract Interface)
                    |  + plan()      |
                    +-------+--------+
                            |
                            | inherits
                            v
                +----------------------+
                | DStarLitePlanner     |  (Concrete Implementation)
                |                      |
                | + plan()             |
                | + updateGoal()       |
                | + addBadState()      |
                | + removeBadState()   |
                | + setTransition...() |
                | + addTransition()    |
                | + replan()           |
                | + setWeights()       |
                +----------------------+
                     |          |
            uses     |          |    uses
                     v          v
        +----------+     +-----------+
        | safety:: |     |  KeyPair  |
        | utils    |     | (priority)|
        +----------+     +-----------+

    +-------+  +------------+  +-----------------+  +----------------+
    | State |  | Transition |  | PlanningProblem |  | PlanningResult |
    +-------+  +------------+  +-----------------+  +----------------+
```

### 11.2 File Organization

```
Safe_Semantic_Planner/
|
+-- include/                    (Header files)
|   +-- state.h                 State class
|   +-- transition.h            Transition class
|   +-- planning_problem.h      PlanningProblem class
|   +-- planning_result.h       PlanningResult class
|   +-- planner.h               Abstract Planner interface
|   +-- d_star_lite_planner.h   D* Lite implementation header
|   +-- safety_utils.h          Safety utilities header
|
+-- src/                        (Source files)
|   +-- d_star_lite_planner.cpp D* Lite implementation (~500 lines)
|   +-- safety_utils.cpp        Safety computation
|   +-- main.cpp                Test driver (all 6 test cases)
|
+-- tests/
|   +-- test_planner.cpp        Automated test suite (14 tests)
|
+-- demo/
|   +-- index.html              Interactive visual demonstration
|
+-- docs/
|   +-- design_report.md        This document
|   +-- user_manual.md          User manual
|
+-- build/                      (Compiled binaries)
|   +-- safe_planner.exe        Main demo executable
|   +-- test_planner.exe        Test suite executable
|
+-- CMakeLists.txt              CMake build system
+-- build.bat                   Windows build script
```

---

## 12. Time Complexity Analysis

Let n = |S| (states), m = |T| (transitions), b = |B| (bad states), d = embedding dimension.

### 12.1 Initial Planning

| Phase | Complexity |
|-------|-----------|
| Build adjacency lists | O(m) |
| Build state lookup | O(n) |
| Compute safety map | O(n * b * d) |
| Initialize g/rhs | O(n) |
| ComputeShortestPath | O((n + m) log n) |
| Extract path | O(path_length * max_degree) |
| **Total** | **O(n*b*d + (n+m) log n)** |

### 12.2 Incremental Replanning

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| Goal change | O((n+m) log n) | Re-init search, reuse adjacency |
| Add bad state | O(n*b*d + (n+m) log n) | Safety map + full update |
| Remove bad state | O(n*b*d + (n+m) log n) | Safety map + full update |
| Toggle transition | O((n+m) log n) | Re-init search |
| Add transition | O(k log n) | Truly incremental |

### 12.3 Heuristic Evaluation

Each call to h(a, b) computes Euclidean distance in d dimensions: O(d).

Since the heuristic is called O((n+m) log n) times total during ComputeShortestPath, the total heuristic cost is O((n+m) * d * log n).

---

## 13. Space Complexity Analysis

| Component | Space | Notes |
|-----------|-------|-------|
| State storage | O(n * d) | n states, d-dimensional embeddings |
| Transition storage | O(m) | m transition objects |
| Forward adjacency | O(n + m) | Hash map + edge index vectors |
| Reverse adjacency | O(n + m) | Hash map + edge index vectors |
| G-values | O(n) | One double per state |
| RHS-values | O(n) | One double per state |
| Priority queue (set) | O(n) | At most n entries |
| Priority queue (keys) | O(n) | Companion hash map |
| Safety map | O(n) | One double per state |
| Bad state set | O(b) | Hash set |
| State lookup | O(n) | Hash map of pointers |
| **Total** | **O(n*d + m)** | |

---

## 14. Test Cases & Experimental Results

### 14.1 Test Case 1: Basic Reachability

**Setup:**
```
S(0) --c=1--> A(1) --c=1--> B(2) --c=1--> G(3)
```
Coordinates: S=(0,0), A=(1,0), B=(2,0), G=(3,0). No bad states.

**Expected:** Unique path 0 -> 1 -> 2 -> 3, cost = 3.0

**Result:**

| Metric | Value |
|--------|-------|
| Success | YES |
| Path | 0 -> 1 -> 2 -> 3 |
| Cost | 3.00 |
| Safety | INF (no bad states) |
| Explored | 3 |

**Verdict:** PASS — Correct path found.

---

### 14.2 Test Case 2: Bad State Avoidance

**Setup:**
```
        A(1) --> X(2) [BAD]
       /                \
  S(0)                   G(4)
       \                /
        C(3) --> D(5)
```
X(2) is a bad state. Both paths have equal cost.

**Expected:** Path through C and D (avoiding X).

**Result:**

| Metric | Value |
|--------|-------|
| Success | YES |
| Path | 0 -> 3 -> 5 -> 4 |
| Cost | 3.00 |
| Safety | 1.4142 |
| Bad states visited | 0 |

**Verdict:** PASS — Bad state X is strictly avoided.

---

### 14.3 Test Case 3: Safety Margin Tradeoff

**Setup:**
```
        A(1) ..near X.. G(4)     Path 1: cost=2, safety=0.71
       /                           
  S(0)      X(5) [BAD]           
       \                          
        B(2) --> C(3) --> G(4)   Path 2: cost=6, safety=3.54
```

**Expected:** Weight configuration determines path choice.

**Results:**

| Configuration | Path | Cost | Safety |
|---------------|------|------|--------|
| Default (beta=1, gamma=0.5) | 0->1->4 | 2.00 | 0.7071 |
| High safety (beta=1, gamma=3.0) | 0->2->3->4 | 6.00 | 3.5355 |

**Verdict:** PASS — Increasing gamma shifts preference from cost to safety, demonstrating the tunable tradeoff. With gamma=3.0, the planner accepts 3x higher cost to gain 5x more safety distance.

---

### 14.4 Test Case 4: Dynamic Transition Unavailability

**Setup:** Initially S->A->G works. Then transition A->G is disabled.

**Results:**

| Phase | Path | Cost |
|-------|------|------|
| Before (all available) | 0 -> 1 -> 2 | 2.00 |
| After (A->G disabled) | 0 -> 3 -> 4 -> 2 | 4.50 |

**Verdict:** PASS — Planner successfully finds alternative route after edge removal.

---

### 14.5 Test Case 5: Goal Update

**Setup:** Goal changes from G1(3) to G2(4).

**Results:**

| Phase | Goal | Path | Cost |
|-------|------|------|------|
| Original | G1=3 | 0->1->2->3 | 3.00 |
| Updated | G2=4 | 0->1->4 | 2.50 |

**Verdict:** PASS — Planner efficiently adapts to goal change, reusing adjacency lists.

---

### 14.6 Test Case 6: Shortcut Transition Addition

**Setup:** Long path exists. Then shortcut A->G is inserted.

**Results:**

| Phase | Path | Cost |
|-------|------|------|
| Before shortcut | 0->1->2->3->4 | 4.00 |
| After A->G shortcut | 0->1->4 | 2.00 |

**Verdict:** PASS — Planner discovers the shortcut incrementally and reduces cost by 50%.

---

### 14.7 Experimental Benchmark Results & Scenarios

![Benchmark Metrics](images/benchmark_metrics.png)

![Planner Scenarios](images/planner_scenarios.png)

### 14.8 Summary Table

| Test | Goal | Bad Visited | Cost | Safety | Explored | Status |
|------|------|-------------|------|--------|----------|--------|
| TC1 | YES | 0 | 3.00 | INF | 3 | PASS |
| TC2 | YES | 0 | 3.00 | 1.41 | 3 | PASS |
| TC3a (default) | YES | 0 | 2.00 | 0.71 | 3 | PASS |
| TC3b (safe) | YES | 0 | 6.00 | 3.54 | 4 | PASS |
| TC4a (before) | YES | 0 | 2.00 | INF | 3 | PASS |
| TC4b (after) | YES | 0 | 4.50 | INF | 4 | PASS |
| TC5a (G1) | YES | 0 | 3.00 | INF | 4 | PASS |
| TC5b (G2) | YES | 0 | 2.50 | INF | 3 | PASS |
| TC6a (before) | YES | 0 | 4.00 | INF | 5 | PASS |
| TC6b (after) | YES | 0 | 2.00 | INF | 7 | PASS |

### 14.8 Automated Test Suite

14 additional edge-case tests all pass:
- Euclidean distance (2D and 4D)
- Safety map with 0 and 1 bad states
- No path exists
- Start equals goal
- All transitions unavailable
- Bad state blocks only path

**Overall: 14/14 PASS**

---

## 15. Discussion

### 15.1 Strengths

1. **Correctness**: Zero bad states visited across all tests. Optimal paths found in every case.
2. **Flexibility**: Configurable weights allow tuning the cost-safety tradeoff for different applications.
3. **Generality**: Works with arbitrary-dimension embeddings, not just 2D.
4. **Modularity**: Clean separation between the Planner interface and DStarLitePlanner implementation allows swapping algorithms.

### 15.2 Limitations

1. **Safety map recomputation**: When bad states change, the safety map is recomputed for all states — O(n*b). For very large graphs, spatial indexing (k-d trees) could reduce this.
2. **Transition removal**: Disabling transitions triggers a full search re-initialization because unexplored graph regions (with g=INFINITY) prevent correct incremental cost-increase propagation. A bidirectional search could address this.
3. **Multi-objective approximation**: The composite cost function is a weighted sum, which may not find all Pareto-optimal solutions. True multi-objective optimization would require MOPBD* or similar.

### 15.3 Potential Improvements

- **k-d tree for safety computation**: Reduce safety map construction from O(n*b) to O(n*log(b))
- **Parallel search**: Expand multiple nodes simultaneously for large graphs
- **Learning-based heuristic**: Train a neural network to predict cost-to-goal, potentially improving search efficiency

---

## 16. Bonus Features

### 16.1 Incremental Replanning (Implemented)

D\* Lite natively supports incremental replanning. When transitions are added, the `addTransition()` method updates only the affected source vertex and runs ComputeShortestPath, which re-expands only inconsistent nodes.

### 16.2 Multi-Goal Planning (Implemented)

The `updateGoal()` method allows sequential goal changes. Each change reuses the existing adjacency lists and safety map, making it efficient for multi-goal scenarios.

### 16.3 Time-Dependent Transition Availability (Implemented)

The `setTransitionAvailability()` method demonstrates time-dependent edge availability. Transitions can be toggled on/off at any point, and the planner adapts accordingly.

---

## 17. Conclusion

This report presented the design, implementation, and evaluation of a Safe Semantic Planner based on the D\* Lite algorithm. The system successfully:

1. Finds optimal paths in Cartesian state spaces of arbitrary dimension
2. Strictly avoids bad states (zero violations across all 14+ tests)
3. Balances cost, safety distance, and reliability via configurable weights
4. Supports dynamic environment changes with efficient replanning
5. Produces detailed metrics for comprehensive evaluation

The implementation consists of approximately 1,500 lines of C++14 code organized into 15 files, with a clean object-oriented architecture that matches the prescribed interfaces exactly.

---

## 18. References

1. Koenig, S., & Likhachev, M. (2002). "D* Lite." In *Proceedings of the AAAI Conference on Artificial Intelligence* (pp. 476-483).

2. Koenig, S., & Likhachev, M. (2002). "Improved Fast Replanning for Robot Navigation in Unknown Terrain." In *Proceedings of the IEEE International Conference on Robotics and Automation*.

3. Stentz, A. (1994). "Optimal and Efficient Path Planning for Partially-Known Environments." In *Proceedings of the IEEE International Conference on Robotics and Automation*.

4. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100-107.

5. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
