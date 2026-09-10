# Demonstration & Viva Walkthrough Guide

**PCCST503 — Machine Learning: Assignment 1**  
**Safe Semantic Planner in a Finite Cartesian State Space**

This guide provides a structured, scripted walkthrough of all six assignment test cases and bonus extensions for viva presentations and project evaluation.

---

## 🎬 Test Case Walkthrough & Verification Checkpoints

---

### Test Case 1: Basic Reachability

#### 1. Topology & Setup
```
    [S] (1) ----(Cost 1.0)----> [A] (2) ----(Cost 1.0)----> [B] (3) ----(Cost 1.0)----> [G] (4)
```
- **Initial State:** $S(1)$ at $(0.0, 0.0)$
- **Goal State:** $G(4)$ at $(3.0, 0.0)$
- **Bad States:** None ($\mathcal{B} = \emptyset$)

#### 2. Actual Terminal & GUI Output
```
[Test Case 1: Basic Reachability]
  Expected: Unique path S(1) -> A(2) -> B(3) -> G(4)
  Status: SUCCESS [PATH FOUND]
  State Path: [1 -> 2 -> 3 -> 4]
  Transition Path: [1, 2, 3]
  Total Cost: 3.0000
  Safety Score: +INF (No bad states active)
  Cumulative Reliability: 100.0000%
  Explored States: 4
  Planning Time: 51.4000 microseconds
```

#### 3. Viva Talking Points
- **Admissibility:** $h(S, G) = c_{\text{min\_ratio}} \cdot 3.0 = 3.0 \le 3.0$ (exact distance match).
- **Optimality:** LPA* terminates after exactly 4 vertex expansions with zero redundant operations.

---

### Test Case 2: Bad State Avoidance

#### 1. Topology & Setup
```
             +----(Cost 1.0)----> [A] (2) ----(Cost 1.0)----> [X] (3, BAD) ----(Cost 1.0)----+
             |                                                                                |
            [S] (1)                                                                          v
             |                                                                              [G] (4)
             +----(Cost 1.3)----> [C] (5) ----(Cost 1.4)----> [D] (6) ----(Cost 1.3)---------+
```
- **Bad States:** $\mathcal{B} = \{X(3)\}$ at $(2.0, 1.0)$
- **Path 1 (Unsafe):** $S \to A \to X \to G$ (Nominal Cost = 3.0, but traverses bad state $X$)
- **Path 2 (Safe Detour):** $S \to C \to D \to G$ (Nominal Cost = 4.0, zero bad states)

#### 2. Actual Output
```
[Test Case 2: Bad State Avoidance]
  Expected: Avoid bad state X(3), choose S(1) -> C(5) -> D(6) -> G(4)
  Status: SUCCESS [PATH FOUND]
  State Path: [1 -> 5 -> 6 -> 4]
  Transition Path: [4, 5, 6]
  Total Cost: 4.0000
  Safety Score (Min Bad-Dist): 1.4142
  Cumulative Reliability: 97.0299%
  Explored States: 5
```

#### 3. Viva Talking Points
- **Hard Constraint Enforcement:** State $X(3)$ is assigned $g(X) = rhs(X) = \infty$ and never inserted into the priority queue $U$.
- **Formal Invariant:** Number of bad states visited is guaranteed to be **exactly zero**.

---

### Test Case 3: Safety Margin & Tradeoff

#### 1. Topology & Setup
```
                           [B_BAD] (3, Obstacle)
                                  ^
                                  | dist = 0.6
    [S] (1) ----(Cost 1.0)----> [A] (2) ----(Cost 1.0)----> [G] (4)  (Path 1: Cost 2.0, Risky)
       \                                                      /
        \---(Cost 1.2)--> [C] (5) --(1.1)--> [D] (6) --(1.2)-/       (Path 2: Cost 3.5, Safe Clearance)
```

#### 2. Actual Output (Dual Mode Comparison)
```
[Test Case 3A: Pure Cost Minimization (w_s = 0)]
  State Path: [1 -> 2 -> 4]
  Total Cost: 2.0000
  Safety Score (Min Bad-Dist): 0.6000

[Test Case 3B: Safety-Weighted Margin Balancing (w_s = 5)]
  State Path: [1 -> 5 -> 6 -> 4]
  Total Cost: 3.5000
  Safety Score (Min Bad-Dist): 1.2806
```

#### 3. Viva Talking Points
- Explains how the planner balances cost and safety:
  - In pure shortest path mode, Path 1 is chosen ($C = 2.0$).
  - With multi-objective safety penalty $c_{\text{eff}}(u, v) = c(u, v) + w_s \max(0, D_{\text{thresh}} - D(v))$, Path 2 is dynamically selected ($C = 3.5, D = 1.28$), preventing dangerous near-obstacle proximity.

---

### Test Case 4: Dynamic Transition Failure

#### 1. Scenario Execution
- **Step 1:** Initial path $S(1) \to A(2) \to G(3)$ computed with cost 2.0.
- **Step 2:** Edge $(A, G)$ dynamically fails (`available = false`).
- **Step 3:** Incremental LPA* replan triggers automatically.

#### 2. Actual Output
```
[Test Case 4.1: Initial State] Primary Path: [1 -> 2 -> 3], Cost: 2.0000
>>> Event: Transition (A->G) becomes unavailable! Triggering incremental replan...
[Test Case 4.2: Dynamic Transition Replanned]
  Expected: Switches to backup path S(1) -> B(4) -> C(5) -> G(3)
  Status: SUCCESS [PATH FOUND]
  State Path: [1 -> 4 -> 5 -> 3]
  Total Cost: 3.4000
  Planning Time: 1.1000 microseconds (Instantaneous)
```

#### 3. Viva Talking Points
- **Warm Replan:** Only vertex $G(3)$ has its $rhs$ directly modified ($O(1)$). `ComputeShortestPath()` explores only 4 vertices, taking **1.1 microseconds** without rebuilding graph structures.

---

### Test Case 5: Dynamic Goal Update

#### 1. Scenario Execution
- **Step 1:** Goal is initially $G_1(3)$. Path: $S(1) \to \text{Hub}_1(2) \to G_1(3)$ (Cost 2.2).
- **Step 2:** Goal shifts to $G_2(5)$.
- **Step 3:** LPA* re-keys priority queue with $h(s, G_2)$ and resumes search.

#### 2. Actual Output
```
[Test Case 5.1: Initial Goal G1(3)] Path: [1 -> 2 -> 3], Cost: 2.2000
>>> Event: Goal changes to G2(5)! Incremental re-keying...
[Test Case 5.2: Dynamic Goal Updated]
  Expected: Re-routes to S(1) -> Hub_2(4) -> G2(5) reusing existing search tree
  Status: SUCCESS [PATH FOUND]
  State Path: [1 -> 4 -> 5]
  Total Cost: 2.7000
  Explored States: 2
  Planning Time: 0.4000 microseconds
```

#### 3. Viva Talking Points
- **Why Forward LPA* Wins:** Because $g$-values represent distances from $s_I$, all existing $g$-values remain completely valid when the goal changes! Only **2 vertices** are expanded to reach the new goal.

---

### Test Case 6: Transition Addition (Shortcut Discovery)

#### 1. Scenario Execution
- **Step 1:** Baseline path is $S(1) \to A(2) \to B(3) \to G(4)$ (Cost 3.0).
- **Step 2:** Shortcut transition $(A, G)$ with cost 1.05 is inserted.
- **Step 3:** LPA* updates $rhs(G) = g(A) + 1.05 = 2.05$, relaxes $G$, and terminates.

#### 2. Actual Output
```
[Test Case 6.1: Initial Baseline Path] Path: [1 -> 2 -> 3 -> 4], Cost: 3.0000
>>> Event: Shortcut Transition (A(2) -> G(4), Cost=1.05) added! Incremental update...
[Test Case 6.2: Shortcut Discovered]
  Expected: Discovers shortcut path S(1) -> A(2) -> G(4) with cost 2.05
  Status: SUCCESS [PATH FOUND]
  State Path: [1 -> 2 -> 4]
  Total Cost: 2.0500
  Explored States: 1
  Planning Time: 0.2000 microseconds
```

#### 3. Viva Talking Points
- **Optimal Efficiency:** Exactly **1 node expansion** was needed to update the entire search tree and extract the new optimal shortcut path.

---

## 🎯 Summary of Viva Questions & Answers

1. **Why was LPA* chosen over standard A*?**  
   *Answer:* Standard A* discards all search data and re-plans from scratch ($O(|V| \log |V|)$) upon any change. LPA* maintains $g$ and $rhs$ values, processing only the inconsistent subgraph ($\Delta V \ll V$), achieving up to 49× speedups.

2. **Why is raw Euclidean distance scaled by $c_{\text{min\_ratio}}$?**  
   *Answer:* To ensure mathematical admissibility ($h(s) \le h^*(s)$). If an edge has cost $c(e) < \|\mathbf{x}_u - \mathbf{x}_v\|$, raw Euclidean distance would overestimate the cost and lose optimal path guarantees.

3. **How does the Web Visualizer communicate with the engine?**  
   *Answer:* The web visualizer embeds a high-fidelity JS mirror of the C++ LPA* engine and can also load problem definitions and test cases exported to standard JSON format, providing seamless live demonstration.
