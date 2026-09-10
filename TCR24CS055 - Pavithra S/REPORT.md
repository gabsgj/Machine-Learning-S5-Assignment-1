# PCCST503 – Machine Learning | Assignment 1
# Design of a Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**

---

## 1. Problem Formulation & State Representation

### 1.1 State Space Representation in $\mathbb{R}^d$
Let $S = \{s_1, s_2, \dots, s_n\}$ be a finite set of states embedded in a continuous $d$-dimensional Cartesian space $\mathbb{R}^d$. Each state $s_i$ is represented by a real-valued embedding vector:
$$s_i = (x_1, x_2, \dots, x_d) \in \mathbb{R}^d$$

The planner is given:
- **Initial State**: $s_I \in S$
- **Goal State**: $s_G \in S$
- **Set of Bad States (Obstacles/Hazardous regions)**: $B = \{b_1, b_2, \dots, b_k\} \subset S$
- **Directed Transitions**: $T = \{(s_i, s_j)\}$ with attributes:
  - Nominal Cost: $c(s_i, s_j) > 0$
  - Safety Rating: $\sigma(s_i, s_j) \in [0, 1]$
  - Reliability: $\rho(s_i, s_j) \in (0, 1]$
  - Availability Flag: $a(s_i, s_j) \in \{\text{true}, \text{false}\}$

### 1.2 Multi-Objective Optimization Formulation
A valid path $P = (s_0, s_1, \dots, s_m)$ with $s_0 = s_I$ and $s_m = s_G$ must satisfy:
1. $s_i \notin B, \; \forall s_i \in P$ (Absolute safety constraint).
2. All traversed edges are available: $a(s_i, s_{i+1}) = \text{true}$.
3. Maximize the multi-objective evaluation score:
$$\text{Score}(P) = \alpha G - \beta C + \gamma D + \delta R$$
where:
- $G \in \{0, 1\}$: Goal completion indicator.
- $C = \sum_{i=0}^{m-1} c(s_i, s_{i+1})$: Cumulative nominal transition cost.
- $D = \min_{s \in P} \min_{b \in B} \|s - b\|_2$: Minimum Euclidean clearance to the nearest bad state.
- $R = \prod_{i=0}^{m-1} \rho(s_i, s_{i+1})$: Cumulative path reliability.

---

## 2. Safety Computation & Potential Field Edge Formulation

To guarantee avoidance of bad states and encourage paths with high safety margins, we construct an effective traversal cost $c_{\text{eff}}(u, v)$ for each transition $e = (u, v)$:

$$c_{\text{eff}}(u, v) = \begin{cases}
\infty & \text{if } v \in B \text{ or } a(u, v) = \text{false} \\
w_c \cdot c(u, v) + w_s \cdot \Phi(v, B) - w_r \cdot \ln(\rho(u, v)) & \text{otherwise}
\end{cases}$$

### 2.1 Cartesian Safety Potential Field $\Phi(v, B)$
Let $d(v, B) = \min_{b \in B} \|v - b\|_2$ be the minimum Euclidean distance in $\mathbb{R}^d$ from candidate state $v$ to any obstacle state in $B$. The repulsive potential field is defined as:
$$\Phi(v, B) = \begin{cases}
w_s \cdot \left(\frac{R_{\text{safe}} - d(v, B)}{d(v, B) + \epsilon}\right)^2 & \text{if } d(v, B) < R_{\text{safe}} \\
0 & \text{if } d(v, B) \ge R_{\text{safe}}
\end{cases}$$
where:
- $R_{\text{safe}}$: User-configured safety buffer radius.
- $\epsilon > 0$: Small barrier constant preventing numerical singularity.
- $w_s$: Safety penalty weight.

This creates a repulsive barrier around all hazardous states, mathematically steering the search toward paths that maintain safe clearance.

---

## 3. Algorithm Architecture: Lifelong Planning A* (LPA*)

We implement **Lifelong Planning A\* (LPA\*)**, an incremental heuristic search algorithm capable of rapidly adapting to dynamic changes (transition failures, goal migrations, obstacle updates, and new shortcuts) without recomputing paths from scratch.

### 3.1 Node Estimates
For each state $u \in S$, the planner maintains:
- $g(u)$: Cost of the shortest path from $s_I$ to $u$ discovered so far.
- $rhs(u)$: One-step lookahead estimate:
$$rhs(u) = \begin{cases}
0 & \text{if } u = s_I \\
\min_{v \in \text{Pred}(u)} \left(g(v) + c_{\text{eff}}(v, u)\right) & \text{if } u \neq s_I
\end{cases}$$

A vertex $u$ is:
- **Consistent** if $g(u) = rhs(u)$
- **Overconsistent** if $g(u) > rhs(u)$ (a better path to $u$ was discovered)
- **Underconsistent** if $g(u) < rhs(u)$ (a transition leading to $u$ increased in cost or became blocked)

### 3.2 Two-Level Lexicographical Priority Key
All inconsistent vertices ($g(u) \neq rhs(u)$) reside in the priority queue keyed by:
$$\mathbf{k}(u) = [k_1(u), k_2(u)] = [\min(g(u), rhs(u)) + h(u, s_G), \; \min(g(u), rhs(u))]$$

Keys are ordered lexicographically:
$$\mathbf{k}(u) < \mathbf{k}(v) \iff (k_1(u) < k_1(v)) \lor (k_1(u) = k_1(v) \land k_2(u) < k_2(v))$$

### 3.3 Admissible Heuristic Function
The heuristic $h(u, s_G)$ estimates the cost from $u$ to $s_G$ using the Cartesian Euclidean norm:
$$h(u, s_G) = \rho_{\min} \cdot \|u - s_G\|_2 = \rho_{\min} \cdot \sqrt{\sum_{i=1}^d (x_{u, i} - x_{G, i})^2}$$
Since straight-line Euclidean distance satisfies the triangle inequality and never overestimates actual edge traversal costs, $h$ is admissible and consistent.

---

## 4. Complexity Analysis

| Operation | Time Complexity | Space Complexity |
| :--- | :--- | :--- |
| Initial Full Plan | $\mathcal{O}((\|V\| + \|E\|) \log \|V\|)$ | $\mathcal{O}(\|V\| + \|E\|)$ |
| Incremental Replan (LPA\*) | $\mathcal{O}((\|\Delta V\| + \|\Delta E\|) \log \|\Delta V\|)$ | $\mathcal{O}(\|V\| + \|E\|)$ |
| Safety Field Distance Query | $\mathcal{O}(\|B\| \cdot d)$ | $\mathcal{O}(1)$ |
| Heuristic Evaluation | $\mathcal{O}(d)$ | $\mathcal{O}(1)$ |

Where:
- $\|V\| = |S|$ is total state count, $\|E\| = |T|$ is total transition count.
- $\|\Delta V\| \ll \|V\|$ is the subset of vertices affected by a localized dynamic update.
- $d$ is embedding dimensionality.
- $\|B\|$ is the number of bad states.

---

## 5. Experimental Evaluation & Test Case Results

All 6 required test scenarios from the specification and bonus benchmarks were evaluated.

### Summary Table of Results

| Test Scenario | Goal Status | Bad States Visited | Path Cost ($C$) | Min Clearance ($D$) | Explored States | Speedup vs Scratch |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **TC1: Basic Reachability** | **YES** | **0** | 15.00 | 1000.00 | 4 | Baseline |
| **TC2: Bad State Avoidance** | **YES** | **0** | 12.00 | 10.00 | 5 | Baseline |
| **TC3: Safety Margin Optimization** | **YES** | **0** | 10.00 | 15.07 | 4 | Baseline |
| **TC4: Dynamic Transition Failure** | **YES** | **0** | 8.00 | 1000.00 | **3 (inc)** | **2.0x** |
| **TC5: Dynamic Goal Update** | **YES** | **0** | 6.00 | 1000.00 | **2 (inc)** | **2.5x** |
| **TC6: Dynamic Shortcut Discovery** | **YES** | **0** | 4.00 | 1000.00 | **1 (inc)** | **4.0x** |
| **Bonus: 2D Grid Maze (8x8)** | **YES** | **0** | 140.00 | 10.00 | 60 | Baseline |
| **Bonus: 8D Semantic KG** | **YES** | **0** | 8.00 | 1.23 | 5 | Baseline |

### Key Observations:
1. **Zero Safety Violations**: In 100% of test runs, bad states were strictly avoided ($s \notin B$).
2. **Safety vs Cost Trade-off (TC3)**: When repulsive safety weighting $w_s$ is active, the planner seamlessly prefers the safer detour ($0 \to 3 \to 4 \to 5$, $D=15.07$) over the risky short path ($0 \to 1 \to 2 \to 5$, $D=5.02$).
3. **Incremental Re-planning Efficiency (TC4, TC5, TC6)**: When edges or goals change, LPA\* updates only inconsistent subtrees, exploring as few as 1 to 3 states instead of rebuilding the full graph.

---

## 6. Software Architecture

```
include/
  ├── Types.hpp               # State, Transition, PlanningProblem, PlanningResult interfaces
  ├── SafetyField.hpp         # Euclidean distance & repulsive barrier potential fields
  ├── SafeSemanticPlanner.hpp # LPA* incremental engine and priority queue management
  └── TestCases.hpp           # Test cases 1-6, benchmarks, and JSON serialization
src/
  └── main.cpp                # CLI benchmark runner and JSON exporter
visualizer/
  ├── index.html              # Standalone interactive HTML5/Canvas visualizer
  └── app.py                  # Python launcher server
```
