# Safe Semantic Planner in a Finite Cartesian State Space (LPA*)

An incremental graph search planner implemented in Python for navigating finite Cartesian state spaces $\mathbb{R}^d$. Using **Lifelong Planning A* (LPA*)**, the planner computes cost-optimal and obstacle-safe paths while dynamically adapting to changes in graph topology, transition availability, and goal locations without re-running searches from scratch.

---

## 📌 Features

* **Incremental Search (LPA*):** Reuses prior search trees and recalculates distances only for locally inconsistent states when transitions or costs change.
* **Safety Margin Maximization:** Implements an inverse Euclidean distance penalty from bad states ($B$) to keep planned trajectories inside safe corridors.
* **Dynamic Environment Support:** Real-time API for toggling transition availability, modifying traversal costs, and re-assigning target goal states on the fly.
* **Built-in Verification Suite:** Includes test scenarios covering reachability, bad-state avoidance, safety buffers, dynamic edge disabling, and goal updates.

---

## 🧮 Mathematical Formulation

### 1. State & Transition Representation
* **States ($S$):** Finite set $S = \{s_1, s_2, \dots, s_n\}$ embedded in $\mathbb{R}^d$ with coordinate vector representations $s_i = (x_1, x_2, \dots, x_d)$.
* **Bad States ($B$):** Subset of hazardous states $B \subset S$ that must never be visited.
* **Transitions ($T$):** Directed transitions $(s_i, s_j)$ with specified base cost, safety, reliability, and availability status[cite: 1].

### 2. Composite Edge Cost Function
To balance total path cost minimization with safety margin maximization[cite: 1], edge costs incorporate a proximity penalty derived from the Euclidean distance to the nearest bad state[cite: 1]:

$$c(u, v) = \begin{cases} \infty & \text{if } u \in B \text{ or } v \in B \text{ or } \text{available}(u, v) = \text{False} \\ \text{cost}(u, v) + \frac{w_{\text{safety}}}{\min_{b \in B} \Vert{}v - b\Vert{}_2} & \text{otherwise} \end{cases}$$

### 3. LPA* Distance Estimates & Inconsistency
Every state $u$ tracks two distance variables:

* **$g(u)$**: The cached shortest distance estimate from $s_{\text{start}}$ to $u$.
* **$rhs(u)$**: The one-step lookahead distance estimate derived from predecessor nodes:

$$
rhs(u) = \begin{cases} 0 & \text{if } u = s_{\text{start}} \\ \min_{p \in \text{Pred}(u)} (g(p) + c(p, u)) & \text{otherwise} \end{cases}
$$

Nodes with $g(u) \neq rhs(u)$ are inserted into a priority queue keyed by $k(u) = [k_1(u), k_2(u)]$:
* $k_1(u) = \min(g(u), rhs(u)) + \Vert{}u - s_{\text{goal}}\Vert{}_2$
* $k_2(u) = \min(g(u), rhs(u))$

---

## 🛠️ Software Interfaces

The project implements the data interfaces specified for the assignment[cite: 1]:

| Class | Description |
|---|---|
| `State` | Stores state ID and embedding vector in $\mathbb{R}^d$[cite: 1]. |
| `Transition` | Represents directed edges with IDs, cost, safety score, reliability, and availability status[cite: 1]. |
| `PlanningProblem` | Input container holding initial state, goal state, bad state IDs, and graph elements[cite: 1]. |
| `PlanningResult` | Output container storing success status, state path, transition path, cost, and safety score[cite: 1]. |
| `LPAPlanner` | Core planner providing `plan()`, `update_edge()`, and `update_goal()`[cite: 1]. |

---

## 🚀 Getting Started

### Prerequisites
* **Python 3.8+** (Uses standard library packages: `math`, `heapq`, `dataclasses`, `typing`).

### Execution

Clone the repository and run the planner directly:

```bash
git clone [https://github.com/Devan444/ML_project.git]
python ml_pro.py
