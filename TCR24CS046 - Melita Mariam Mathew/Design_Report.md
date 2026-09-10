# PCCST503 – Machine Learning | Assignment 1
# Formal Design Report: Design of a Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**  
**Course Code:** PCCST503  
**Subject:** Machine Learning & AI Planning  
**Deliverable:** 2 — Academic & Technical Design Report  

---

## Executive Summary

This report presents the theoretical formulation, architectural design, data structures, heuristic engineering, and algorithmic proofs for a **Safe Semantic Planner** operating within a finite Cartesian state space $S \subset \mathbb{R}^d$. The system integrates incremental graph search algorithms (**Lifelong Planning A\*** and **D\* Lite**) with multi-objective semantic optimization to compute safe, cost-minimal, and highly reliable trajectories between initial and goal states while strictly avoiding hazardous ("bad") states and maximizing clearance margins.

---

## 1. Problem Formulation & Mathematical Model

### 1.1 State Space and Vector Representation
Let the environment state space be defined as a finite set of $n$ discrete semantic states:
$$S = \{s_1, s_2, \dots, s_n\}$$
Each state $s_i \in S$ is embedded in a $d$-dimensional continuous Cartesian space $\mathbb{R}^d$:
$$\mathbf{x}(s_i) = (x_{i,1}, x_{i,2}, \dots, x_{i,d})^\top \in \mathbb{R}^d$$
This Cartesian embedding captures geometric locations, semantic feature coordinates, or latent conceptual representations.

### 1.2 Directed Transitions & Attribute Tensor
The connectivity of the environment is represented as a directed graph $G = (S, T)$, where $T \subseteq S \times S$ is the set of directed transitions $e = (s_i, s_j)$. Each transition $e \in T$ is augmented with an attribute tuple:
$$\tau(e) = \langle c(e), \sigma(e), \rho(e), a(e) \rangle$$
where:
- $c(e) \in \mathbb{R}^+$: Direct transition cost (e.g., Euclidean travel length, energy expenditure, or latency).
- $\sigma(e) \in [0, 1]$: Inherent safety rating of the transition channel.
- $\rho(e) \in [0, 1]$: Transition execution reliability (probability of successful traversal without failure).
- $a(e) \in \{0, 1\}$: Dynamic availability flag ($1 = \text{available}$, $0 = \text{blocked/disabled}$).

### 1.3 Hazardous / Bad States & Invariant Constraint
A subset of states $B \subset S$ is designated as the **Hazardous (Bad) State Set**:
$$B = \{b_1, b_2, \dots, b_k\} \subset S$$
**Fundamental Safety Invariant:** Under no circumstance shall any valid trajectory $P = (s_{0}, s_{1}, \dots, s_{m})$ visit a state belonging to $B$:
$$\forall s \in P, \quad s \notin B$$

---

## 2. Multi-Objective Optimization Formulation

A valid planning solution is a sequence of transitions $P = (e_1, e_2, \dots, e_m)$ connecting the initial state $s_I$ to the goal state $s_G$. The planner solves a constrained multi-objective optimization problem modeled by the composite objective score:

$$\text{Maximize } \text{Score}(P) = \alpha G(P) - \beta C(P) + \gamma D(P) + \delta R(P)$$

Subject to:
1. **Reachability:** $s_0 = s_I$ and $s_m = s_G$.
2. **Transition Continuity:** $\text{to}(e_i) = \text{from}(e_{i+1}) \quad \forall i \in \{1, \dots, m-1\}$.
3. **Availability:** $a(e_i) = 1 \quad \forall e_i \in P$.
4. **Strict Safety Invariant:** $\{s_0, s_1, \dots, s_m\} \cap B = \emptyset$.

### Component Definitions:
- **Goal Completion Indicator ($G$):**
  $$G(P) = \begin{cases} 1 & \text{if } s_m = s_G \\ 0 & \text{otherwise} \end{cases}$$
- **Cumulative Transition Cost ($C$):**
  $$C(P) = \sum_{e \in P} c(e)$$
- **Minimum Safety Distance ($D$):**
  The minimum Cartesian Euclidean clearance between all trajectory states and the nearest bad state:
  $$D(P) = \min_{s \in P} \min_{b \in B} \|\mathbf{x}(s) - \mathbf{x}(b)\|_2$$
- **Cumulative Path Reliability ($R$):**
  The joint probability of successful traversal across independent transitions:
  $$R(P) = \prod_{e \in P} \rho(e)$$

---

## 3. Heuristic Function Design & Admissibility

### 3.1 Cartesian Euclidean Distance Heuristic
In a Cartesian space $\mathbb{R}^d$, the shortest possible physical or semantic distance between state $u$ and goal $s_G$ is given by the $L_2$ Euclidean norm:
$$h(u, s_G) = \|\mathbf{x}(u) - \mathbf{x}(s_G)\|_2 = \sqrt{\sum_{k=1}^d (x_{u,k} - x_{s_G,k})^2}$$

### 3.2 Proof of Admissibility and Consistency
**Theorem 1 (Admissibility):** *The Euclidean heuristic $h(u, s_G)$ never overestimates the true optimal transition cost $c^*(u, s_G)$, provided edge costs are lower-bounded by Euclidean step lengths.*

*Proof:* For any direct transition $e = (u, v)$, the triangle inequality in Euclidean metric space dictates:
$$\|\mathbf{x}(u) - \mathbf{x}(s_G)\|_2 \le \|\mathbf{x}(u) - \mathbf{x}(v)\|_2 + \|\mathbf{x}(v) - \mathbf{x}(s_G)\|_2$$
Since $c(u, v) \ge \|\mathbf{x}(u) - \mathbf{x}(v)\|_2$, we have:
$$h(u, s_G) \le c(u, v) + h(v, s_G)$$
This satisfies the definition of **monotonicity (consistency)**:
$$h(u) \le c(u, v) + h(v)$$
By induction over path steps, $h(u) \le c^*(u, s_G)$, proving admissibility. $\blacksquare$

---

## 4. Safety Clearance & Barrier Potential Field

To ensure both strict safety ($s \notin B$) and conservative clearance margins ($D(P) \ge D_{\text{safe}}$), the planner incorporates an exact spatial repulsive barrier into the effective transition cost $c_{\text{eff}}(u, v)$:

$$c_{\text{eff}}(u, v) = \begin{cases} 
\infty & \text{if } v \in B \text{ or } u \in B \text{ or } a(u, v) = 0 \\
\beta \cdot c(u, v) + \delta (1 - \rho(u, v)) + \Phi(v) & \text{otherwise}
\end{cases}$$

Where $\Phi(v)$ is the **Safety Repulsive Potential Field**:
$$\Phi(v) = \begin{cases} 
\gamma \cdot \left[ (D_{\text{safe}} - D(v, B))^2 + \frac{1}{D(v, B) + \epsilon} \right] & \text{if } D(v, B) < D_{\text{safe}} \\
0 & \text{if } D(v, B) \ge D_{\text{safe}}
\end{cases}$$

Here, $D(v, B) = \min_{b \in B} \|\mathbf{x}(v) - \mathbf{x}(b)\|_2$, $D_{\text{safe}}$ is the safety buffer threshold, and $\epsilon > 0$ prevents numerical overflow.

---

## 5. Algorithmic Architecture & Incremental Search

The core reasoning engine implements **Lifelong Planning A\* (LPA\*)** and **D\* Lite**.

```
                           +-------------------------------------+
                           |        Planning Problem Setup       |
                           |  (States, Bad States, Transitions)  |
                           +-------------------------------------+
                                              |
                                              v
                           +-------------------------------------+
                           |    Exact Spatial Safety Engine      |
                           |   Compute Euclidean Distance to B   |
                           +-------------------------------------+
                                              |
                                              v
                           +-------------------------------------+
                           |       LPA* / D* Lite Priority       |
                           |      Queue Initialization (k1, k2)  |
                           +-------------------------------------+
                                              |
                     +------------------------+------------------------+
                     |                                                 |
                     v                                                 v
      +-----------------------------+                   +-----------------------------+
      |      Initial Search         |                   |     Dynamic Environment     |
      |   g(s) / rhs(s) Convergence |                   |  Edge Failure / Goal Update |
      +-----------------------------+                   +-----------------------------+
                     |                                                 |
                     v                                                 v
      +-----------------------------+                   +-----------------------------+
      |  Optimal Safe Path Output   |                   |   Local Key Recalculation   |
      |   (Cost, Clearance, Rel)    |<------------------|  Incremental Replanning     |
      +-----------------------------+                   +-----------------------------+
```

### 5.1 Lifelong Planning A* (LPA*) Mechanics
LPA\* maintains two distance estimates for each vertex $u \in S$:
1. $g(u)$: The current estimated shortest path cost from $s_I$ to $u$.
2. $rhs(u)$: A one-step lookahead estimate based on the $g$-values of predecessors:
   $$rhs(u) = \begin{cases} 0 & \text{if } u = s_I \\ \min_{v \in \text{pred}(u)} (g(v) + c_{\text{eff}}(v, u)) & \text{otherwise} \end{cases}$$

A vertex $u$ is termed:
- **Consistent:** if $g(u) = rhs(u)$.
- **Overconsistent:** if $g(u) > rhs(u)$ (cost decreased or newly discovered shorter path).
- **Underconsistent:** if $g(u) < rhs(u)$ (edge cost increased or edge deleted).

#### 2-Tuple Priority Key:
Vertices with $g(u) \neq rhs(u)$ are stored in a min-priority queue with key $\mathbf{k}(u) = [k_1(u), k_2(u)]^\top$:
$$k_1(u) = \min(g(u), rhs(u)) + h(u, s_G)$$
$$k_2(u) = \min(g(u), rhs(u))$$

Lexicographical ordering: $\mathbf{k}(u) < \mathbf{k}(v) \iff (k_1(u) < k_1(v)) \lor (k_1(u) = k_1(v) \land k_2(u) < k_2(v))$.

### 5.2 LPA* Core Algorithm Pseudocode

```text
Procedure CalculateKey(s):
    return [ min(g(s), rhs(s)) + h(s, s_G), min(g(s), rhs(s)) ]

Procedure UpdateVertex(u):
    if u != s_I:
        rhs(u) = min_{v in Pred(u)} (g(v) + c_eff(v, u))
    if u in Queue:
        Queue.Remove(u)
    if g(u) != rhs(u):
        Queue.Insert(u, CalculateKey(u))

Procedure ComputeShortestPath():
    while Queue.TopKey() < CalculateKey(s_G) or rhs(s_G) != g(s_G):
        u = Queue.Pop()
        if g(u) > rhs(u):
            g(u) = rhs(u)
            for v in Succ(u):
                UpdateVertex(v)
        else:
            g(u) = infinity
            UpdateVertex(u)
            for v in Succ(u):
                UpdateVertex(v)

Procedure Main():
    for all s in S:
        g(s) = rhs(s) = infinity
    rhs(s_I) = 0
    Queue.Insert(s_I, CalculateKey(s_I))
    ComputeShortestPath()
    while true:
        Wait for environment change (edge update, goal shift)
        for all changed edges (u, v):
            Update edge cost c_eff(u, v)
            UpdateVertex(v)
        ComputeShortestPath()
```

---

## 6. Dynamic Replanning Mechanisms

When an environment change occurs (e.g., road blockage, obstacle detection, goal shift, or new edge insertion):
1. **Transition Availability / Cost Change:** Only the destination node $v$ of the modified edge $(u, v)$ is updated via `UpdateVertex(v)`. Inconsistencies propagate exclusively to affected downstream nodes without touching unaffected subtrees.
2. **Transition Addition:** A newly added shortcut $(u, v)$ triggers `UpdateVertex(v)`, which lowers $rhs(v)$ if $g(u) + c_{\text{eff}}(u, v) < rhs(v)$, instantly propagating the improvement.
3. **Goal State Relocation:** When $s_G$ shifts to $s_G'$, heuristic values $h(u, s_G')$ change. LPA\* updates the priority keys in the active queue without re-evaluating unchanged $g$ and $rhs$ values. In D\* Lite, backward search natively retains the cost-to-goal tree.

---

## 7. Complexity Analysis

### 7.1 Time Complexity
- **Initial Planning:** In the worst case where all nodes are expanded, LPA\* behaves identically to A\*. With a binary heap priority queue, each state insertion/key update takes $O(\log |V|)$. Thus, the initial search time is bounded by:
  $$T_{\text{initial}} = O((|V| + |E|) \log |V|)$$
- **Incremental Replanning:** When $k$ edges change dynamically, only the subset of vertices $V_{\text{affected}} \subseteq V$ whose shortest paths depend on the modified edges become inconsistent. The replanning time complexity is:
  $$T_{\text{replan}} = O((|V_{\text{affected}}| + |E_{\text{affected}}|) \log |V_{\text{affected}}|)$$
  Where $|V_{\text{affected}}| \ll |V|$ in localized changes, resulting in empirical speedup factors of $5\times \text{ to } 60\times$.

### 7.2 Space Complexity
The planner stores:
- State embeddings: $O(|V| \cdot d)$ where $d$ is embedding dimension.
- Graph adjacency lists: $O(|V| + |E|)$.
- Lookup hash maps ($g, rhs$, keys): $O(|V|)$.
- Priority Queue: $O(|V|)$.

Total Space Complexity:
$$S_{\text{total}} = O(|V| \cdot d + |E|)$$
This is strictly linear with respect to the state graph scale and Cartesian dimensionality.

---

## 8. Conclusion

The Safe Semantic Planner integrates rigorous Euclidean geometry with state-of-the-art incremental search theory. By leveraging continuous Cartesian embeddings for admissible heuristics, enforcing hard barrier invariants for zero-hazard guarantees, and applying dynamic key updates for instant replanning, the architecture achieves optimal, safe, and computationally scalable trajectory generation.
