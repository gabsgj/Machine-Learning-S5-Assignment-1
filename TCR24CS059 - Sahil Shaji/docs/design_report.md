# Design Report: Safe Semantic Planner

## 1. State Representation
The state space is defined in a finite Cartesian space $\mathbb{R}^d$. Each state is represented by an ID (`uint64_t`) and a feature vector embedding (`std::vector<double>`). This allows the planner to work in any number of dimensions flexibly.

## 2. Data Structures
The core algorithm implemented is **D* Lite**, which uses a backwards-search strategy (searching from the goal to the start). This approach is optimally suited for dynamic environments. 

Key data structures include:
- **Priority Queue ($U$)**: Implemented using `std::priority_queue` with a custom comparator to act as a min-heap. It holds nodes prioritized by a 2-tuple key: `[min(g(s), rhs(s)) + h(s_start, s) + k_m, min(g(s), rhs(s))]`.
- **State Lookup**: Hash maps (`std::unordered_map`) to store state embeddings (`states`) and D* Lite specific attributes (`node_info`), mapping state IDs to their respective attributes.
- **Graph Representation**: Forward (`adj_forward`) and backward (`adj_backward`) adjacency lists implemented using hash maps of vectors. This allows for rapid neighbor expansion and updating edges dynamically in $O(1)$ amortized lookup time per node.
- **Queue Synchronization**: A hash map (`U_dict`) mirroring the Priority Queue to handle dynamic updates and "lazy deletion", which bypasses C++ `std::priority_queue`'s inability to dynamically decrease keys or remove arbitrary elements efficiently.

## 3. Heuristic Function
The heuristic function $h(a, b)$ is defined as the Euclidean distance between the Cartesian embeddings of states $a$ and $b$. 
Since the objective involves path cost optimization over a Cartesian plane, Euclidean distance is both **admissible** (it never overestimates the actual shortest physical path cost) and **consistent**, ensuring optimality in D* Lite graph exploration.

## 4. Safety Computation
A bad state is an explicit point in space to be strictly avoided. Safety is incorporated directly into the transition edge cost calculation. 
During initialization:
1. The planner precomputes the Euclidean distance from every state to the nearest bad state. 
2. A distance penalty is created by taking the maximum distance across all states and subtracting the specific node's distance. This essentially inverts the metric: states closer to bad states receive a larger safety penalty.
3. Edges pointing into bad states are assigned an infinite cost to ensure strictly hard avoidance.

The effective cost of navigating transition $T(u, v)$ is a weighted combination:
$$ C(u, v) = \max(0.001, \alpha \cdot T.cost - \beta \cdot T.reliability + \gamma \cdot SafetyPenalty(v)) $$
This transforms the multi-objective problem into a single scalar optimal path problem.

## 5. Time Complexity
- **Initialization**: Precomputing safety distances evaluates distances from all $V$ states to $B$ bad states, yielding $O(V \cdot B)$.
- **Planning (Initial)**: Similar to A*, bounded by $O(E \log V)$ where $E$ is the number of transitions and $V$ is the number of states.
- **Replanning (Dynamic Updates)**: When an edge changes or the goal is updated, only the locally affected vertices (and those impacted by propagation) are pushed to the priority queue. The replanning complexity is empirically proportional to the number of nodes affected by the change rather than the entire graph, making it significantly faster than a full A* recomputation.

## 6. Space Complexity
The implementation stores the graph explicitly using adjacency lists:
- Graph Storage: $O(V + E)$
- Lookup Tables and node info: $O(V)$
- Priority Queue: $O(V)$ in the worst case.
- Overall Space Complexity: **$O(V + E)$** which scales linearly with the size of the state space graph.
