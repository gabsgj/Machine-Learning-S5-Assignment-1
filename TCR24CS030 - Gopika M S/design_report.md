PCCST503 – Machine Learning

Assignment 1

Design of a Safe Semantic Planner in a Finite Cartesian State Space

Department: Computer Science and Engineering
Implementation Language: Python 3
Algorithm: D* Lite

Student Name: GOPIKA M S
Roll Number: 30

---

1. Introduction

The objective of this assignment is to design and implement a safe semantic planner that computes a valid path through a finite Cartesian state space.

The planner receives an initial state, a goal state, a set of bad states and a collection of directed transitions.

Each transition contains:

- Transition ID
- Source state
- Destination state
- Cost
- Safety score
- Reliability
- Availability

The planner must find a path that reaches the goal while avoiding bad states.

The implementation uses the D* Lite algorithm because it is suitable for dynamic environments where transition availability, transition costs and goal states may change during execution.

---

2. Problem Definition

Let the finite state space be:

[
S = {s_1,s_2,\ldots,s_n}
]

Each state is represented by a point in a Cartesian space:

[
s_i=(x_1,x_2,\ldots,x_d)
]

The planning problem contains:

- Initial state s_I
- Goal state s_G
- Bad states B
- Directed transitions T

A transition contains:

[
t=(from,to,cost,safety,reliability,availability)
]

The planner must produce a path:

[
P=(s_I,s_1,s_2,\ldots,s_G)
]

such that no state in the path belongs to the set of bad states.

---

3. State Representation

Each state is represented using a Python dataclass:

@dataclass(frozen=True)
class State:
    id: int
    embedding: Tuple[float, ...]

The "id" uniquely identifies the state.

The "embedding" stores the Cartesian coordinates.

For example:

State(
    id=1,
    embedding=(1.0, 0.0)
)

The implementation uses two-dimensional Cartesian coordinates for the experimental test cases.

---

4. Transition Representation

Each directed transition is represented as:

@dataclass
class Transition:
    id: int
    from_state: int
    to_state: int
    cost: float
    safety: float
    reliability: float
    available: bool = True

The transition ID uniquely identifies the transition.

"from_state" and "to_state" define the direction.

"cost" represents the basic transition cost.

"safety" represents the supplied safety score.

"reliability" represents the reliability of the transition.

"available" determines whether the transition can currently be used.

Unavailable transitions are ignored by the planner.

---

5. Planning Problem

The complete problem is represented using:

@dataclass
class PlanningProblem:
    initial_state: int
    goal_state: int
    bad_states: Set[int]
    states: List[State]
    transitions: List[Transition]

This provides all information required by the planner.

---

6. Planning Result

The planner returns a "PlanningResult" containing:

@dataclass
class PlanningResult:
    success: bool
    state_path: List[int]
    transition_path: List[int]
    total_cost: float
    minimum_safety_distance: float
    cumulative_reliability: float
    explored_states: int
    planning_time_ms: float

The result records both the solution path and experimental measurements.

---

7. Data Structures

The implementation uses the following major data structures.

7.1 State Dictionary

States are stored in a dictionary:

self.states = {
    state.id: state
}

This allows efficient access to state coordinates.

7.2 Adjacency List

Outgoing transitions are stored in:

self.adj

This represents the directed graph.

7.3 Reverse Adjacency List

Incoming transitions are stored in:

self.rev

D* Lite uses reverse graph information to update predecessor states efficiently.

7.4 g and rhs Values

D* Lite maintains:

- "g(s)" – current cost estimate
- "rhs(s)" – one-step lookahead value

These values are stored in dictionaries.

7.5 Priority Queue

An implementation of a priority queue is maintained using Python's "heapq" module.

The queue stores states according to their D* Lite priority keys.

7.6 Distance Cache

Distances to the nearest bad state are cached in:

self.distance_cache

This avoids repeatedly calculating the same Euclidean distances.

---

8. D* Lite Algorithm

D* Lite is an incremental graph-search algorithm designed for path planning in changing environments.

Instead of completely discarding previous search information after every environmental change, D* Lite updates affected states.

The implementation maintains:

[
g(s)
]

and:

[
rhs(s)
]

for each state.

The goal state is initialized with:

[
rhs(goal)=0
]

and inserted into the priority queue.

The algorithm repeatedly processes states until the start state becomes locally consistent.

---

9. Priority Key

The D* Lite priority key is implemented as:

def key(self, state_id):
    minimum = min(
        self.g[state_id],
        self.rhs[state_id]
    )

    return (
        minimum
        + self.distance(self.start, state_id)
        + self.km,
        minimum
    )

The first component combines the current best estimate with the heuristic distance from the start state.

The second component is the minimum of "g" and "rhs".

---

10. Heuristic Function

The planner uses Euclidean distance as the heuristic.

For two states a and b:

[
h(a,b)=
\sqrt{
\sum_{i=1}^{d}(a_i-b_i)^2
}
]

The implementation is:

def distance(self, a, b):

    x = self.states[a].embedding
    y = self.states[b].embedding

    return math.sqrt(
        sum(
            (u-v)**2
            for u,v in zip(x,y)
        )
    )

Euclidean distance is appropriate because the states are embedded in Cartesian space.

---

11. Bad-State Avoidance

Bad states are treated as forbidden states.

A transition is assigned infinite effective cost when:

- The transition is unavailable.
- Its source state is bad.
- Its destination state is bad.

This guarantees that the planner does not intentionally select a path through a bad state.

The important condition is:

if (
    not transition.available
    or transition.from_state in self.bad
    or transition.to_state in self.bad
):
    return INF

Therefore, bad states are excluded from feasible paths.

---

12. Safety Computation

For every visited state, the planner calculates its Euclidean distance from the nearest bad state.

For a state s:

[
D(s)=\min_{b\in B}d(s,b)
]

For the complete path, the minimum safety distance is:

[
D_{min}=\min_{s\in P}D(s)
]

A larger value indicates that the path stays farther away from bad states.

If no bad states exist, the safety distance is considered infinite.

---

13. Safety-Aware Cost

The planner can include a safety penalty in the effective transition cost.

The implemented cost is:

[
C_{effective}

C
+
\frac{w_s}{D+\epsilon}
+
0.1(1-R)
]

where:

- C = transition cost
- w_s = safety weight
- D = distance to the nearest bad state
- R = transition reliability
- \epsilon = small value preventing division by zero

The safety weight can be changed when creating the planner:

DStarLitePlanner(
    safety_weight=2.0
)

A safety weight of zero gives cost-focused planning.

A larger safety weight increases the importance of distance from bad states.

---

14. Reliability

Transition reliability is incorporated through a small reliability penalty.

For a transition with reliability R:

[
Penalty=0.1(1-R)
]

The cumulative reliability of the final path is calculated as:

[
R_{path}

\prod_{i=1}^{n}R_i
]

This provides an additional measure of path quality.

---

15. Path Selection

After D* Lite computes the shortest estimated path, the planner reconstructs the path from the initial state to the goal.

At every state, available transitions are examined.

A transition is considered only if:

- It is available.
- It does not enter a bad state.
- Its destination has a finite "g" value.
- It does not create a cycle.

The transition with the lowest effective cost plus remaining estimated cost is selected.

---

16. Dynamic Environment

The assignment requires the planner to handle changes such as:

- Transition failure
- Transition cost changes
- New transitions
- Goal changes

The implementation provides methods for these operations.

---

17. Dynamic Transition Update

The following method updates transition availability or cost:

planner.update_transition(
    transition_id=1,
    available=False
)

The affected source vertex is then updated.

This allows the planner to respond to a transition becoming unavailable.

---

18. Adding a New Transition

A new transition can be added using:

planner.add_transition(
    new_transition
)

The new transition is inserted into the adjacency and reverse adjacency structures.

The affected vertex is updated so that the planner can consider the new route.

This is demonstrated in Test Case 6.

---

19. Dynamic Goal Update

The goal can be changed using:

planner.update_goal(new_goal)

The planner then prepares the search structures for the new goal.

This demonstrates the ability to adapt the planning problem when the target changes.

---

20. Test Cases

Six assignment scenarios are evaluated.

Test Case 1 – Basic Reachability

Graph:

0 -> 1 -> 2 -> 3

Expected result:

The planner should find the path from state 0 to state 3.

---

Test Case 2 – Bad State Avoidance

Two paths are provided.

The first contains bad state 5.

The second avoids the bad state.

Expected result:

The planner selects the safe path.

---

Test Case 3 – Safety Margin

Two valid routes are provided with different costs and distances from a bad state.

Two configurations are tested:

- Cost-focused planning.
- Safety-weighted planning.

This demonstrates the effect of the safety weight on route selection.

---

Test Case 4 – Dynamic Transition Failure

Initially:

0 -> 1 -> 3

is the preferred route.

The transition from state 1 to state 3 is then made unavailable.

Expected result:

The planner finds an alternative route.

---

Test Case 5 – Goal Update

The initial goal is state 3.

The goal is then changed to state 4.

Expected result:

The planner produces a revised route to the new goal.

---

Test Case 6 – Transition Addition

A new shortcut transition from state 0 directly to state 3 is inserted.

Expected result:

The planner discovers the improved shortcut route.

---

21. Experimental Evaluation

The following metrics are collected:

Metric| Description
Goal success| Whether the goal was reached
Bad states visited| Number of bad states in the path
Total cost| Sum of original transition costs
Minimum safety distance| Minimum distance to a bad state
Cumulative reliability| Product of transition reliabilities
Explored states| Number of processed states
Planning time| Time required to compute the path

The results are stored in:

experimental_results.csv

---

22. Time Complexity

Let:

- V = number of states
- E = number of transitions

D* Lite uses priority-queue based graph processing.

A typical complexity bound is approximately:

[
O(E\log V)
]

for a search/update operation, depending on the number of affected states.

The Euclidean distance calculation for a state in d-dimensional space takes:

[
O(d)
]

and nearest-bad-state calculation can take:

[
O(|B|d)
]

when the value is not already cached.

The distance cache reduces repeated calculations.

---

23. Space Complexity

The planner stores:

- States
- Transitions
- Adjacency lists
- Reverse adjacency lists
- "g" values
- "rhs" values
- Priority queue
- Distance cache

Therefore, the main graph storage requires approximately:

[
O(V+E)
]

space, with additional storage for the priority queue and cached distance values.

---

24. Advantages of the Implementation

The implementation provides several advantages:

1. Uses a standard incremental planning algorithm.
2. Avoids bad states.
3. Uses Cartesian Euclidean distance.
4. Considers transition cost and reliability.
5. Supports safety weighting.
6. Supports dynamic transition updates.
7. Supports new transition insertion.
8. Supports dynamic goal changes.
9. Measures planning performance.

---

25. Limitations

The current implementation uses a relatively small finite test graph.

The safety score supplied with transitions is stored but the main safety calculation is based on Euclidean distance to bad states.

The current experiments also use a simple safety penalty rather than a fully learned or probabilistic safety model.

For large-scale problems, additional optimizations could be required.

---

26. Future Improvements

Possible improvements include:

- Larger state spaces.
- Multi-goal planning.
- Time-dependent transition availability.
- More sophisticated safety models.
- Learning-based heuristics.
- Parallel search.
- Visualization of the state graph.
- More extensive dynamic replanning experiments.

---

27. Conclusion

A Python implementation of a D* Lite based Safe Semantic Planner was developed for a finite Cartesian state space.

The planner represents states using Cartesian embeddings and represents transitions using cost, safety, reliability and availability information.

Bad states are treated as forbidden states, while Euclidean distance is used to measure safety margins.

The implementation also supports dynamic transition changes, transition insertion and goal updates.

The six test scenarios demonstrate basic reachability, bad-state avoidance, safety-aware planning and adaptation to dynamic changes.

Therefore, the implementation satisfies the major functional requirements of the assignment and provides a foundation for further improvements in dynamic and safety-aware planning.
