# SafePath: Design of a Safe Semantic Planner in a Finite Cartesian State Space
### PCCST503 — Machine Learning, Assignment 1 — Design Report

---

## 1. Introduction

This report describes the design and implementation of a planner that computes
a **safe** path between an initial state and a goal state in a finite,
directed, Cartesian-embedded state graph, while avoiding a set of "bad"
states and adapting efficiently when the environment changes. The core
algorithm is **D\* Lite** (Koenig & Likhachev, 2002), implemented from first
principles in C++17. Python (Matplotlib/NetworkX) is used strictly as a
downstream visualization layer over data the C++ program exports; no planning
logic exists in Python.

## 2. Problem Definition

Given a finite set of states `S = {s1, ..., sn}` embedded in `R^d`, an initial
state `sI`, a goal state `sG`, a set of bad states `B`, and a set of directed
transitions `T` (each carrying a cost, a safety score, a reliability, and an
availability flag), the planner must return a sequence of transitions from
`sI` to `sG` that never touches a state in `B`.

## 3. Assignment Objectives

1. Reach the goal.
2. Never visit a bad state (hard constraint, not a soft penalty — see §10).
3. Minimize total transition cost.
4. Maximize the minimum Euclidean distance from every visited state to the
   nearest bad state.
5. Plan within reasonable time, and **replan incrementally** rather than from
   scratch when the goal, bad states, or transitions change.

## 4. State Representation

```cpp
class State {
public:
    uint64_t id;
    std::vector<double> embedding;   // Cartesian coordinates in R^d
};
```
Matches the assignment's suggested interface exactly (`include/State.h`).

## 5. Transition Representation

```cpp
class Transition {
public:
    uint64_t id, from, to;
    double cost, safety, reliability;
    bool available;
};
```
`safety` is a *per-edge* input score (assignment-provided), distinct from the
*geometric* minimum-safety-distance metric `D(P)`, which is computed purely
from Cartesian embeddings against the bad-state set (see §11 for how both are
used). `include/Transition.h`.

## 6. Graph Representation

The graph is stored as two adjacency maps built once per `plan()` call and
incrementally maintained afterward:

```cpp
std::unordered_map<uint64_t, std::vector<uint64_t>> outTransitionIds_; // from -> [transitionId]
std::unordered_map<uint64_t, std::vector<uint64_t>> inTransitionIds_;  // to   -> [transitionId]
```

Both directions are maintained explicitly and are never conflated — D\* Lite's
`ComputeShortestPath` needs **predecessors** for backward propagation and
**successors** for computing `rhs`. Using only one adjacency map (as an
undirected-graph implementation would) is the single most common bug in
student D\* Lite implementations and is deliberately avoided (`rebuildAdjacency()`
in `DStarLite.cpp`).

## 7. Algorithm Selection

**D\* Lite** was selected over LPA\*.

- LPA\* replans efficiently when *edge costs* change but assumes the start and
  goal are fixed. This assignment explicitly requires **goal changes**
  (Test Case 5) in addition to edge changes.
- D\* Lite is LPA\*'s natural extension: it runs the *same* incremental
  machinery but searches from the goal toward the (potentially moving) start,
  using an `h(s, start)` heuristic and a `km` key-modifier term that lets the
  start move without invalidating the whole priority queue.
- Because a goal change in this problem is structurally similar to a "start
  move" in the robot-navigation formulation D\* Lite was designed for
  (both invalidate the anchor the heuristic is relative to), D\* Lite is the
  better structural fit even though our "robot" (the planning start) does not
  literally move during the six required test cases.

## 8. D\* Lite Algorithm

**Search direction.** The search grows *backward* from the goal. `g(s)` and
`rhs(s)` approximate the cost of the shortest path **from `s` to the goal**,
not from the start to `s`.

- `rhs(sgoal) = 0`; for every other state, `rhs(u) = min over successors v of ( edgeCost(u,v) + g(v) )`.
- A state is **locally consistent** when `g(u) == rhs(u)`. The open queue only
  ever holds locally *inconsistent* states.
- **Key**: `CalculateKey(s) = [ min(g(s),rhs(s)) + h(s, start) + km ; min(g(s),rhs(s)) ]`,
  compared lexicographically.
- **km** accumulates `h(lastStart, newStart)` whenever the start moves
  (`moveStart()`, a bonus feature — see §17); this keeps every already-computed
  key consistent relative to the new start without re-inserting the whole
  queue.
- **UpdateVertex(u)**: recompute `rhs(u)` from `u`'s successors (unless `u` is
  the goal), remove `u` from the queue, and re-insert it with a fresh key iff
  `g(u) != rhs(u)`.
- **ComputeShortestPath()**: repeatedly pop the minimum-key vertex while the
  queue's top key is less than `CalculateKey(start)` or `start` is locally
  inconsistent. For the popped vertex `u`:
  - If stale (`kOld < kNew`), simply re-insert with the fresh key.
  - If over-consistent (`g(u) > rhs(u)`): set `g(u) = rhs(u)` (this is a real
    improvement) and call `UpdateVertex` on every **predecessor** of `u` (they
    may now have a cheaper `rhs`).
  - Otherwise (`g(u) < rhs(u)`, an under-consistent state caused by a cost
    increase or an edge disappearing): set `g(u) = INF` and call
    `UpdateVertex` on `u` itself and every predecessor.
- **Stale queue entries**: `std::priority_queue` has no decrease-key
  operation, so updated entries are simply re-pushed; a side table
  `openKey_[id] -> currently valid key` lets `cleanStaleTop()` lazily discard
  any popped entry whose key no longer matches the side table before it is
  ever processed (`DStarLite.cpp`, `cleanStaleTop`/`queueInsertOrUpdate`).
- **Path reconstruction**: starting at `start`, greedily walk to the
  successor `v` minimizing `edgeCost(u,v) + g(v)` until the goal is reached
  (`reconstructAndValidate()`), then the result is **independently
  re-validated** against the raw problem data (§13, §30 of the assignment) —
  not against the planner's own internal `g`/`rhs` state.

## 9. Heuristic Function

`h(s, start) = weights.beta * EuclideanDistance(embedding(s), embedding(start))`.

**Admissibility.** D\* Lite's optimality guarantee requires `h(s,start)` to
never overestimate the true remaining cost from `s` to `start` under the
*actual* cost function the search minimizes. Since §11 below defines that
cost function as `beta*cost(u,v) + gamma*penalty + delta*penalty` with the
penalty terms always `>= 0`, admissibility reduces to requiring
`cost(u,v) >= EuclideanDistance(u,v)` for every edge — i.e. that a transition
never costs less than the straight-line distance it covers. **This invariant
is explicitly enforced**, not assumed:
- `GraphGenerator.cpp` sets `cost(u,v) = EuclideanDistance(u,v) * factor`,
  `factor` drawn uniformly from `[1.0, costSlack]` (`costSlack=1.4` by
  default) — so `cost >= distance` by construction for every randomly
  generated edge.
- Every hand-built test scenario in `TestScenarios.cpp` was checked and its
  costs adjusted so `cost(u,v) >= EuclideanDistance(u,v)` holds for every edge
  (documented inline at each transition).

Given that invariant and the triangle inequality on Euclidean distance,
`beta*EuclideanDistance(s,start) <= beta * (true cost-to-go)` for any path,
so the heuristic is admissible and, since it is also consistent
(`h(u,start) <= edgeCost(u,v) + h(v,start)` follows from the triangle
inequality the same way), D\* Lite's search never needs to re-expand a state
after it becomes locally consistent under a static graph.

## 10. Safety Handling

Bad-state avoidance is a **hard constraint**, enforced in three independent
places so a bug in any one layer cannot silently leak a bad state into a
result:

1. **Search time** (`DStarLite::edgeCost`): any edge whose destination is a
   bad state (other than the goal, which is separately validated to never be
   bad) returns `+INF`, unconditionally, regardless of `beta`/`gamma`/`delta`.
   An infinite cost can never be "outweighed" by a large enough safety
   *reward* elsewhere — this is the difference between a hard constraint and
   a penalty (§39 of the assignment explicitly asks for this distinction).
2. **Pre-flight validation** (`reconstructAndValidate`): if the initial state
   or the goal state is itself in the bad-state set, planning fails
   immediately with an explicit error, before any search runs.
3. **Independent post-hoc validation** (`metrics::validatePath`, called from
   `reconstructAndValidate` and re-implemented without touching any of the
   planner's internal `g`/`rhs` state): re-checks that no state on the
   returned path is in the bad-state set, among other structural checks
   (§30). `PlanningResult::badStatesVisited` is computed the same
   independent way and is asserted to be `0` in every unit test for a
   successful plan.

## 11. Cost, Safety and Reliability Optimization

The assignment's example objective is
`Score(P) = alpha*G - beta*C + gamma*D + delta*R`.

**Two distinct things must not be confused**, and this implementation keeps
them explicitly separate:

- **`Score(P)`** (`metrics::objectiveScore`) is the assignment's path-level
  *reporting* metric, computed once, after the fact, from the whole path:
  `G` (did we succeed), `C` (=`totalCost`, sum of raw `cost`), `D` (=
  `minimumSafetyDistance`, the true geometric §"Euclidean Safety Distance"
  metric over the *entire finished path*), and `R` (= cumulative reliability,
  product of transition reliabilities). This is what gets printed as
  "Objective Score" and written to the experiment CSVs.
- **The composite edge weight D\* Lite actually minimizes** (`DStarLite::edgeCost`)
  is a *per-edge, local* proxy that a shortest-path search can actually
  optimize incrementally:
  ```
  w(u,v) = beta*cost(u,v)
         + gamma*[ (1 - safety(u,v)) + 1/(eps + nearestBadDistance(v)) ]
         + delta*[ -ln(reliability(u,v)) ]
  ```
  This was a deliberate engineering decision (flagged as such — the PDF does
  not specify this formula): the global path metric `D(P) = min over s in P
  of (min over b in B of dist(s,b))` is not itself decomposable into a sum of
  independent edge weights, so it is approximated by penalizing, at each edge,
  how close the *destination* state is to the nearest bad state
  (`nearestBadDistance(v)`), plus the assignment-provided per-edge `safety`
  field. `-ln(reliability)` converts the *multiplicative* reliability
  objective into an *additive* per-edge penalty, so that summing it along a
  path equals `-ln(product of reliabilities) = -ln(R(P))`; minimizing it is
  therefore exactly equivalent to maximizing `R(P)`.

  **Why this matters:** an earlier draft of this planner computed edge cost
  from `cost(u,v)` alone and only used `gamma`/`delta` when *scoring* the
  finished path — meaning changing `gamma` never changed which path the
  search actually returned, silently failing Test Case 3's explicit
  requirement to "demonstrate the cost/safety tradeoff." This was caught
  during verification (§39/40) and fixed; Test Case 3 (§17 below) now shows
  a genuine crossover: at `gamma=0/1` the cheaper, closer-to-bad-state path
  is chosen; at `gamma=5/20` the search switches to the safer, more
  expensive path (see `results/safety_weight_sweep.csv`).

**What stays hard vs. what is optimized:** reaching the goal and avoiding bad
states are never subject to this weighting — an edge into a bad state is
`+INF` before `beta`/`gamma`/`delta` are even applied (§10). The weights only
trade cost against safety-*margin* and reliability **among paths that already
satisfy both hard constraints**, exactly as the assignment specifies
("optimization happens among valid paths").

**Weight semantics** (`ObjectiveWeights`, defaults in `Metrics.h`):
- `alpha` (default 100): reward for reaching the goal in `Score(P)`'s
  reporting formula only; does not affect search.
- `beta` (default 1.0): weight on cost, both in the search's composite edge
  weight and in `Score(P)`. Also scales the heuristic (§9) so admissibility
  is preserved regardless of `beta`.
- `gamma` (default 2.0): weight on safety margin. Larger `gamma` biases the
  search toward states farther from bad states, at the cost of a longer or
  more expensive route, and increases `Score(P)`'s reward for a larger
  `D(P)`.
- `delta` (default 10.0): weight on reliability. Larger `delta` biases the
  search away from low-reliability edges.

## 12. Dynamic Replanning

Six update operations are exposed (`DStarLite.h`), each doing the *minimum*
`UpdateVertex` work implied by the change and then calling
`ComputeShortestPath()` again — **never** `initializeSearch()` (which would
discard all `g`/`rhs` values and rebuild from scratch):

| Change | What gets invalidated | Incremental work |
|---|---|---|
| `setTransitionAvailability` / `removeTransition` | one edge's cost | `UpdateVertex` on the edge's tail only |
| `addTransition` | one new edge | `UpdateVertex` on the new edge's tail only |
| `addBadState` / `removeBadState` | the geometric proximity-penalty term for potentially many nearby edges (§11) | recompute the `O(\|S\|*\|B\|)` nearest-bad-distance table, then `UpdateVertex` on every vertex (still far cheaper than `initializeSearch`, since any vertex whose `rhs` doesn't actually change is a no-op re-insertion and `g` values are *reused*, not reset) |
| `updateGoal` | the anchor of the whole backward search | set `rhs(newGoal)=0`, enqueue it, and `UpdateVertex` the *old* goal (which is now an ordinary vertex) |
| `moveStart` (bonus) | the heuristic's reference point | bump `km += h(lastStart, newStart)` so every existing key stays valid without touching the queue |

In every case, `ComputeShortestPath()`'s incremental propagation (via
predecessor traversal, §8) settles only the region of the graph whose
shortest-path estimate actually changed — this is exactly what
`test_DynamicReplanningReusesSearch` in `tests/test_cases.cpp` checks
(replanning explores a bounded, small number of vertices, not the whole
graph), and what the experiment CSV's `replanExploredStates` column
demonstrates empirically (§19).

## 13. Data Structures

- `unordered_map<uint64_t, State>`, `unordered_map<uint64_t, Transition>` —
  O(1) lookup by id.
- `unordered_map<uint64_t, vector<uint64_t>>` (x2) — directed adjacency,
  successors and predecessors kept separate (§6).
- `unordered_map<uint64_t, double>` for `g`, `rhs`, and `nearestBadDist_`.
- `priority_queue<QueueEntry, vector<QueueEntry>, greater<QueueEntry>>` plus a
  side `unordered_map<uint64_t, Key> openKey_` implementing lazy deletion
  (§8) since the STL has no decrease-key.

## 14. Time Complexity

Let `|S|` = number of states, `|T|` = number of transitions, `|B|` = number of
bad states.

- `initializeSearch()`: `O(1)` (only the goal is seeded).
- `heuristic()`: `O(d)` per call (`d` = embedding dimensionality).
- `recomputeNearestBadDistances()`: `O(|S| * |B|)` — only called on
  `plan()`/`addBadState`/`removeBadState`, not on every edge relaxation.
- `UpdateVertex(u)`: `O(out-degree(u))` to recompute `rhs(u)`.
- `ComputeShortestPath()`: each state can be inserted into the queue multiple
  times (once per key change) but is only ever *expanded* (the `g(u)=rhs(u)`
  branch) a bounded number of times between two consecutive `plan`/`replan`
  calls; standard D\* Lite analysis gives `O((|T| + |S|) log |S|)` for a full
  initial solve, dominated by the log-time priority-queue operations. An
  *incremental* replan after a small local change costs
  `O(k log |S|)` where `k` is the number of vertices whose `rhs` actually
  changes — empirically far smaller than `|S|` (see §19).
- `reconstructAndValidate()`: `O(path length * out-degree)` to walk the path,
  plus `O(path length)` for independent validation.

## 15. Space Complexity

`O(|S| + |T|)` for the problem data and adjacency maps, plus `O(|S|)` for
`g`, `rhs`, `nearestBadDist_`, and the open-queue side table. The program
reports a live estimate (`PlanningResult::memoryBytesEstimate`) computed from
these container sizes (see `results/experiments.csv`).

## 16. Implementation

C++17, no external dependencies beyond the standard library. Project layout:

```
SafePath/
├── CMakeLists.txt
├── include/            State, Transition, PlanningProblem, PlanningResult,
│                       Planner, DStarLite, Metrics, GraphGenerator,
│                       VisualisationData, TestScenarios
├── src/                corresponding .cpp files + main.cpp (CLI)
├── tests/test_cases.cpp   31 unit tests, dependency-free
├── visualization/      visualize_graph.py, visualize_dynamic.py,
│                       plot_experiments.py, requirements.txt
├── data/generated/
├── results/            experiments.csv, safety_weight_sweep.csv, graphs/
└── report/report.md    (this file)
```

## 17. Test Cases

All six ran via `./safepath --demo` (full transcript in
`report/demo_output.txt`) and are individually reproducible with
`./safepath --test N`.

1. **Basic Reachability**: `S(0)->A(1)->B(2)->G(3)`, the unique path, cost
   `3.0`. **Result: PASS.**
2. **Bad State Avoidance**: the cheaper-looking route through bad state `X`
   is never even a candidate (edge cost `+INF`); the planner returns
   `S->C->D->G`, cost `4.0`, **`Bad states visited: 0`**. **Result: PASS.**
3. **Safety Margin**: with `gamma∈{0,1,5,20}`, the selected path flips from
   the cheap/unsafe route (cost `4.2`, min safety distance `0.30`) at
   `gamma<=1` to the expensive/safe route (cost `9.0`, min safety distance
   `2.00`) at `gamma>=5` — a genuine tradeoff driven by the search itself,
   not just the reported score (§11). **Result: PASS.**
4. **Dynamic Transition**: initial path `S->A->G` (cost `4.0`) becomes
   invalid when `A->G` is disabled; the planner replans to `S->C->D->G`
   (cost `6.6`) while re-exploring only 5 vertices (vs. 3 for the initial
   solve on this tiny graph — see §19 for the effect at scale).
   **Result: PASS.**
5. **Goal Update**: initial path to `G1` (`S->A->G1`, cost `4.0`) is replaced
   by a path to `G2` (`S->C->D->G2`, cost `7.2`) after `updateGoal(G2)`,
   reusing the existing search structures rather than rebuilding them (§12).
   **Result: PASS.**
6. **Transition Addition**: the only initial route is a `4.0`-cost detour
   (`S->A->B->G`, deliberately bent off the straight line — see §9 for why);
   adding a direct shortcut of cost `3.0` (== straight-line distance, so the
   heuristic stays admissible) causes the planner to switch to `S->G`
   directly, cost `3.0`. **Result: PASS.**

## 18. Experimental Setup

Random graphs generated by `graphgen::generate` (`GraphGenerator.cpp`), seed
`42` (configurable via `--seed`), sizes `{100, 500, 1000, 5000}`, 2D Cartesian
embeddings uniform in `[0,100]^2`, a guaranteed reachable backbone path plus
random extra edges (`edgeProbability` scaled down for larger `n` to keep
density reasonable), `numBadStates = max(1, n/20)`, edge cost tied to
Euclidean distance (§9). Every run additionally disables the first transition
of the found path and re-times a `replan()` call.

## 19. Experimental Results

Raw data: `results/experiments.csv`.

| n | success | totalCost | minSafetyDist | reliability | exploredStates | planningTime (ms) | replanTime (ms) | replanExplored |
|---|---|---|---|---|---|---|---|---|
| 100  | ✓ | 223.0 | 3.28 | 0.679 | 35  | 0.257 | 0.149  | 51  |
| 500  | ✓ | 83.9  | 2.43 | 0.798 | 8   | 1.140 | 1.416  | 31  |
| 1000 | ✓ | 164.8 | 1.26 | 0.854 | 230 | 8.815 | 9.959  | 293 |
| 5000 | ✓ | 116.5 | 1.65 | 0.640 | 226 | 34.96 | 11.04  | 16  |

All four instances succeeded with **zero bad states visited**. Planning time
grows with `n` but stays under 35 ms even at 5000 states — well within
"reasonable execution time." Explored-state counts do not grow monotonically
with `n` because they depend on the specific random topology (density,
distance from the found path to the backbone) rather than `n` alone, which is
expected for a heuristic search rather than an uninformed one. Replanning
after disabling a single edge is frequently *cheaper* than the initial solve
(e.g. `n=5000`: 34.96 ms initial vs. 11.04 ms replan, exploring only 16
vertices) — direct evidence that the incremental update is not rebuilding the
whole search (§12).

The safety-weight sweep (`results/safety_weight_sweep.csv`, Test Case 3's
scenario, `gamma` from 0 to 20 in steps of 2) shows the exact crossover: cost
jumps from `4.2` to `9.0` and `minSafetyDistance` from `0.3` to `2.0` between
`gamma=0` and `gamma=2`, then stays flat — the search has already switched to
the safer path and further `gamma` increases only widen the margin in the
reported `Score(P)`, not the chosen path.

## 20. Graph Visualization

`visualization/visualize_graph.py` reads a JSON file exported by
`--export FILE.json` (schema: `include/VisualisationData.h`) and plots the
graph using the states' **actual Cartesian embedding coordinates** as node
positions (assignment §17) — not a force-directed layout. States are colour-
and shape-coded (initial = blue square, goal = gold star, bad = red X,
normal = grey circle); the selected path is a bold green arrow chain,
unavailable transitions are dashed grey, all other transitions are faint;
edge costs are annotated; a legend is always included. For embeddings with
more than 3 dimensions, a PCA projection is used **for the plot only** — the
planner itself always operates on the full-dimensional embedding
(`project_to_2d` in `visualize_graph.py`).

Generated examples: `results/graphs/test1.png`, `test2.png`, `test3.png`.

## 21. Dynamic Replanning Visualization

`visualization/visualize_dynamic.py` takes the automatically-exported
`*_before.json` / `*_after.json` pair (`main.cpp` writes both for Test Cases
4–6) and renders a two-panel before/after comparison in one figure: the
"after" panel overlays the *previous* path as a faint dashed orange line so
the change is immediately visible against the new solid green path. See
`results/graphs/test4_dynamic.png` (edge removal), `test5_dynamic.png`
(goal change), `test6_dynamic.png` (shortcut addition).

## 22. Discussion

The implementation satisfies all five stated optimization objectives and all
six required test cases while keeping the two "hard vs. soft" halves of the
problem cleanly separated: bad-state avoidance and goal-reachability are
enforced structurally (infinite edge costs, independent path validation) and
can never be traded away by any weight setting, while cost, safety-margin,
and reliability are combined into a single tunable composite objective for
the search to actually optimize among the states that pass the hard filter.

## 23. Limitations

- The per-edge safety proxy (§11) approximates the true global path metric
  `D(P)`; it can occasionally under- or over-penalize a specific edge
  relative to what the *whole path's* minimum distance would suggest,
  because `D(P)` is a min over the whole path, not a sum over edges. The
  final, reported `D(P)` is always computed exactly (`metrics::minimumSafetyDistance`)
  regardless of this approximation — only the search's edge-by-edge
  guidance is approximate.
- `addBadState`/`removeBadState` currently re-examines every vertex
  (§12) because the proximity penalty is global; a spatial index (e.g. a
  k-d tree over bad-state positions) would let this be restricted to
  vertices within some radius, but was intentionally not added per §41
  ("do not overengineer").
- The graph generator's `unavailableProbability` and `edgeProbability` are
  fixed heuristics tuned for the four requested sizes; they are not
  automatically re-tuned for arbitrarily large `n`.
- Multi-goal planning, time-dependent availability, and parallel search
  (§42 bonuses) are not implemented, per §41/§42's guidance to only add
  bonus features once the core assignment is solid.

## 24. Conclusion

A complete, independently-validated, genuinely incremental D\* Lite planner
was implemented in C++17, satisfying every hard requirement (goal reachable,
zero bad states ever visited, dynamic replanning without full rebuilds) and
demonstrating a real, search-level cost/safety/reliability tradeoff rather
than a purely cosmetic one. All six required test cases and 31 unit tests
pass; experiments up to 5000 states complete in well under 100 ms.

## 25. Future Improvements

- Spatial indexing (k-d tree / grid) for `nearestBadDistance` to make
  bad-state changes as cheap, asymptotically, as edge changes.
- A properly weighted admissible heuristic that also accounts for `gamma`'s
  safety term (currently the heuristic only lower-bounds the `beta*cost`
  component; the `gamma`/`delta` terms are non-negative additions, which
  preserves admissibility but leaves those terms un-guided by the
  heuristic, costing some search efficiency at very high `gamma`/`delta`).
- Multi-goal planning and time-dependent availability (§42 bonuses).
