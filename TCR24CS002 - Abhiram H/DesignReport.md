# Design Report — Safe Semantic Planner (D* Lite)

## 1. State Representation

Each state `s_i ∈ S` is a point in `R^d`:

```cpp
class State {
    uint64_t id;
    std::vector<double> embedding; // (x1, ..., xd)
};
```

`id` gives O(1) hashing/lookup; `embedding` is used only for two purposes:
the heuristic (Sec. 3) and the safety metric "distance to nearest bad
state" (Sec. 4). The planner itself reasons over the *graph* induced by
`T`, not over raw coordinates — coordinates never gate reachability, they
only bias the search towards safer, closer-to-goal states.

## 2. Data Structures

| Structure | Purpose | Complexity |
|---|---|---|
| `stateOf_` (hash map id→State) | O(1) state lookup | O(1) avg |
| `transitionOf_` (hash map id→Transition) | O(1) edge lookup, supports live mutation of `.available`/`.cost` | O(1) avg |
| `outEdges_`, `inEdges_` (hash map id→vector\<id\>) | adjacency in both directions — D* Lite needs **predecessors** because it searches backward from the goal | O(deg) |
| `badStates_` (hash set) | O(1) bad-state membership test | O(1) avg |
| `g_`, `rhs_` (hash maps) | D* Lite's two cost estimates per state | O(1) avg |
| `openSet_` (`std::set<pair<Key,id>>`) + `openKeyOf_` (hash map) | priority queue with O(log n) insert/erase/top and O(log n) "is this vertex already open, with what key" lookup — the second map is what makes `UpdateVertex` cheap | O(log n) |

Using a *predecessor* list is what lets `updateVertex` be called only on
the handful of states actually affected by a change (a removed edge, a
newly-bad state, …), instead of rescanning the whole graph.

## 3. Heuristic Function

D* Lite requires `h(s, start)` to be **admissible and consistent** with
respect to the search-graph edge cost, or the algorithm's correctness
guarantees break down. We use:

```
h(s) = minCostRate * EuclideanDistance(s, start)
minCostRate = min over all transitions (t.cost / EuclideanDistance(t.from, t.to))
```

`minCostRate` is the cheapest observed "cost per unit of geometric
distance" anywhere in the graph. By the triangle inequality, no path from
`s` to `start` can cost less than `minCostRate * EuclideanDistance(s,
start)`, so `h` never overestimates the *raw* cost. Since the safety and
reliability penalty terms added in Sec. 4 are always `≥ 0`, `h` remains a
valid lower bound on the full safety-augmented edge cost too — it just
becomes a looser (but still admissible) bound. If the graph has no
geometric structure worth exploiting (e.g. all-zero distances), the code
falls back to `minCostRate = 0`, i.e. plain Dijkstra/uniform-cost search,
which is always correct, just less directed.

## 4. Safety Computation

Three independent signals from the brief — `cost`, `safety`,
`reliability` — plus the "distance to nearest bad state" requirement are
folded into a single non-negative search cost per edge `(u, v)`:

```
edgeCost(u,v) = cost(u,v)
              + λ_safety      * (1 − safety(u,v))
              + λ_reliability * (1 − reliability(u,v))
              + λ_proximity   / (distToNearestBad(v) + ε)
```

* `distToNearestBad(v)` — Euclidean distance from `v`'s embedding to the
  nearest bad state's embedding. As `v` approaches a bad state this term
  blows up, pushing the search away; `ε` (default `0.5`) just prevents a
  literal division by zero.
* Any edge whose `available` flag is false, or that leads **directly**
  into a bad state, is assigned `edgeCost = +∞` and is never selected —
  this is what gives the hard guarantee "never visit a bad state"
  (Optimization Objective #2), independent of the tunable weights.
* `λ_safety = 4.0`, `λ_reliability = 2.0`, `λ_proximity = 3.0` are
  constructor-configurable (`DStarLitePlanner::Weights`) so the
  cost/safety trade-off (`α,β,γ,δ` in the brief's `Score(P)`) can be
  retuned without recompiling logic, only the weight struct.

Two numbers are then reported per solved path (`PlanningResult`):
`totalCost` (sum of *raw* `cost`, for comparability across runs) and
`safetyScore` (the minimum, over all visited states, of
`distToNearestBad`) — directly answering Optimization Objectives #3 and
#4.

## 5. Time Complexity

Let `V` = number of states, `E` = number of transitions.

* **Initial `plan()`**: identical asymptotic bound to Dijkstra with a
  binary-heap-equivalent structure: `O((V + E) log V)`. Each vertex is
  popped from `openSet_` at most a small constant number of times before
  becoming consistent, and each pop triggers `O(deg⁻(u))` predecessor
  updates, each an `O(log V)` heap operation.
* **Incremental replan after a local change** (edge cost/availability
  flip, single bad-state add/remove, start move): `O((k + E') log V)`
  where `k` is the number of vertices whose `g`/`rhs` actually become
  inconsistent as a result — in practice a small, localized fringe
  around the change, not the whole graph. This is the entire point of
  using D* Lite over re-running Dijkstra/A* from scratch on every update.
* **Goal change**: handled as a bounded reinitialization (Sec. "Dynamic
  Environment" below) — `O(V + E)` to reset `g`/`rhs` plus a fresh
  `O((V+E) log V)` search. Still avoids re-parsing the problem or
  rebuilding adjacency lists.
* **Path extraction**: `O(P · deg⁺)` where `P` is the path length —
  greedy descent through `g`, no re-search needed.

## 6. Space Complexity

`O(V + E)`: two adjacency maps (`O(E)`), the state/transition tables
(`O(V + E)`), `g_`/`rhs_`/`openKeyOf_` (`O(V)` each), and `openSet_`
(`O(V)` worst case).

## Dynamic Environment / Replanning Strategy

The brief asks explicitly how the implementation replans efficiently
after goal changes, bad-state changes, and edge add/remove. The class
exposes this directly instead of forcing a re-solve of the whole problem:

```cpp
planner.initialize(problem);      // one-time setup
planner.setEdgeAvailability(id, false);
planner.addTransition(t);
planner.removeTransition(id);
planner.addBadState(id);
planner.removeBadState(id);
planner.updateStart(id);
planner.updateGoal(id);           // bounded reinitialization, see below
PlanningResult r = planner.replan();
```

* **Edge availability / cost changes, bad-state add/remove, start
  moves**: these call `updateVertex` only on the directly affected
  states (and propagate via the priority queue only as far as the change
  actually matters) — this is the classical D* Lite incremental-replan
  path. `km_` is accumulated exactly as in Koenig & Likhachev (2002) to
  keep keys ordered correctly as the search origin (`start_`) moves.
* **Goal changes**: D* Lite fixes `rhs(goal) = 0` as the anchor of the
  whole backward search; changing the goal invalidates every `g`/`rhs`
  value computed relative to the old anchor. We therefore treat a goal
  change as a **bounded reinitialization**: the graph, adjacency lists,
  and bad-state set are untouched (no re-parsing, no reallocation) — only
  `g_`, `rhs_`, and the open set are reset before a fresh
  `computeShortestPath()`. This is still asymptotically the same as one
  fresh solve, but avoids repeating the O(V+E) graph-construction work
  test case 5 implicitly asks planners to avoid. A fully incremental
  goal-change scheme exists (the "Focussed D*"/dual-graph trick of
  swapping search direction) and is noted as a natural extension in the
  Bonus section of the README.

## Test Case Results Summary

See `docs/ExperimentalResults.md` for the full table generated by
`./planner_demo` (also written machine-readably to `results.csv`). All
six illustrative test cases from the brief pass with the expected
qualitative behavior (unique path found, bad state avoided, safer-but-
costlier path chosen, and all three dynamic-update cases correctly
reroute without rebuilding the graph).
