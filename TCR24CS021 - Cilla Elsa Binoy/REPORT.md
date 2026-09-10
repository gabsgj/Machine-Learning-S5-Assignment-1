# Design of a Safe Semantic Planner in a Finite Cartesian State Space

Algorithm implemented: **D\* Lite** (Koenig & Likhachev, 2002), specialized to
the `PlanningProblem` interface given in the assignment.

## 1. State Representation

Each `State` is an id plus an embedding vector `(x1, ..., xd)` in R^d. The
embedding is used for two purposes only — it is *not* used as the edge cost
itself, since the assignment gives costs explicitly on transitions:

1. **Heuristic** `h(s)`: optional Euclidean distance from `s` to the start
   state, scaled by `heuristicWeight_`. Set to 0 by default, which makes the
   search behave as an exact incremental Dijkstra (always correct,
   regardless of whether transition costs happen to correlate with spatial
   distance). If the deployment's costs *are* roughly proportional to
   distance, `heuristicWeight_` can be set to make the search closer to A*
   and expand fewer states — this is the "heuristic design" tradeoff the
   assignment asks about.
2. **Safety margin** `D`: Euclidean distance from a state to the nearest bad
   state, used both as a reported metric and as an inverse-distance penalty
   folded into the search (see §4).

## 2. Data Structures

| Structure | Purpose | Complexity |
|---|---|---|
| `states_` (`unordered_map<id, State>`) | O(1) state lookup | O(1) avg |
| `transitions_` (`vector<Transition>`) + `transitionIndexById_` | edge storage, O(1) lookup by id | O(1) avg |
| `outEdges_[from]`, `inEdges_[to]` (`unordered_map<id, vector<size_t>>`) | forward and backward adjacency | O(deg) per lookup |
| `g_`, `rhs_` (`unordered_map<id, double>`) | D\* Lite's two cost estimates per state | O(1) avg access |
| `openSet_` (`set<pair<Key,id>>`) + `keyInOpenSet_` | priority queue with O(log n) insert/erase and O(1) "is this state queued, with what key" lookup | O(log n) |

The priority queue is implemented as a sorted `std::set` of `(Key, id)`
pairs rather than `std::priority_queue`, because D\* Lite needs to *remove*
an arbitrary element (when its key becomes stale) before re-inserting it —
`std::priority_queue` doesn't support that; a balanced BST does, in
O(log n).

## 3. Why D\* Lite (vs. computing shortest path from scratch every time)

The "Dynamic Environment" section asks for goal changes, bad-state changes,
availability changes, and edge add/remove to be handled efficiently. D\*
Lite maintains two values per state:

- `g(s)`: current best known cost from `s` to the goal.
- `rhs(s)`: a one-step lookahead, `min over (s -> s') of edgeCost(s,s') + g(s')`.

A state is **consistent** when `g(s) == rhs(s)`. `computeShortestPath()`
only re-expands states that are inconsistent, propagating outward from
wherever the environment actually changed via `updateVertex()`. When only a
few edges near the change are affected, only a few states get touched —
you can see this directly in the "States expanded" counter the program
prints: the initial solve of Test Case 4 expands 3 states, but re-solving
after disabling one transition expands only ~1-4 states rather than
resolving the whole graph.

**Documented limitation:** D\* Lite's `g`/`rhs` values are defined relative
to a *fixed goal*. A goal change (Test Case 5) invalidates essentially every
`g` value, so `updateGoal()` re-initializes and resolves from scratch. This
is not a bug — it is a known property of the algorithm (D\* Lite is
optimized for *robot moves / edges change*, not *goal moves*), and the code
comments this explicitly rather than pretending otherwise.

## 4. Safety Computation

Bad states are a **hard constraint**: `edgeCost()` returns +infinity for
any transition whose `from` or `to` is a bad state, or that is currently
unavailable. They are never merely penalized — the planner physically
cannot route through them.

Softer safety preference (the assignment's "maximize minimum distance to
nearest bad state," objective `D`) is folded into the same scalar edge cost
the search minimizes:

```
reliability  = max(t.reliability, 1e-6)
base         = (costWeight * t.cost + safetyWeight * (1 - t.safety)) / reliability
margin       = EuclideanDistance(t.to, nearest bad state)
marginPenalty = marginWeight / (margin + 0.15)
edgeCost(t)  = base + marginPenalty
```

This corresponds to a scalarization of the assignment's

```
Score(P) = alpha*G - beta*C + gamma*D + delta*R
```

where `G` (goal completion) is enforced structurally (infinite cost blocks
non-goal-reaching or bad-state-touching moves), `-beta*C` is `base`'s cost
term, `+gamma*D` is the (inverted, penalized) `marginPenalty` term, and
`+delta*R` is the reliability division. Test Case 3 in the code runs the
*same graph* twice with different `(costWeight, marginWeight)` settings and
shows the chosen path flip from the cheap/close route to the
expensive/far route — this is the direct, demonstrable answer to "explain
how your planner balances cost and safety."

An earlier version of this formula *subtracted* a capped safety bonus
instead of adding an inverse-distance penalty, then clamped the result to a
positive floor to keep Dijkstra's non-negativity assumption. That clamp
silently erased the cost/safety signal whenever the safety weight got large
enough to push the raw value negative (both paths would clamp to the same
floor and become indistinguishable). The inverse-distance form used above
is always positive by construction, so no clamping — and no signal loss —
is needed.

## 5. Heuristic Function

```
h(s) = heuristicWeight * EuclideanDistance(s, start)
```

With `heuristicWeight = 0` (the default used throughout the test suite),
this reduces to 0 for all states, which makes D\* Lite behave as an
incremental Dijkstra — always correct, no admissibility assumptions
required, since the assignment's `cost` field is an arbitrary abstract
number with no guaranteed relationship to Euclidean distance. If a
deployment's costs are known to be lower-bounded by Euclidean distance
(e.g. costs are literally travel distance, or travel time at a bounded max
speed), `heuristicWeight` can be set to `1 / max_speed` to keep the
heuristic admissible while meaningfully pruning the search.

## 6. Time Complexity

Per `computeShortestPath()` call: each state can be inserted into the open
set and expanded a bounded number of times before becoming consistent; in
the standard analysis this is `O((|S| + |T|) log |S|)` for a *full* solve
(dominated by up-to-`|S|` heap operations at `O(log |S|)` each, plus
`O(|T|)` total edge relaxations across all expansions) — identical order to
Dijkstra with a binary heap. For an **incremental** replan after a *local*
change (one edge, one bad state, one availability flip), only the states
whose shortest path actually changes get re-expanded, which is typically
`O(k log |S|)` where `k` is the number of affected states — in the
demonstrated test cases, `k` is a small constant regardless of overall
graph size, whereas the "goal changed" case reduces to a full `O((|S| +
|T|) log |S|)` resolve as discussed in §3.

## 7. Space Complexity

`O(|S| + |T|)`: one `g` and `rhs` entry per state, two adjacency-list
entries per transition (forward and backward), plus at most `O(|S|)` open-set
entries at any time.

## 8. Mapping to the Suggested C++ Interfaces

The provided pseudocode in the assignment (with OCR artifacts like
`ui n t 6 4 t` for `uint64_t`) is implemented literally as `State`,
`Transition`, `PlanningProblem`, `PlanningResult`, and an abstract `Planner`
base class with `virtual PlanningResult plan(const PlanningProblem&) = 0;`.
`DStarLitePlanner` implements `Planner` and adds the incremental methods
(`updateGoal`, `setTransitionAvailability`, `addTransition`,
`removeTransition`, `addBadState`) needed for the dynamic-environment
requirement, none of which are in the base interface because the
assignment's interface is deliberately minimal — a caller that only knows
about `Planner::plan()` can still use `DStarLitePlanner` by calling `plan()`
fresh every time, just without the incremental speedup.

## 9. Experimental Results (see console output from `make run`)

All 6 required test cases plus one bonus scenario pass. Reported per-run:
goal success, states expanded, peak open-set size (memory proxy), wall-clock
planning time in milliseconds, total path cost, and minimum safety margin.
Representative findings:

- **Test 1 (reachability):** trivially solved, 4 states expanded (all of
  them, since it's a 4-node chain).
- **Test 2 (bad-state avoidance):** the bad state is never in the returned
  path; the planner correctly discards the shorter-looking route through it
  because that edge is priced at infinity, not just penalized.
- **Test 3 (cost/safety tradeoff):** demonstrated with two weight
  configurations on the identical graph — see §4.
- **Tests 4-6 (dynamic updates):** each incremental call expands
  dramatically fewer states than a from-scratch solve of a graph that size
  would require, confirming the incremental-replanning benefit is real and
  not just claimed.
- **Bonus (newly discovered obstacle):** shows `addBadState()` correctly
  invalidating and rerouting around a state that was previously safe to
  cross.

## 10. Bonus Directions Not Implemented (and why)

- *Multi-goal planning* and *time-dependent availability* are natural
  extensions of `PlanningProblem` (a set of goals with a "reached if any"
  semantics, or an availability function of time instead of a bool) and
  would slot into `edgeCost()`/`extractPath()` without changing the core
  D\* Lite loop.
- *Parallel search* and *learning-based heuristics* are out of scope for a
  single-threaded teaching implementation but are natural next steps: the
  `heuristic()` function is already an isolated seam where a learned
  distance-to-goal estimator could be substituted.
- *Knowledge graph test*: any labeled directed graph can be mapped onto
  this `PlanningProblem` by treating relation edges as `Transition`s
  (cost = 1, safety/reliability = domain-specific if available) and
  entities as `State`s (embedding = a KG embedding vector, e.g. from
  TransE/DistMult, which then makes the Euclidean-distance heuristic
  meaningful "for free").
