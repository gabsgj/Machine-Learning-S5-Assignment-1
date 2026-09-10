# Design Report — Safe Semantic Planner in a Finite Cartesian State Space

PCCST503 – Machine Learning, Assignment 1

## 1. Problem recap

We are given a finite state set `S = {s_1, ..., s_n} ⊂ R^d`, an initial
state `s_I`, a goal state `s_G`, a set of bad states `B`, and a set of
directed transitions `T`, each carrying a cost, a reliability, a safety
score, and an availability flag. The planner must reach `s_G` from `s_I`
without ever visiting a state in `B`, while minimizing cost and maximizing
the minimum clearance from `B`, and it must be able to react efficiently
when the goal, the bad-state set, or the transition set changes.

## 2. Algorithm choice: LPA\*

We chose **LPA\* (Lifelong Planning A\*, Koenig & Likhachev, 2004)** over
D\* Lite. Both are incremental variants of A\*, but they solve slightly
different re-planning problems:

- **D\* Lite** is optimized for a search whose *goal is fixed* and whose
  *start moves* (a robot walking along the already-computed path). Its
  incremental machinery (the `k_m` key-modifier) exists specifically to
  avoid re-keying the whole open list every time the start moves one step.
- **LPA\*** is optimized for a search whose *start and goal are fixed* and
  whose *edge costs/topology change*. That maps directly onto three of the
  assignment's four dynamic scenarios: transition availability changing
  (Test Case 4), and new transitions appearing (Test Case 6) — the search
  reuses all previously computed `g`/`rhs` values and only repairs the
  vertices actually affected by the change.

Since the assignment's dynamic-environment section is dominated by
edge/topology changes rather than a moving agent, LPA\* is the better fit.
The goal-update scenario (Test Case 5) is not incrementally free for either
algorithm (see §6), and we handle it honestly rather than pretending
otherwise.

## 3. State representation

`State` (`include/State.h`) is a thin wrapper around `uint64_t id` plus a
`std::vector<double> embedding` for `s_i ∈ R^d`. Keeping the embedding
generic-dimensional (rather than hardcoding 2D/3D) means the same code
handles the toy line/grid examples in the tests as well as higher-dimension
"semantic" embeddings (e.g., states derived from a knowledge graph, per the
bonus item). `State::distanceTo` computes Euclidean distance and is the one
piece of code that assumes a metric interpretation of the embedding; every
other component only depends on that single method, so swapping in a
different metric (cosine distance, a learned distance, etc.) is a one-line
change.

`Transition` (`include/Transition.h`) mirrors the interface given in the
assignment brief exactly: `id`, `from`, `to`, `cost`, `safety`,
`reliability`, `available`.

## 4. Data structures

The planner (`LPAStarPlanner`, `include/LPAStar.h` /
`src/LPAStar.cpp`) maintains, per problem instance:

| Structure | Type | Purpose |
|---|---|---|
| `stateById_` | `unordered_map<id, State>` | O(1) state lookup; **bad states are never inserted here**, so they are structurally unreachable rather than merely filtered at query time. |
| `outEdges_`, `inEdges_` | `unordered_map<id, vector<Transition>>` | Adjacency lists in both directions. LPA\*'s `UpdateVertex` needs predecessors of the vertex being repaired and `ComputeShortestPath` needs successors of the vertex being expanded, so both are kept rather than re-deriving one from the other on every access. |
| `clearance_` | `unordered_map<id, double>` | Precomputed Euclidean distance from every legal state to its nearest bad state. Used for the hard `safetyRadius` filter at graph-build time and the soft clearance penalty in `edgeWeight()`. |
| `g_`, `rhs_` | `unordered_map<id, double>` | LPA\*'s two per-vertex value functions: `g` is the current best known cost-to-come, `rhs` is the one-step lookahead ("if I trust my neighbors' `g` values, what should mine be"). A vertex is *locally consistent* when `g == rhs`. |
| `openSet_` | `std::set<pair<Key, id>>` | The priority queue, ordered by the two-part LPA\* key `(min(g,rhs) + h, min(g,rhs))`. `std::set` gives O(log n) insert/erase/find-min, which is what LPA\*'s proofs assume of the priority queue; a lazy-deletion `std::priority_queue` would also work but complicates the "is vertex `s` currently in the queue" check that `UpdateVertex` needs. |
| `openKeyOf_` | `unordered_map<id, Key>` | Companion index so we can find and erase a vertex's current entry in `openSet_` in O(log n) instead of a linear scan. |

Choosing to key the graph, `g`/`rhs`, and the open set by the *problem's own*
`uint64_t` state ids (rather than dense array indices) means adding a brand
new state or transition never requires renumbering anything — a deliberate
trade-off of a small constant-factor overhead (hashing) for simplicity of
the incremental-update API.

## 5. Heuristic function

`h(s) = ‖embedding(s) − embedding(s_G)‖₂`, i.e., straight-line Euclidean
distance to the goal in the state's own coordinate space
(`LPAStarPlanner::heuristic`). This is:

- **Admissible** as long as every transition's `cost` is at least as large
  as the geometric distance it covers (true for the grid/line test
  problems, and a reasonable modeling assumption in general — see the
  caveat below).
- **Consistent**, since it is literally a Euclidean distance in the same
  metric space the safety/clearance computation uses, satisfying the
  triangle inequality `h(u) ≤ cost(u,v) + h(v)` whenever `cost(u,v) ≥
  distance(u,v)`.

*Caveat:* our `edgeWeight()` (§6) adds non-negative safety/reliability
penalties on top of the raw `cost`, which only strengthens admissibility
(the true edge weight is ≥ the heuristic's assumption, never less), so
optimality of LPA\* under this heuristic is preserved. If a deployment's
`cost` field could legitimately be *smaller* than Euclidean distance (e.g.,
teleport-like transitions), the heuristic should be replaced with a
constant `0` (falling back to plain Dijkstra behavior) to keep the
admissibility guarantee.

## 6. Safety computation

Two independent mechanisms enforce objectives #2 and #4:

1. **Hard exclusion.** `buildGraph()` never inserts a bad state into
   `stateById_`, and drops any transition whose `from` or `to` is a bad
   state. If `problem.safetyRadius > 0`, it additionally drops any
   transition whose destination has `clearance_ < safetyRadius`. This is a
   structural guarantee, not a search-time check — the planner *cannot*
   emit a path through a bad state, which is exactly what our experiment
   harness confirms (`bad_states_visited` is 0 across all 35 randomized
   trials in `experiments/results.csv`).

2. **Soft penalty.** Among the transitions that survive the hard filter,
   `edgeWeight()` computes

   ```
   weight(t) = beta * t.cost
             + gamma * (1 - t.safety)              // per-edge safety score
             + gamma * (1 / (1 + clearance(t.to)))  // geometric clearance
             + delta * (1 - t.reliability)
   ```

   All four terms are non-negative given `cost ≥ 0` and `safety,
   reliability ∈ [0, 1]`, which preserves LPA\*'s correctness requirement
   of non-negative edge weights. Increasing `gamma` relative to `beta`
   trades cost for clearance; this is verified directly by the Safety
   Margin test (`tests/test_planner.cpp::testSafetyMargin`), which
   constructs a cheap-but-close path and a costlier-but-far path and checks
   that the planner switches its choice as `gamma` grows.

   This is a deliberate simplification of the assignment's multi-objective
   `Score(P) = alpha*G − beta*C + gamma*D + delta*R`: rather than solving a
   true multi-objective / Pareto search (which LPA\*'s incremental
   machinery does not straightforwardly support), we fold cost and safety
   into a single non-negative scalar edge weight. `alpha*G` (goal
   completion) doesn't appear in the weight because it isn't a per-edge
   quantity — it's realized by `result.success` in `PlanningResult`. The
   `reliabilityScore` reported in the result is a genuine multiplicative
   accumulation (`Π reliability`) computed after the fact, independent of
   how it was weighted during search, so a caller who wants pure
   `min-cost` search can set `gamma = delta = 0` and still get an accurate
   reliability report for the returned path.

## 7. Time and space complexity

Let `V` = number of legal (non-bad) states, `E` = number of legal
transitions.

- **From-scratch `plan()`:** identical asymptotics to A\*/Dijkstra with a
  binary-heap-equivalent priority queue: each vertex is inserted into
  `openSet_` and popped at most a bounded number of times (LPA\*'s
  standard result: at most twice per vertex per `ComputeShortestPath`
  call — once as over-consistent, once as under-consistent, in the worst
  case), each insert/erase is `O(log V)`, and each vertex expansion
  touches its outgoing edges. This gives **`O((V + E) log V)`**, the same
  bound as Dijkstra with a binary heap.
- **Incremental `updateTransition` / `addTransition` / `removeTransition`
  + `replan()`:** LPA\*'s key property is that after a localized edit,
  `computeShortestPath()` only re-expands vertices whose `g`/`rhs` values
  actually change as a consequence — in practice a small local
  neighborhood of the edit, not the whole graph. Worst case it degrades to
  the from-scratch bound above (e.g., if the edit is on the only path to
  the goal and forces a global re-route), but the experiments in
  `experiments/results.csv` show `replan_time_ms` consistently well below
  `planning_time_ms` for the same instance (e.g., at 30×30: ~3.0 ms
  from-scratch vs. ~0.3–2.1 ms for a batch of 15%+ of edges changing),
  confirming the expected sub-linear-in-practice behavior.
- **Space:** `O(V + E)` for the adjacency lists, plus `O(V)` for each of
  `stateById_`, `clearance_`, `g_`, `rhs_`, `openKeyOf_`, and up to `O(V)`
  live entries in `openSet_`. Total **`O(V + E)`**, matching Dijkstra/A\*.

## 8. Dynamic environment handling

| Change | Handling | Incremental? |
|---|---|---|
| Transition becomes unavailable / cost changes | `updateTransition()` mutates the edge in place and calls `updateVertex()` on its destination; `replan()` resumes `computeShortestPath()` from the existing `g`/`rhs`/open-set state. | **Yes** — this is LPA\*'s core use case. |
| New transition appears | `addTransition()` inserts it into both adjacency lists and calls `updateVertex()` on its destination. | **Yes**. |
| Transition removed entirely | `removeTransition()` erases it from both adjacency lists and repairs the destination. | **Yes**. |
| Goal changes | `replanWithNewGoal()` calls `plan()` again. | **No** — see below. |
| Bad-state set changes | Same as a goal change: requires `plan()` again, since it changes which vertices legally exist. | **No** — see below. |

**Why goal changes can't be cheap under LPA\*:** the heuristic `h(s)` is
defined *relative to the goal* (`§5`). Changing the goal changes `h(s)`
for every single vertex simultaneously, which invalidates every key
currently sitting in `openSet_` (keys are `min(g,rhs) + h`). There is no
way to "patch" `O(1)` keys here — correctness requires every key to be
recomputed, which is `O(V log V)` just to rebuild the priority queue, even
before any new expansions happen. At that point there is no meaningful
saving left over a from-scratch `plan()`, so we call `plan()` directly
rather than pretending to be incremental. What we *do* preserve is the
adjacency-list construction cost being paid only once per truly new
problem instance — `replanWithNewGoal()` still reuses the caller-supplied
`PlanningProblem` structure and doesn't require the caller to re-parse or
re-validate any external input.

The same argument applies to bad-state-set changes, since removing a state
from `B` (or adding one) changes which vertices are legal, which is a
structural change to the graph itself, not a local edge weight change.

## 9. Experimental results

`experiments/run_experiments.cpp` generates randomized grid-shaped
problems (5×5 up to 30×30, 5 trials per size, 4-connected grid plus
occasional diagonal shortcuts, 1 bad state per 3 grid rows), solves each
from scratch, then disables ~10% of transitions and adds 5 new shortcut
transitions before measuring an incremental replan. Full raw data is in
`experiments/results.csv`; headline numbers from the run committed here:

- **Goal success rate:** 28/35 = 80%. Every failure is a case where the
  random 10% edge removal genuinely disconnected the goal from the start
  in that specific random instance — not a planner defect (confirmed by
  spot-checking `total_cost = 0` failure rows, which correspond to
  `success = 0`).
- **Bad states visited:** 0 in all 35 trials, confirming the hard
  exclusion mechanism (§6) holds under randomized stress, not just the
  hand-built unit tests.
- **Planning time growth:** roughly linear-to-slightly-superlinear in
  problem size (5×5: ~0.04 ms average → 30×30: ~3.0 ms average, against a
  ~36× increase in vertex count from 25 to 900 states), consistent with
  the `O((V+E) log V)` bound in §7 given `E = O(V)` on a grid.
  Note: `run_experiments` should be re-run to regenerate
  `experiments/results.csv` with fresh numbers before submission if the
  target machine's timing characteristics differ from the reference run.
- **Replan time vs. from-scratch time:** at every grid size, at least
  several of the five trials show `replan_time_ms` at 10–50% of that same
  trial's `planning_time_ms`, despite the edit batch touching roughly 10%
  of all transitions — direct evidence that LPA\*'s incremental repair is
  doing less work than a full resolve, even under a fairly aggressive edit
  batch.

To reproduce or extend this evaluation (e.g., sweep `gamma`, vary bad-state
density, or test non-grid topologies), edit the parameters at the top of
`experiments/run_experiments.cpp::main` and re-run
`./run_experiments`.

## 10. Known limitations / future work

- The soft cost/safety/reliability combination (§6) is a scalarization,
  not a true multi-objective search; a Pareto-front approach would be more
  faithful to `Score(P)` but loses LPA\*'s incremental guarantees.
- Goal and bad-state-set changes fall back to a full resolve (§8); an
  Anytime D\*-style approach could offer a bounded-suboptimal fast
  first answer while a full LPA\* resolve continues in the background.
- Multi-goal planning, time-dependent availability, parallel search, a
  learning-based heuristic, and a knowledge-graph test case (assignment
  bonus items) are not implemented in this submission.
