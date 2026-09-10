# Student Details

Name: Madhav Krishna T
Register Number: TCR24CS043

---

# PCCST503: Safe Semantic Planner & Live D* Lite Visualizer

A high-performance **Transposed D* Lite Planning Engine** with spatial hazard exclusion, multi-objective trade-off analysis, dynamic edge and goal replanning, and an interactive real-time Cartesian SVG dashboard.

---

## 🌟 Overview

The **Safe Semantic Planner** solves multi-objective path planning over topological graphs embedded in metric space. It implements:

- **Stage 1 (Safety Exclusion & Spatial Indexing)**: $N$-dimensional K-d Tree indexing over hazard/bad states to enforce hard safety exclusion radius buffers ($r$).
- **Stage 2 (Transposed D* Lite Core)**: Backward incremental heuristic search on the transposed graph, rooted at a fixed $s_{\text{init}}$ with dynamic target goals $s_{\text{goal}}$. Handles lazy priority updates, edge degradation/severing, goal shifts ($k_m$), and unreachability termination without priority queue exceptions.
- **Stage 3 & 4 (WASM & Web Worker Integration)**: Stateful C++ boundary exposed via Emscripten and executed entirely within a Web Worker (`planner_worker.js`) to guarantee a non-blocking, responsive UI.
- **Interactive Dashboard**: Static HTML/CSS/SVG dashboard featuring preset test cases (TC1–TC6), click-to-set-goal, interactive edge severing/restoring, and discoverable dormant shortcut transitions (⚡).

---

## 🧠 Design & Implementation Notes

This section documents the reasoning behind the core architecture — the constraints that shaped it, and the bugs that shaped it further.

### State space & the safety objective

States are pre-enumerated graph nodes carrying a Cartesian embedding, not points in a continuously searchable space — the embedding is used only for the safety-distance computation, never for navigation. The assignment's "maximize minimum distance to bad states" objective is a **bottleneck (min-over-path) term**, which doesn't decompose additively the way Dijkstra/A*/D* Lite require (`g(v) = g(u) + w(u,v)` assumes a sum, not a min). Rather than implement a heavier Pareto/bottleneck-aware search for a term the spec treats as a constraint in spirit ("never visit a bad state"), safety is enforced as a **hard exclusion radius `r`**: any state within `r` of a bad state is removed from the searchable graph at load time via a k-d tree spatial query. `D` (minimum clearance) is then reported as a metric on the resulting path, not something the search itself optimizes over a min.

### The combined objective and its correctness constraint

The search minimizes `w(u,v) = β·cost(u,v) − δ·reliability(u,v)`, with reliability kept as a literal per-edge sum (matching the assignment's wording) rather than a log-probability product. This creates a real correctness risk Dijkstra-family algorithms don't tolerate: if `δ·reliability > β·cost` for some edge, `w(u,v)` goes negative, breaking the non-negative-edge-weight assumption every one of these algorithms depends on. `ProblemLoader` validates this at load time:

```
w_min = β · min(cost) − δ · max(reliability)
```

and rejects construction if `w_min < 0`, naming the violating bound, rather than silently clamping or rescaling weights the caller didn't ask to change. For the deployed sandbox scenario at default weights (β=1.00, δ=0.15): `w_min = 1.00×0.85 − 0.15×0.99 = 0.7015 > 0` — valid. Because `w_min` is a function of `δ`, there's a scenario-specific ceiling above which *any* weight combination becomes invalid (≈0.86 for this graph's cost/reliability bounds) — the UI's weight sliders are bounded below that ceiling so ordinary interaction can't trigger the rejection path live.

### Transposed D* Lite

The planner runs D* Lite on the **transposed graph**: the fixed initial state plays D* Lite's normal "goal" role, and the mutable goal state plays D* Lite's normal "start" role. D* Lite is specifically optimized for its start moving (the classic robot-navigation case); by swapping which endpoint is fixed, a *goal* change in this problem becomes exactly that "start moved" case, letting `notify_goal_changed` reuse D* Lite's incremental-replan machinery (the `km` key-modifier accumulation) instead of requiring a near-full resolve. Edge cost/availability changes (`notify_edge_changed`) work identically regardless of graph direction.

### Heuristic admissibility

`h(s, s_goal) = β · c_min · euclidean_distance(s, s_goal)`, where `c_min` is the minimum observed `cost / distance` ratio across all transitions (computed once at load, ≈0.803 for the sandbox scenario). This guarantees `h` never overestimates the true remaining cost. The reliability term `−δ·reliability(u,v)` is safely omitted from the heuristic rather than estimated, since it's bounded below by `−δ` regardless of path — admissibility only requires a valid *lower* bound, and zero is one.

### A key correctness bug worth documenting explicitly

D* Lite's priority queue orders entries by a two-part key `(k1, k2)`. The original comparator used `std::abs(k1 - other.k1) > 1e-9` to decide ordering — but when both keys are `Infinity` (as happens for any unreached node), `abs(∞ − ∞)` evaluates to `NaN`, and `NaN > 1e-9` is `false`, silently breaking the ordering contract for exactly the nodes an unreachable-goal or heavily-disconnected graph produces most of. The fix adds an equality short-circuit (`k1 != other.k1 && ...`) before the subtraction — `∞ == ∞` is `true` in IEEE 754, so the NaN-producing branch is never reached for genuinely equal infinite keys. This was the root cause behind an early "UI freezes on heavy edge deletion" symptom, and is now covered by dedicated disconnect/reconnect regression tests (TC7, TC8) plus interleaved-operation tests (TC9–TC11) confirming goal-shift and edge-sever remain correct and order-independent when combined in the same session.

### Instrumentation

Metrics not observable from outside the core are self-reported by the engine rather than inferred by the browser: an expansion counter increments on each priority-queue pop that leads to a real state expansion (reset separately for initial-solve vs. replan calls, so the efficiency gap between them is directly visible), and memory usage is computed analytically (`sizeof × count` over the g/rhs map, open list, and k-d tree) rather than read from `performance.memory`, which reflects browser/WASM allocator overhead rather than the planner's actual data structures.

### WASM boundary

The core is exposed as a **stateful, handle-based** API (`create_planner` → handle; `notify_edge_changed(handle, ...)`, `notify_goal_changed(handle, ...)` mutate that same instance in WASM memory) rather than a one-shot `solve()` call per interaction. This is deliberate: D* Lite's entire value proposition is that incremental replanning is cheap relative to a full resolve, and a one-shot API would silently degrade every interaction to a from-scratch solve regardless of what the core actually implements. All boundary marshaling uses JSON strings (`nlohmann::json`) rather than Embind type bindings, for debuggability at negligible cost at this graph scale. Solving runs inside a Web Worker so a slow or pathological query degrades to a loading state rather than freezing the tab.

### Sandbox topology

The interactive sandbox graph was empirically audited and redesigned to guarantee genuine route redundancy rather than a single fragile corridor: three distinct S→G route families (a low-cost central expressway passing near two hazards, a low-cost southern highway, and a high-reliability northern bypass), validated against the real WASM engine — not a parallel reimplementation — confirming a cost/reliability crossover point at `δ ≈ 0.48`, below which the cheaper near-hazard route wins and above which the planner shifts to the safer, costlier bypass. This is the live demonstration of the assignment's cost-vs-safety tradeoff requirement (Test Case 3).

---

## 🚀 Live Demo

- **Vercel URL**: `https://safe-semantic-planner.vercel.app` *(or your deployed Vercel deployment domain)*

---

## 🛠️ Local Development & Quick Start

### 1. Run Everything (Tests + Local Dashboard)

To build and run all 11 C++ test suites and launch the local HTTP server:

```
powershell -ExecutionPolicy Bypass -File run.ps1
```

Or run the test suite only:

```
powershell -ExecutionPolicy Bypass -File run.ps1 -TestOnly
```

### 2. View the Dashboard

Serve the project root with any standard HTTP server:

```
python -m http.server 8080
```

Open <http://localhost:8080> in your browser.

---

## 🔨 Rebuilding the WASM Module Locally

If you modify any C++ source code in `src/` or headers in `include/`, rebuild the WASM module using Emscripten (`emcc`):

### Option A: Shell Script (Linux / macOS / Git Bash)

```
bash build_wasm.sh
```

### Option B: Direct `emcc` Command

```
mkdir -p wasm
emcc -std=c++14 -O2 -I include \
    src/problem_loader.cpp \
    src/kd_tree.cpp \
    src/dstar_lite.cpp \
    src/wasm_api.cpp \
    -s WASM=1 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME="'PlannerModule'" \
    -s EXPORTED_FUNCTIONS="['_create_planner','_notify_edge_changed','_notify_goal_changed','_notify_bad_states_changed','_recompute_with_weights','_get_metrics','_get_result','_destroy_planner','_malloc','_free']" \
    -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap','UTF8ToString','stringToUTF8','lengthBytesUTF8']" \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s INITIAL_MEMORY=16777216 \
    -s NO_EXIT_RUNTIME=1 \
    -s ENVIRONMENT='web,node' \
    --no-entry \
    -o wasm/planner.js
```

This generates `wasm/planner.js` and `wasm/planner.wasm`.

---

## 🧪 Benchmark Test Cases

| Suite    | Description                                   | Invariant Verified                                                            |
| -------- | --------------------------------------------- | ----------------------------------------------------------------------------- |
| **TC1**  | Nominal Baseline Planning                     | Optimal path when $r = 0$, zero bad states visited.                           |
| **TC2**  | Safety Margin Sweep ($r \in [0.5, 1.5, 2.5]$) | Clean spatial detour around hazard zones.                                     |
| **TC3**  | Multi-Objective Weight Validation             | Throws a validation error when $w_{\min} < 0$.                                |
| **TC4**  | Dynamic Edge Degradation                      | Incremental local subtree repair via `notify_edge_changed`.                   |
| **TC5**  | Transposed Goal Shift                         | Fast goal shift via $k_m$ accumulator and `notify_goal_changed`.              |
| **TC6**  | Unsolvable Graph & Safety Barrier             | Blocked corridors cleanly return `success = false`.                           |
| **TC7**  | Empty Priority Queue Recovery                 | Full disconnect exits cleanly with `success = false`; restores on reconnect.  |
| **TC8**  | Grid Cut Disconnect & Restoration             | Vertical cut cleanly detected; single bridge restores optimal path.           |
| **TC9**  | Off-Backbone Goal Shift                       | Disjoint branch search from previous solve cleanly navigates to new goal.     |
| **TC10** | Goal Shift → Edge Sever                       | Shift to off-backbone goal followed by edge sever reroutes to alternate path. |
| **TC11** | Edge Sever → Goal Shift                       | Reversed operation ordering confirms order-independent consistency.           |

---

## 📦 Deploying to Vercel

1. Push this repository to GitHub / GitLab.
2. In the [Vercel Dashboard](https://vercel.com/new):
   - Click **Import Project** and select this repository.
   - Set **Framework Preset** to `Other`.
   - Leave **Build Command** and **Output Directory** empty / default.
   - Click **Deploy**.
3. `vercel.json` ensures all `*.wasm` assets are served with the correct `Content-Type: application/wasm` and security headers.
