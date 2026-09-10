# User Manual — SafePath

Assumes **Ubuntu/Linux**. All commands below are run from the project root
(`SafePath/`).

## 1. Prerequisites

- A C++17 compiler (verified with GCC 13.3.0 / Ubuntu 24.04). No third-party
  C++ libraries are required — only the standard library.
- Python 3.8+ with `matplotlib`, `networkx`, and `numpy` for the
  visualization scripts (planning itself does not need Python at all).
- (Optional) CMake 3.10+, if you prefer the CMake build over calling `g++`
  directly.

Check what you have:
```bash
g++ --version
python3 --version
cmake --version   # optional
```

## 2. Installation

Nothing to install beyond the compiler and Python packages:
```bash
pip install -r visualization/requirements.txt --break-system-packages
```
(Drop `--break-system-packages` if you're using a virtualenv.)

## 3. Compilation

### Option A — direct g++ (fastest, no CMake needed)
```bash
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude src/*.cpp -o safepath
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude tests/test_cases.cpp \
    src/State.cpp src/Transition.cpp src/DStarLite.cpp src/Metrics.cpp \
    src/GraphGenerator.cpp src/VisualisationData.cpp src/TestScenarios.cpp \
    -o test_runner
```

### Option B — CMake
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
# binaries land in build/safepath and build/test_runner
cd ..
```
Both options were verified to build cleanly with zero warnings under `-Wall -Wextra`.

## 4. Running the Six Test Cases

```bash
./safepath --test 1   # Basic Reachability
./safepath --test 2   # Bad State Avoidance
./safepath --test 3   # Safety Margin (cost/safety tradeoff sweep)
./safepath --test 4   # Dynamic Transition (edge removal)
./safepath --test 5   # Goal Update
./safepath --test 6   # Transition Addition (shortcut)
```
Each prints the scenario, the initial plan, (for 3–6) the dynamic change and
the replanned result, and all required metrics (path, cost, minimum safety
distance, reliability, objective score, explored states, planning time, bad
states visited).

## 5. Running Demonstration Mode

```bash
./safepath --demo
```
Runs all six test cases back-to-back with a summary at the end. Full sample
output is saved at `report/demo_output.txt`.

## 6. Running the Unit Tests

```bash
./test_runner
```
or, if you built with CMake:
```bash
cd build && ctest --output-on-failure
```
31 tests covering reachability, unreachable/bad initial/goal states,
bad-state avoidance, unavailable/added/removed transitions, goal updates,
safety-distance calculation, path validation, and dynamic replanning.

## 7. Running Experiments

```bash
./safepath --experiment            # seed 42 by default
./safepath --experiment --seed 7   # custom seed
```
Writes `results/experiments.csv` (graph sizes 100/500/1000/5000) and
`results/safety_weight_sweep.csv` (the Test-Case-3 scenario swept over
`gamma = 0..20`).

## 8. Generating Graph Visualizations

First export a graph + result as JSON (any test case, plus `--export`):
```bash
./safepath --test 2 --export results/graphs/test2.json
python3 visualization/visualize_graph.py results/graphs/test2.json
# writes results/graphs/test2.png ; add --show for an interactive window
```
Test cases 4, 5, and 6 automatically export **two** files when `--export` is
given — `..._before.json` and `..._after.json` — because they involve a
dynamic change:
```bash
./safepath --test 4 --export results/graphs/test4.json
# -> results/graphs/test4_before.json, results/graphs/test4_after.json
```

## 9. Generating Dynamic Replanning Visualizations

```bash
python3 visualization/visualize_dynamic.py \
    results/graphs/test4_before.json results/graphs/test4_after.json \
    --out results/graphs/test4_dynamic.png
```
Produces a side-by-side before/after panel with the superseded path drawn
as a faint dashed line in the "after" panel.

## 10. Generating Performance Plots

```bash
./safepath --experiment
python3 visualization/plot_experiments.py results/experiments.csv \
    --sweep results/safety_weight_sweep.csv --outdir results/graphs
```
Produces, in `results/graphs/`:
1. `plot1_planning_time_vs_states.png`
2. `plot2_explored_vs_states.png`
3. `plot3_initial_vs_replan_time.png`
4. `plot4_cost_vs_safety_weight.png`
5. `plot5_safety_distance_vs_weight.png`

All values are read from the CSVs the C++ program produced — nothing is
hard-coded in the plotting scripts.

## 11. Changing Configuration Parameters

- **Objective weights** (`alpha, beta, gamma, delta`): set
  `planner.weights.gamma = ...` etc. in C++ before calling `plan()` (see
  `runTest3` in `src/main.cpp` for a worked example), or edit the defaults in
  `include/Metrics.h`.
- **Random graph generation** (`GraphGenConfig` in `include/GraphGenerator.h`):
  `numStates`, `dimensions`, `areaMin/areaMax`, `edgeProbability`,
  `numBadStates`, `seed`, `costSlack`, `minReliability/maxReliability`,
  `minSafety/maxSafety`, `unavailableProbability`. Adjust in
  `runExperiments()` in `src/main.cpp`, or construct your own `GraphGenConfig`.
- **Experiment sizes/seed**: `--seed N` on the command line; graph sizes are
  set in `runExperiments()` (`std::vector<int> sizes = {100, 500, 1000, 5000};`).

## 12. Interpreting the Output

- `Success`: whether a valid path was found.
- `Path (states)`: the returned state sequence.
- `Total Cost`: sum of `cost` over the transitions used (raw, not weighted).
- `Minimum Safety Distance`: `D(P)`, the true geometric minimum Euclidean
  distance from any visited state to the nearest bad state (`inf`/blank if
  the problem has no bad states at all).
- `Reliability`: product of transition reliabilities along the path.
- `Objective Score`: `Score(P) = alpha*G - beta*C + gamma*D + delta*R`.
- `Explored States`: number of states popped from the open queue during that
  particular `plan()`/`replan()` call — the key evidence for "incremental,
  not full rebuild."
- `Planning Time` / replanning time: wall-clock milliseconds.
- `Bad States Visited`: must always read `0` for a successful plan; this is
  independently re-derived from the path, not trusted from the search.
