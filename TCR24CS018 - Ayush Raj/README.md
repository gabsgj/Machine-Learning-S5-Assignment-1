# Quality-Aware D* Lite

Demonstration: https://path-planning-ecru.vercel.app/

A small C++17 implementation of backward D* Lite with a quality-aware second queue key. Travel cost remains the `g/rhs` Bellman quantity; reliability and normalized obstacle clearance bias queue and extraction priorities.

Bad states and edges into them are excluded. Call `plan()` again with an updated `PlanningProblem` after a goal, bad-state, transition-availability, or transition-set change. The planner rebuilds its graph indexes from the supplied problem and updates its D* Lite state before extracting the revised path.

Build and test:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The quality key is a heuristic extension, not a proof of global optimality for the non-additive `cost / quality` objective.

