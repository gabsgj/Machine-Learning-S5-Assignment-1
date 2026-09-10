# Experimental Results

## Method

The experiments use the C++17 `quality_dstar_tests` executable. Each scenario is a small, deterministic directed graph from the assignment's illustrative test cases. A result is successful only when the returned state path and total cost match the expected values; the safety-margin scenario also verifies that the safety-focused plan has a higher normalized safety score than the cost-focused plan.

Run the suite with:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/quality_dstar_tests
```

## Results

| Test case | Scenario / change | Returned path | Total cost | Explored states | Planning / replanning time | Memory usage | Verification result |
| --- | --- | --- | ---: | --- | --- | --- | --- |
| 1. Basic reachability | One unique route from `S` to `G` | `S -> A -> B -> G` | 3.00 | 4 | 0.039596 ms / 0 ms | 5,816.32 kB | PASS — reaches the goal through the unique valid path. |
| 2. Bad-state avoidance | `X` is marked as a bad state | `S -> C -> D -> G` | 3.00 | 4 | 0.035458 ms / 0 ms | 5,816.32 kB | PASS — the route through `X` is rejected; bad states visited: 0. |
| 3. Safety margin | Safety-only weights | `S -> upper route -> G` | 6.00 | 5 | 0.035287 ms / 0 ms | 5,816.32 kB | PASS — selects the higher-clearance route. |
| 3. Safety margin | Cost-only weights | `S -> near-obstacle route -> G` | 2.00 | 5 | 0.036029 ms / 0 ms | 5,816.32 kB | PASS — selects the lower-cost route and has a lower safety score. |
| 4. Dynamic transition | Original middle-to-goal transition becomes unavailable | `S -> alternate -> G` | 4.00 | 6 | 0.042171 ms / 0.042171 ms | 5,816.32 kB | PASS — replans through the available alternative. |
| 5. Goal update | Goal changes from `G` to the alternate state | `S -> alternate` | 2.00 | 3 | 0.024476 ms / 0.024476 ms | 5,816.32 kB | PASS — replans to the revised goal. |
| 6. Transition addition | Direct shortcut `S -> G` is added | `S -> G` | 0.25 | 5 | 0.045597 ms / 0.045597 ms | 5,816.32 kB | PASS — discovers the improved solution. |

## Summary

All six illustrative test cases pass. Every successful scenario reaches its requested goal, and the only scenario containing a bad state verifies that no bad state is included in the returned path. The safety-margin test demonstrates the intended trade-off: queue/extraction quality weights can favor clearance over cost, while cost-only weights recover the cheaper route.

The table records one local test run; the test executable prints fresh values for every run. `explored` counts queue-processing expansions that changed a state's D* Lite value. Times use `std::chrono::steady_clock` and are in milliseconds. `peak_memory_kb` is the process peak resident-set size on Linux in kB (and is `0` where this measurement is unavailable). The performance values depend on the machine and build configuration.
