# Experimental Results

The program contains six required test cases.

## Metrics

| Test Case | Goal Reached | Bad States Visited | Path Cost | Safety Distance | Explored States |
|---|---|---:|---:|---:|---:|
| 1. Basic Reachability | Yes | 0 | Reported by program | N/A | Reported by program |
| 2. Bad State Avoidance | Yes | 0 | Reported by program | Reported by program | Reported by program |
| 3. Safety Margin | Yes | 0 | Reported by program | Reported by program | Reported by program |
| 4. Dynamic Transition | Yes | 0 | Reported by program | N/A | Reported by program |
| 5. Goal Update | Yes | 0 | Reported by program | N/A | Reported by program |
| 6. Transition Addition | Yes | 0 | Reported by program | N/A | Reported by program |

## Expected Observations

### Test Case 1

The only available route is selected:

```text
0 → 1 → 2 → 3
```

### Test Case 2

The route containing bad state `2` is rejected. The safe route is selected.

### Test Case 3

The planner considers the safety-adjusted transition value and reports the minimum distance between the selected route and the bad state.

### Test Case 4

The initial route uses the direct transition. After it becomes unavailable, the planner selects the alternative route.

### Test Case 5

Changing the goal causes a new route to be calculated.

### Test Case 6

After inserting a shortcut, the new transition becomes available to the planner and can improve the resulting path.

## Reproducibility

Compile and run:

```bash
g++ -std=c++17 -O2 src/main.cpp -o planner
./planner
```

On Windows MinGW:

```bash
g++ -std=c++17 -O2 src/main.cpp -o planner.exe
planner.exe
```
