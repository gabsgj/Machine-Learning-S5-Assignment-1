# Experimental Results

## 1. Experimental Setup

The Safe Semantic Planner was compiled and executed using a C++17 compiler with optimization enabled.

Compilation command:

```bash
g++ -std=c++17 -O3 safe_planner.cpp -o safe_planner
```

The program was executed on a Windows PowerShell environment.

The planner was evaluated using six test cases covering reachability, bad-state avoidance, safety margin, dynamic transition changes, goal updates, and transition additions.

---

## 2. Test Case 1 – Basic Reachability

The first test case verifies whether the planner can find a path through a simple directed graph.

The obtained state path was:

```text
0 -> 1 -> 2 -> 3
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    3.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.8574 |
| States Explored         |         4 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

The planner successfully reached the goal using the expected path.

---

## 3. Test Case 2 – Bad State Avoidance

This test verifies that the planner avoids a state marked as bad.

The obtained path was:

```text
0 -> 3 -> 4 -> 5
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    3.0000 |
| Minimum Safety Distance |    1.0000 |
| Cumulative Reliability  |    0.8574 |
| States Explored         |         5 |
| Planning Time           | 1.5870 ms |
| Bad States Visited      |         0 |

The planner successfully avoided the bad state. The number of bad states visited was zero.

---

## 4. Test Case 3 – Safety Margin Balance

This test evaluates the planner when multiple valid paths are available and safety must be considered.

The selected path was:

```text
0 -> 3 -> 4 -> 5
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    6.0000 |
| Minimum Safety Distance |    1.7000 |
| Cumulative Reliability  |    0.9412 |
| States Explored         |         5 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

The selected path maintains a minimum distance of 1.7 from the nearest bad state.

This demonstrates the effect of the safety-aware cost function.

---

## 5. Test Case 4 – Dynamic Transition

This test evaluates the planner when an existing transition becomes unavailable.

### Before the transition change

The planner produced:

```text
0 -> 1 -> 4
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    2.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.9025 |
| States Explored         |         3 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

### After `A -> G` becomes unavailable

The planner produced the alternative path:

```text
0 -> 3 -> 4
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    4.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.9025 |
| States Explored         |         3 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

The planner successfully adapted to the unavailable transition and selected an alternative route.

---

## 6. Test Case 5 – Dynamic Goal Relocation

This test evaluates the planner when the goal state changes.

### Original goal

The original goal was State 3.

The planner produced:

```text
0 -> 1 -> 3
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    2.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.9025 |
| States Explored         |         3 |
| Planning Time           | 0.7510 ms |
| Bad States Visited      |         0 |

### Updated goal

The goal was changed to State 4.

The planner then produced:

```text
0 -> 2 -> 4
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    2.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.9025 |
| States Explored         |         3 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

The planner successfully generated a new path to the updated goal.

---

## 7. Test Case 6 – Transition Addition

This test evaluates the planner after a new shortcut transition is added.

### Before shortcut insertion

The planner produced:

```text
0 -> 1 -> 4
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    6.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.8100 |
| States Explored         |         4 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

### After shortcut insertion

A new direct transition from State 0 to State 4 was added.

The planner produced:

```text
0 -> 4
```

Results:

| Metric                  |    Result |
| ----------------------- | --------: |
| Status                  |   Success |
| Total Cost              |    1.0000 |
| Minimum Safety Distance |    0.0000 |
| Cumulative Reliability  |    0.9900 |
| States Explored         |         2 |
| Planning Time           | 0.0000 ms |
| Bad States Visited      |         0 |

The newly added shortcut reduced the total path cost from 6.0 to 1.0.

---

## 8. Overall Results

| Test Case   | Status  | Path          | Cost | Min Safety Distance | Reliability | Explored States | Bad States |
| ----------- | ------- | ------------- | ---: | ------------------: | ----------: | --------------: | ---------: |
| TC1         | Success | 0 → 1 → 2 → 3 |  3.0 |                 0.0 |      0.8574 |               4 |          0 |
| TC2         | Success | 0 → 3 → 4 → 5 |  3.0 |                 1.0 |      0.8574 |               5 |          0 |
| TC3         | Success | 0 → 3 → 4 → 5 |  6.0 |                 1.7 |      0.9412 |               5 |          0 |
| TC4         | Success | 0 → 1 → 4     |  2.0 |                 0.0 |      0.9025 |               3 |          0 |
| TC4 Updated | Success | 0 → 3 → 4     |  4.0 |                 0.0 |      0.9025 |               3 |          0 |
| TC5         | Success | 0 → 1 → 3     |  2.0 |                 0.0 |      0.9025 |               3 |          0 |
| TC5 Updated | Success | 0 → 2 → 4     |  2.0 |                 0.0 |      0.9025 |               3 |          0 |
| TC6         | Success | 0 → 1 → 4     |  6.0 |                 0.0 |      0.8100 |               4 |          0 |
| TC6 Updated | Success | 0 → 4         |  1.0 |                 0.0 |      0.9900 |               2 |          0 |

---

## 9. Goal Success Rate

All six test cases successfully produced a valid path.

Therefore:

$$
Goal\ Success\ Rate =
\frac{6}{6}\times100
$$

$$
\boxed{100\%}
$$

---

## 10. Bad-State Avoidance

No bad state was visited in any successful test case.

Therefore:

$$
\boxed{0}
$$

bad states were visited.

This satisfies the safety constraint of the assignment.

---

## 11. Dynamic Planning Observations

The dynamic experiments demonstrate that the planner can respond to changes in the environment.

In Test Case 4, making the original transition unavailable caused the planner to select an alternative route.

In Test Case 5, changing the goal caused the planner to produce a path to the new goal.

In Test Case 6, adding a direct shortcut reduced the path cost from 6.0 to 1.0.

These experiments demonstrate the planner's ability to adapt to changes in the planning problem.

---

## 12. Safety Observation

Test Case 3 demonstrates the safety-margin objective.

The selected path had:

$$
D(P)=1.7
$$

meaning that the closest visited state was 1.7 units away from the nearest bad state.

The result shows that safety can influence path selection in addition to ordinary transition cost.

---

## 13. Conclusion

The experimental evaluation shows that the planner successfully solved all six test cases.

The goal success rate was 100%, and no bad states were visited.

The planner successfully demonstrated:

* Basic path finding.
* Bad-state avoidance.
* Safety-aware planning.
* Dynamic transition handling.
* Goal relocation.
* Transition addition.

The experiments also showed that environmental changes can cause the selected path and path cost to change.

The measured execution times for the small test graphs are frequently reported as 0.0000 ms because the computations are extremely fast relative to the timer resolution. Therefore, these values should not be interpreted as zero computational effort.

For larger graphs, more meaningful planning and replanning times can be obtained.
