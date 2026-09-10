# Experimental Results

## 1. Experimental Setup

The Safe Semantic Planner was implemented in C++ using the D* Lite path-planning algorithm. The experiments were conducted using the six demonstration scenarios defined for the assignment.

The planner was evaluated for:

* Goal reachability
* Avoidance of bad states
* Path cost
* Minimum safety distance from bad states
* Number of explored states
* Cumulative transition reliability
* Initial planning time
* Replanning time after dynamic changes

The six scenarios cover basic reachability, bad-state avoidance, safety margin handling, dynamic transition removal, goal updates, and transition addition. These correspond to the illustrative test cases specified in the assignment.

---

## 2. Experimental Results

The following results were obtained from an actual execution of the planner on a Linux environment.

| Test Case                     | Success | State Path    | Total Cost | Minimum Safety Distance | Cumulative Reliability | Explored States | Planning Time (ms) | Replanning Time (ms) |
| ----------------------------- | ------- | ------------- | ---------: | ----------------------: | ---------------------: | --------------: | -----------------: | -------------------: |
| 1. Basic Reachability         | True    | 0 → 1 → 2 → 5 |     3.0000 |                  0.0000 |                 0.8574 |               4 |             0.8149 |                  N/A |
| 2. Bad State Avoidance        | True    | 0 → 3 → 4 → 5 |     3.2000 |                  2.0000 |                 0.8574 |               6 |             0.1149 |                  N/A |
| 3. Safety Margin              | True    | 0 → 3 → 4 → 5 |     3.2000 |                  1.4142 |                 0.8574 |               6 |             0.0439 |                  N/A |
| 4. Dynamic Transition Removal | True    | 0 → 1 → 6 → 5 |     2.8000 |                  0.0000 |                 0.8574 |               6 |             0.0240 |               0.0240 |
| 5. Goal Update                | True    | 0 → 3 → 4     |     2.2000 |                  0.0000 |                 0.9025 |               3 |             0.0069 |               0.0069 |
| 6. Transition Addition        | True    | 0 → 5         |     0.5000 |                  0.0000 |                 0.9900 |               1 |             0.0020 |               0.0020 |

---

## 3. Test Case Analysis

### Test 1: Basic Reachability

The planner successfully reached the goal using the path:

`0 → 1 → 2 → 5`

The total path cost was **3.0000**, with **4 states explored**. The measured planning time was **0.8149 ms**.

This demonstrates that the planner can find a valid path from the initial state to the goal state.

---

### Test 2: Bad State Avoidance

The planner successfully avoided the bad state and selected the alternative path:

`0 → 3 → 4 → 5`

The total cost was **3.2000**, and the minimum safety distance was **2.0000**. The planner explored **6 states** and completed planning in **0.1149 ms**.

The result demonstrates that the planner can select a safe alternative when a direct route contains an undesirable state.

---

### Test 3: Safety Margin

The planner again selected:

`0 → 3 → 4 → 5`

The total cost was **3.2000**, while the minimum safety distance was **1.4142**. A total of **6 states** were explored, with a planning time of **0.0439 ms**.

This test demonstrates the effect of safety considerations when selecting a path near bad states.

---

### Test 4: Dynamic Transition Removal

After a transition was dynamically removed, the planner successfully generated the updated path:

`0 → 1 → 6 → 5`

The resulting path had a total cost of **2.8000** and a minimum safety distance of **0.0000**.

The initial planning time was **0.0240 ms**, and the reported replanning time was also **0.0240 ms**.

This demonstrates that the planner can adapt its route when transition availability changes.

---

### Test 5: Goal Update

After changing the goal, the planner successfully generated:

`0 → 3 → 4`

The total cost was **2.2000**, with a cumulative reliability of **0.9025**. The planner explored **3 states**.

The planning time and reported replanning time were both **0.0069 ms**.

This demonstrates that the planner can update its route when the destination changes.

---

### Test 6: Transition Addition

After a new transition was added, the planner selected the direct path:

`0 → 5`

The resulting path had a total cost of **0.5000** and cumulative reliability of **0.9900**.

Only **1 state** was explored. The planning and reported replanning times were both **0.0020 ms**.

This demonstrates that the planner can take advantage of newly available transitions.

---

## 4. Overall Observations

All six test cases completed successfully, giving a **100% observed success rate** across the demonstration scenarios.

The results show that:

1. The planner can successfully reach the required goals.
2. The planner can avoid undesirable states when an alternative route is available.
3. Safety distance is incorporated into route evaluation.
4. The planner responds to dynamic transition removal.
5. The planner responds to changes in the goal.
6. The planner can exploit newly added transitions.
7. Replanning is performed successfully for the tested dynamic scenarios.
8. The measured planning and replanning times were below 1 ms for all reported runs in this experiment.

The transition-addition case produced the shortest path and lowest total cost among the six demonstrations, with a cost of **0.5000** and a planning time of **0.0020 ms**.

The goal-update case required only **3 explored states**, while the basic reachability case explored **4 states**.

---

## 5. Safety and Reliability Observations

The experiments demonstrate that safety and reliability are considered as part of the planner's evaluation.

The minimum safety distance varied between the test cases:

* Test 1: **0.0000**
* Test 2: **2.0000**
* Test 3: **1.4142**
* Test 4: **0.0000**
* Test 5: **0.0000**
* Test 6: **0.0000**

The highest reported cumulative reliability was **0.9900** in the transition-addition scenario.

The bad-state avoidance scenario achieved a minimum safety distance of **2.0000**, while the safety-margin scenario achieved **1.4142**.

---

## 6. Performance

The measured planning times were:

| Test Case                  | Planning Time (ms) |
| -------------------------- | -----------------: |
| Basic Reachability         |             0.8149 |
| Bad State Avoidance        |             0.1149 |
| Safety Margin              |             0.0439 |
| Dynamic Transition Removal |             0.0240 |
| Goal Update                |             0.0069 |
| Transition Addition        |             0.0020 |

The minimum measured planning time was **0.0020 ms**, while the maximum was **0.8149 ms** in the basic reachability test.

The reported replanning times for the three dynamic scenarios were:

| Dynamic Test               | Replanning Time (ms) |
| -------------------------- | -------------------: |
| Dynamic Transition Removal |               0.0240 |
| Goal Update                |               0.0069 |
| Transition Addition        |               0.0020 |

These measurements indicate that the tested replanning operations completed quickly for the small demonstration graph.

---

## 7. Limitations

The experiments were conducted on the small demonstration graph supplied with the project. Therefore, the measured execution times should not be interpreted as performance benchmarks for very large state spaces.

The current experiment output does not include a direct operating-system-level measurement of memory usage. Therefore, no memory-usage value is reported here rather than inventing a measurement.

Further experiments with larger graphs, more states, more transitions, and repeated runs would provide a stronger evaluation of scalability and memory consumption.

---

## 8. Conclusion

The experimental results demonstrate that the Safe Semantic Planner successfully handled all six required demonstration scenarios.

The planner was able to find valid routes, avoid bad states, incorporate safety and reliability considerations, and adapt to dynamic changes such as transition removal, goal updates, and transition addition.

All six tested scenarios returned `success: true`, demonstrating successful operation of the implemented planner for the assignment's demonstration cases.
