User Manual

PCCST503 – Safe Semantic Planner

1. Requirements

The project requires:

- Python 3.x
- Standard Python libraries

No external Python packages are required.

---

2. Project Structure

The repository contains:

planner.py
testcases.py
test.py
experimental_results.csv
design_report.md
user_manual.md
README.md
.gitignore

---

3. Running the Program

Open a terminal in the project directory.

Run:

python test.py

The program executes all six assignment scenarios.

---

4. Test Results

The program displays:

- Test name
- Success/failure
- State path
- Transition path
- Total cost
- Minimum safety distance
- Cumulative reliability
- Number of explored states
- Planning time

The experimental results are also saved to:

experimental_results.csv

---

5. Running the Test Scenarios Directly

The test scenarios can also be executed using:

python testcases.py

---

6. Understanding the Output

Success

"True" means that the planner found a valid path from the initial state to the goal.

State Path

Example:

0 -> 1 -> 3

This means the planner moves from state 0 to state 1 and then to state 3.

Transition Path

Example:

0 -> 1

These are the IDs of the transitions used by the planner.

Total Cost

The total original transition cost of the selected path.

Minimum Safety Distance

The smallest Euclidean distance between any visited state and the nearest bad state.

Cumulative Reliability

The product of the reliability values of the transitions used.

Explored States

The number of states processed during D* Lite computation.

Planning Time

The execution time required to compute the plan, measured in milliseconds.

---

7. Dynamic Replanning

The planner supports changes during execution.

Disable a transition

planner.update_transition(
    transition_id=1,
    available=False
)

Change transition cost

planner.update_transition(
    transition_id=1,
    cost=5.0
)

Add a new transition

planner.add_transition(
    new_transition
)

Change the goal

planner.update_goal(new_goal)

After a dynamic change, call:

planner.plan()

to obtain the updated path.

---

8. Troubleshooting

If Python cannot find "planner.py", make sure all Python files are in the same directory.

If the command "python" does not work, try:

python3 test.py

---

9. Conclusion

The program provides a Python implementation of a D* Lite based safe semantic planner and demonstrates planning under both static and dynamic conditions.
