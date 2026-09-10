# User Manual
## Safe Semantic Planner Using D* Lite

### 1. Project Description

This project implements a Safe Semantic Planner for a finite Cartesian state space using the D* Lite path planning algorithm.

The planner finds a path from an initial state to a goal state while avoiding predefined bad states.

Each state contains a Cartesian coordinate representation. Each transition contains:

- Transition ID
- Source state
- Destination state
- Cost
- Safety score
- Reliability
- Availability

---

### 2. Software Requirements

The following software is required:

- Windows 10 or Windows 11
- Visual Studio Code
- GNU G++ compiler
- C++17 compatible compiler

---

### 3. Project Structure

The project contains the following files:

```text
safe-semantic-planner/
│
├── src/
│   └── main.cpp
│
├── results/
│   └── results.csv
│
├── report/
│   └── Design_Report.pdf
│
├── README.md
│
└── User_Manual.md
File Description

main.cpp

Contains the complete implementation of the planner and D* Lite algorithm.

results.csv

Contains the experimental results obtained from the different test cases.

Design_Report.pdf

Contains the detailed design, algorithm explanation, complexity analysis, test cases and experimental discussion.

README.md

Provides a short overview of the project and instructions for compilation and execution.

User_Manual.md

Provides detailed instructions for using the project.

4. Opening the Project

Open Visual Studio Code.

Select:

File → Open Folder

Select the:

safe-semantic-planner

folder.

The project files will appear in the Explorer panel.

5. Opening the Terminal

In Visual Studio Code, open:

Terminal → New Terminal

The terminal should open at the project directory.

For example:

PS C:\Users\ASUS\safe-semantic-planner>
6. Compilation

Compile the program using the following command:

g++ -std=c++17 src/main.cpp -o planner

If compilation is successful, a file named:

planner.exe

will be created.

7. Running the Program

Execute the program using:

.\planner.exe

The program automatically executes all six test cases.

8. Test Cases

The implementation contains the following test cases.

Test Case 1: Basic Reachability

Tests whether the planner can find a valid path from the initial state to the goal state.

Expected result:

A valid path from S to G is found.

Test Case 2: Bad State Avoidance

Tests whether the planner avoids states marked as bad.

The dangerous state X is marked as a bad state.

Expected result:

The planner must not visit X.

Test Case 3: Safety Margin

Tests the behavior of the planner when multiple valid paths exist with different costs and safety characteristics.

The planner evaluates the available paths while maintaining the hard constraint that bad states cannot be visited.

Test Case 4: Dynamic Transition

Tests the planner after a transition becomes unavailable.

Initially, the planner finds a valid route.

Then the transition A → B is disabled.

Expected result:

The planner computes another valid route.

Test Case 5: Goal Update

Tests the planner when the goal state changes.

Expected result:

The planner generates a path toward the updated goal.

Test Case 6: Transition Addition

Tests the planner when a new shortcut transition is added.

Expected result:

The planner can discover and use the newly available transition when it provides a better valid path.

9. Output

For each test case, the program displays:

Planned state path
Success or failure
Total path cost
Minimum safety distance
Cumulative reliability
Number of explored states
Planning time

Example:

Path: S -> C -> G
Success: YES
Total Cost: 11.000
Minimum Safety Distance: ...
Cumulative Reliability: ...
Explored States: ...
Planning Time: ... ms
10. Safety Calculation

The safety of a path is evaluated using the minimum Euclidean distance between the visited states and the bad states.

For every visited state, the distance to the nearest bad state is calculated.

The minimum of these distances is reported as the path's minimum safety distance.

A bad state is never intentionally included in a valid path.

11. Dynamic Environment

The planner considers changes in the environment such as:

Transition becoming unavailable
Goal state changing
Addition of a new transition

After an environment change, the planner generates a revised path.

This demonstrates the suitability of D* Lite for environments where the planning problem can change.

12. Experimental Results

After running the program, the experimental results are stored in:

results/results.csv

The CSV file contains measurements for:

Goal success
Number of bad states visited
Total path cost
Minimum safety distance
Cumulative reliability
Number of explored states
Planning time

The CSV file can be opened using Microsoft Excel or any spreadsheet application.

13. Troubleshooting
Problem: g++ is not recognized

If the terminal displays:

'g++' is not recognized

the GNU C++ compiler is not correctly installed or its location has not been added to the system PATH.

Install a GNU C++ compiler and restart Visual Studio Code.

Problem: Compilation errors

Make sure that:

src/main.cpp exists.
The terminal is opened in the project directory.
The compiler supports C++17.

Then run:

g++ -std=c++17 src/main.cpp -o planner
Problem: planner.exe does not run

Make sure compilation completed successfully.

Then run:

.\planner.exe
14. Conclusion

The Safe Semantic Planner provides a graph-based planning solution using D* Lite concepts.

It considers transition costs, state safety, reliability and dynamic changes while ensuring that predefined bad states are avoided.

The program provides both planning results and experimental measurements for evaluating the performance of the planner.


---

# 6.5 Save it

Press:

```text
Ctrl + S

That's it.

You don't need to run this file.