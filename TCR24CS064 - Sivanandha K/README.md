# PCCST503 – Machine Learning

## Assignment 1: Design of a Safe Semantic Planner in a Finite Cartesian State Space

### Student Details

Name: **SIVANANDHA K**  
Register Number: **TCR24CS064**  
Course: Machine Learning

---

## 📌 Project Description

This project implements a safe semantic planner in a finite Cartesian state space using the **D* Lite Algorithm**.

The planner finds safe and reliable paths between an initial state and a goal state while avoiding predefined bad states.

The program considers:

* Transition cost
* State safety
* Transition reliability
* Transition availability
* Dynamic changes in the environment

The D* Lite algorithm enables efficient incremental replanning when transitions or the goal state change.

---

## 📥 Inputs

The planner accepts:

1. Initial state
2. Goal state
3. Cartesian state representation
4. Directed transitions
5. Transition costs
6. Safety information
7. Reliability values
8. Transition availability
9. Bad states

---

## 📤 Outputs

The program produces:

* Planned path from initial state to goal
* Goal success status
* Bad states visited
* Path cost
* Minimum safety distance
* Number of explored states
* Planning time
* Memory usage
* Replanning time
* Results for different test cases

---

## 🛠️ Technologies Used

* C++17
* STL
* CMake
* D* Lite Algorithm
* Euclidean distance
* Priority Queue
* Graph-based state representation

---

## ⚙️ How to Run the Project

### Step 1: Install a C++ Compiler

Install a C++17-compatible compiler such as GCC.

Make sure the compiler is added to PATH.

---

### Step 2: Compile Using g++

Open a terminal in the project folder and run:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic src/main.cpp -o planner
```

---

### Step 3: Run the Application

Run:

```bash
./planner
```

---

### Step 4: Compile Using CMake

Create a build directory:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Run the generated executable:

```bash
./safe_planner
```

---

## 📊 Visualization / Results

The application evaluates the planner using different test cases.

The results include:

* Successful or unsuccessful goal reachability
* Path cost
* Safety distance from bad states
* Number of explored states
* Initial planning time
* Replanning time
* Memory usage
* Effect of transition and goal changes

Six illustrative test cases are included as part of the assignment.

---

## 📖 Algorithm Used

**D* Lite Algorithm**

D* Lite is an incremental path-planning algorithm designed for environments where the graph or path costs can change.

The implementation uses:

1. `g` values
2. `rhs` values
3. Priority queue
4. Predecessor states
5. Successor states
6. Incremental replanning

The effective transition cost is calculated using:

```text
effective_cost =
    transition_cost
    + safety_penalty
    + reliability_penalty
```

The safety penalty increases when a destination state is close to a bad state.

The reliability penalty increases when transition reliability decreases.

Bad states are treated as hard constraints and are never selected or expanded during path planning.

The planner uses a scaled Euclidean distance as its heuristic.

When transition availability, transitions, or the goal changes, D* Lite reuses previously calculated information and performs incremental replanning instead of calculating the complete path from scratch.

---

## 📁 Project Structure

```text
PCCST503_Assignment1/
│
├── Testcases/
│   ├── Test_Case_1,2.png
│   ├── Test_Case_3.png
│   ├── Test_Case_4.png
│   ├── Test_Case_5.png
│   ├── Test_Case_6.png
│   ├── experimental_results.txt
│   └── test_cases.md
│
├── LICENSE
├── README.md
├── design_report.md
├── main.cpp
└── user_manual.md

---

## 🔓 Repository Status

This repository contains the complete implementation, design report, experimental results, test cases, and user manual required for the assignment.

The repository is public as required by the assignment instructions.
