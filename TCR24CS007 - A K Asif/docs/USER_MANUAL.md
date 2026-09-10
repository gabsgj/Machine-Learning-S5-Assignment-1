# User Manual

## 1. Requirements

Install:

- Git
- MinGW/G++ or another C++17 compiler
- VS Code (recommended)

## 2. Open the project

Open the project folder in VS Code.

## 3. Compile

Open the VS Code terminal and run:

```cmd
g++ -std=c++17 src/main.cpp src/Planner.cpp -o planner.exe
```

## 4. Run

```cmd
planner.exe
```

## 5. Results

The terminal displays the six test cases.

A CSV file is generated at:

```text
results/results.csv
```

## 6. Modify a test

Test data can be changed in:

```text
src/main.cpp
```

The states, transitions, bad states and goal are defined there.
