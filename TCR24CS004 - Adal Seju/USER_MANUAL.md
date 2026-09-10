# User Manual: Safe Semantic Planner

**PCCST503 — Machine Learning: Assignment 1**  
**Safe Semantic Planner in a Finite Cartesian State Space**

---

## 1. System Requirements & Prerequisites

- **C++ Compiler:** MSYS2 GCC / MinGW-w64 (g++ $\ge 9.0$), Clang ($\ge 10.0$), or MSVC ($\ge 2019$) with C++17 support.
- **Web Browser (for Frontend GUI):** Google Chrome, Firefox, Safari, or Microsoft Edge (any modern browser supporting HTML5 Canvas).
- **Optional:** Python 3.x (to run local web server `python -m http.server 8080`).

---

## 2. Compiling and Running the C++ Core Engine

### 2.1 One-Click Windows Build
From the project root directory in PowerShell or Command Prompt:

```cmd
build.bat
run.bat
```

### 2.2 Manual Compilation
To compile directly with `g++` (zero external dependencies required):

```bash
# Compile with maximum optimization and all standard warnings enabled
g++ -std=c++17 -O3 -Wall -Wextra -Iinclude src/main.cpp -o bin/planner.exe

# Run default test suite + scalability benchmark
./bin/planner.exe
```

### 2.3 Command-Line Options

```bash
# Run all 6 assignment test cases + bonus knowledge graph
bin/planner.exe --test

# Run scalability benchmarks across graphs with 50 to 2000 states
bin/planner.exe --benchmark

# Display CLI help menu
bin/planner.exe --help
```

---

## 3. Launching and Operating the Web Visualizer

### 3.1 Launching the GUI
You can open `frontend/index.html` directly in any web browser, or launch a local web server:

```bash
python -m http.server 8080 --directory frontend
# Open your browser and navigate to: http://localhost:8080
```

### 3.2 Visualizer Interface Layout

```
+---------------------------------------------------------------------------------------+
|  SAFE SEMANTIC PLANNER                                   Algorithm: LPA* Incremental  |
+----------------------+------------------------------------+---------------------------+
| [ Left Control Panel ]| [ Center Interactive Canvas ]      | [ Right Telemetry Panel ] |
|                      |                                    |                           |
| - Scenario Selector  | - 2D Cartesian Coordinate Space    | - Optimal Path Sequence   |
|   (TC 1 to 6, Drone) | - Nodes (Start/Goal/Bad/Normal)    | - Total Cost (C)          |
| - Cold Plan / Replan | - Directed Transition Arrows       | - Safety Margin (D)       |
| - Step-by-Step / Play| - Glowing Red Hazard Fields        | - Cumulative Rel. (R)     |
| - Canvas Edit Tools  | - Pulsing Neon Optimal Path        | - Objective Score         |
| - Objective Sliders  | - Zoom (Scroll) / Pan (Shift-Drag) | - LPA* Queue Inspector   |
|   (alpha, beta, etc) | - Node Dragging                    | - Export / Import JSON    |
+----------------------+------------------------------------+---------------------------+
```

### 3.3 Interactive Canvas Tools

| Tool | Icon / Label | How to Use |
|---|---|---|
| **Drag / Move** | ✋ `Drag / Move` | Click and drag any state node to update its Cartesian coordinates in real time. LPA* automatically updates heuristics and replans. |
| **Add State** | ➕ `Add State` | Click anywhere on the canvas to instantiate a new state $s_{new} \in \mathbb{R}^2$. |
| **Add Edge** | ➔ `Add Edge` | Click the source state $s_u$, then click the target state $s_v$ to create a directed transition $(s_u, s_v)$. |
| **Toggle Bad State**| ⚠️ `Toggle Bad` | Click any state to toggle it between a safe state and a red hazard obstacle. |
| **Toggle Edge** | ⚡ `Toggle Edge` | Click any transition edge to disable (`available = false`) or enable it, demonstrating instantaneous LPA* replanning. |
| **Set Start / Goal** | 🟢 / 🟡 | Click any state to designate it as the new start $s_I$ or new goal $s_G$. |

---

## 4. Multi-Objective Weight Configuration

The planner optimizes the objective function:
$$\text{Score}(\mathcal{P}) = \alpha G - \beta C(\mathcal{P}) + \gamma D(\mathcal{P}) + \delta R(\mathcal{P})$$

Adjust the sliders in the left sidebar to observe real-time trade-off shifts:
- **$\alpha$ (Goal Weight):** Bonus awarded for successfully terminating at the goal.
- **$\beta$ (Cost Weight):** Penalty multiplier on cumulative edge traversal costs.
- **$\gamma$ (Safety Weight):** Reward multiplier for maintaining large clearance from bad states.
- **Safety Buffer Distance:** Threshold radius below which states incur a repulsive proximity penalty.
- **$\delta$ (Reliability Weight):** Multiplier rewarding high operational reliability edges.

---

## 5. Problem JSON Schema (Custom Graph Definitions)

Custom planning scenarios can be imported and exported via JSON:

```json
{
  "testName": "My Custom Scenario",
  "description": "Scenario description text.",
  "initialState": 1,
  "goalState": 4,
  "badStates": [3],
  "states": [
    { "id": 1, "embedding": [100.0, 300.0], "label": "Start" },
    { "id": 2, "embedding": [300.0, 200.0], "label": "Hub_A" },
    { "id": 3, "embedding": [450.0, 200.0], "label": "Hazard_Obstacle" },
    { "id": 4, "embedding": [700.0, 300.0], "label": "Goal" }
  ],
  "transitions": [
    { "id": 1, "from": 1, "to": 2, "cost": 1.2, "safety": 1.0, "reliability": 0.99, "available": true },
    { "id": 2, "from": 2, "to": 3, "cost": 0.8, "safety": 0.0, "reliability": 0.50, "available": true },
    { "id": 3, "from": 2, "to": 4, "cost": 2.5, "safety": 1.0, "reliability": 0.98, "available": true }
  ]
}
```

---

## 6. Troubleshooting & FAQ

**Q1: Compilation error `g++ not recognized`**  
*Fix:* Ensure MinGW-w64 or MSYS2 `bin` directory (e.g. `C:\msys64\ucrt64\bin`) is in your Windows system `PATH`.

**Q2: Web visualizer canvas is blank when opened directly**  
*Fix:* Ensure all files (`index.html`, `style.css`, `app.js`, `engine.js`, `scenarios.js`) remain together inside the `frontend/` directory.

**Q3: How do I zoom and pan on the canvas?**  
*Fix:* Use mouse scroll wheel to zoom in/out. Middle-click and drag (or Shift + Left-Click and drag) to pan around the graph.
