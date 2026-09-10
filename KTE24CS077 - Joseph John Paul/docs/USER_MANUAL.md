# Orchestration Algorithm Workbench - User Manual & Repository Guide

**Live Web Application:** [https://orchestration-algorithm-4ot1.vercel.app/](https://orchestration-algorithm-4ot1.vercel.app/)  
**Version:** 1.0.0  
**Target Systems:** Windows, macOS, Linux (Any OS with a modern Web Browser or C++ Compiler)  
**Core Project Files:** [`web/index.html`](../web/index.html), [`web/app.js`](../web/app.js), [`web/styles.css`](../web/styles.css), [`cpp/orchestration.c++`](../cpp/orchestration.c++)  

---

## 1. Quick Start Guide for New Repository Users

If you have just cloned or downloaded this repository, follow the simple steps below to run either the **Web Application Visualizer** or the **C++ Core Engine**.

### 1.1 Download / Clone Repository
```bash
# Clone the repository using Git
git clone https://github.com/Josephjohnpaul/Orchestration-Algorithm.git

# Navigate into the project folder
cd Orchestration-Algorithm
```

---

### 1.2 Running the Web Application (No Installation Required!)

The web visualizer runs in **any modern web browser** (Chrome, Edge, Firefox, Safari) without needing any npm packages or external builds!

#### Option A: Direct File Launch (Fastest)
Simply double-click `web/index.html` in your file explorer, or drag `web/index.html` into your web browser window!

#### Option B: Local Web Server (Recommended)
Starting a local HTTP server prevents browser security restrictions when loading assets or JSON files:

- **Using Python 3**:
  ```bash
  python -m http.server 8080
  # Or on macOS/Linux:
  python3 -m http.server 8080
  ```
  Then open [http://localhost:8080/web/index.html](http://localhost:8080/web/index.html) in your browser.

- **Using Node.js**:
  ```bash
  npx serve .
  # OR
  npx http-server .
  ```

- **Using VS Code**:
  Right-click `web/index.html` and choose **Open with Live Server**.

---

### 1.3 Compiling and Running the Native C++ Engine

To compile and run the native C++ Lifelong Planning A* (LPA*) implementation:

#### On Windows (PowerShell / Command Prompt):
```powershell
# Navigate to cpp folder
cd cpp

# Using GCC (MinGW / MSYS2)
g++ -O3 orchestration.c++ -o orchestration.exe ; .\orchestration.exe

# Using Clang
clang++ -O3 orchestration.c++ -o orchestration.exe ; .\orchestration.exe
```

#### On Linux / macOS (Terminal):
```bash
# Navigate to cpp folder
cd cpp

# Using GCC / Clang
g++ -O3 orchestration.c++ -o orchestration
./orchestration
```

**Expected Console Output**:
```text
Plan found successfully!
Path: 1 2 4 
Total Cost: 8
Min Safety Distance: 1
```

---

## 2. Interactive Canvas & UI Operating Guide

The web visualizer provides an interactive graphical interface for constructing, editing, and solving pathfinding problems.

```
+---------------------------------------------------------------------------------------+
|  [Orchestration Visualizer]              [Presets] [Save JSON] [Load JSON] [Export C++]|
+------------------------------------+--------------------------------------------------+
| PROBLEM CONFIGURATION              | INTERACTIVE GRAPH CANVAS                         |
|                                    |                                                  |
| Start State: [ State 1  v ]        |                 🟢 S1 (Start)                    |
| Goal State:  [ State 4  v ]        |                /             \                   |
|                                    |               / (Golden)     \                   |
| Bad States (Hazards):              |              v                v                  |
|  [ State 3  x ]                    |          🟡 S2               🔴 S3 (Hazard)      |
|  [ Add Hazard v ] [+ Add]          |              \                /                  |
|                                    |               \              /                   |
| Safety Weight (α):                 |                v            v                     |
|  (----o-----------------) 2.0      |                 🟣 S4 (Goal)                     |
|                                    |                                                  |
| GRAPH MANAGER                      |  Legend: 🟢 Start  🟣 Goal  🔴 Bad  🟡 Path      |
|  [States Table] [Transitions Table]|                                                  |
|  [+ Add Node]   [Reset Graph]      |                                                  |
+------------------------------------+--------------------------------------------------+
| PLANNER RESULTS DRAWER                                                                |
| Status: Optimal Plan Found  | Total Cost: 4.015  | Path Flow: S1 -> S2 -> S4         |
+---------------------------------------------------------------------------------------+
```

### Canvas Mouse Controls & Shortcuts

| Action | Control Shortcut | Description |
| :--- | :--- | :--- |
| **Move Node** | `Left Click + Drag` | Drag any state node to reposition its 2D embedding `[x, y]` coordinates. |
| **Pan Canvas** | `Left Click + Drag` on Background | Pan the viewport across the infinite grid. |
| **Zoom View** | `Mouse Scroll Wheel` | Zoom in or out centered at cursor location. |
| **Connect Edge** | `Shift + Click` Node A $\to$ Click Node B | Select Node A with `Shift + Click`, then click Node B to create a directed transition $A \to B$. |
| **Toggle Hazard** | `Right Click` Node | Instantly toggle a state's status as a **Bad State** (Hazard). |

---

## 3. Configuration & Parameter Controls

### 3.1 Problem Setup Panel
- **Initial State (Start)**: The starting state ID for the planner.
- **Goal State**: The destination target state ID.
- **Bad States (Hazards)**: Designated obstacle states. Paths are automatically rerouted around these hazard zones.
- **Safety Penalty Weight ($\alpha$)**: Adjust the slider from `0.0` to `10.0`. Higher values force the pathfinder to maintain wider clearance from Bad State hazard fields.

### 3.2 Graph Manager Tools
- **States Button**: Opens the **State Manager Modal** to edit numeric state IDs and embedding coordinates `[X, Y]`.
- **Transitions Button**: Opens the **Transition Manager Modal** to adjust edge costs, safety, reliability, and edge availability toggles (`true`/`false`).
- **Add Node**: Places a new state at the current canvas center.
- **Reset**: Clears all states and transitions to start building a custom problem.

---

## 4. Presets & Data Management

### 4.1 Preset Scenarios
Use the top **Presets** dropdown to explore pre-configured problems:
1. **C++ Test Verification Benchmark**: Canonical 4-state diamond test problem matching `orchestration.c++` `main()`.
2. **Autonomous Mobile Robot Grid**: Grid obstacle navigation scenario.
3. **Microservice Fault Resilient Mesh**: Cloud microservices network with compromised service failover.
4. **Urban Drone Air Corridor**: Aerial pathing avoiding radar no-fly zones.

### 4.2 Save & Load Configurations
- **Save JSON**: Export the current graph setup, embeddings, hazards, and parameters into a reusable `.json` file.
- **Load JSON**: Load any saved `.json` problem configuration.

### 4.3 C++ Code Generator
Click **Export C++ Code** in the top navigation bar to generate copy-pasteable C++ setup code matching the current visual layout for `orchestration.c++`.

---

## 5. Troubleshooting & FAQ

| Issue | Common Cause | Solution |
| :--- | :--- | :--- |
| **"Planner Status: No Path Found"** | Goal state is disconnected, or all connecting transitions are disabled/blocked by Bad States. | Check transition availability checkboxes or reduce Safety Penalty Weight $\alpha$. |
| **Canvas appears blank** | Browser window resized during initial render. | Click the **Reset View** (`[ [] ]`) button on the top-right canvas toolbar. |
| **C++ compilation error** | Missing C++17 compiler support. | Ensure your compiler supports C++17 or later (`g++ -std=c++17 orchestration.c++`). |
