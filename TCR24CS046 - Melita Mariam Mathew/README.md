# Safe Semantic Planner

**Course:** PCCST503 – Machine Learning (Assignment 1)
**Student:** Melita Mariam Mathew
**Registration #:** TCR24CS046
**Roll #:** 45

---

## Overview

The *Safe Semantic Planner* is a C++ implementation of incremental path‑planning algorithms (LPA* and D* Lite) for a finite Cartesian state space. It provides:
- Core planning library (under `include/` and `src/`).
- A web‑based demonstration UI (`demo/`) that visualises the planner in real‑time.
- Design report, experimental results and a user manual (to be filled later).

---

## Prerequisites

| Tool | Minimum version | Install command (Windows) |
|------|------------------|---------------------------|
| **CMake** | 3.16 | `winget install Kitware.CMake` |
| **C++ compiler** | GCC/Clang (MinGW) **or** MSVC | `winget install llvm` *(for clang/GCC)* or install Visual Studio 2022 with the "Desktop development with C++" workload |
| **Node.js** (optional, for serving the demo) | 14.x | `winget install OpenJS.NodeJS` |
| **Python** (optional, for a quick HTTP server) | 3.8+ | `winget install Python.Python` |

Make sure the chosen compiler is on your `PATH` (`g++ --version` or `cl` should work).

---

## Building the C++ Core

```powershell
# From the project root
cd "C:\\Users\\LENOVO\\OneDrive\\Desktop\\Semantic Planner"
mkdir build
cd build
# Choose your generator:
# MSVC (Visual Studio 2022)
cmake -G "Visual Studio 17 2022" -A x64 ..
# OR MinGW (GCC/Clang)
# cmake -G "MinGW Makefiles" ..

# Build (Release configuration)
cmake --build . --config Release
```

The resulting executable will be located at:
- `build\\Release\\SafeSemanticPlanner.exe` (MSVC) **or**
- `build\\SafeSemanticPlanner.exe` (MinGW).

---

## Running the Planner (CLI)

```powershell
# Show the help screen for required arguments
.\\SafeSemanticPlanner.exe --help
```

Typical usage (replace placeholders with your own data):

```powershell
.\\SafeSemanticPlanner.exe ^
    --graph data\\graph.txt ^
    --start 0 ^
    --goal 42 ^
    --output results.json
```

If the binary starts an internal HTTP server you will see a line such as:

```
Listening on http://127.0.0.1:8080
```

You can then open that URL in a browser.

---

## Serving the Web Demo

The UI lives in the `demo/` folder (HTML, CSS, JavaScript). You can serve it with a lightweight server:

### Using Python (built‑in)
```powershell
cd "C:\\Users\\LENOVO\\OneDrive\\Desktop\\Semantic Planner\\demo"
python -m http.server 8080
```

### Using Node.js (`serve`)
```powershell
npm install -g serve   # one‑time install
cd "C:\\Users\\LENOVO\\OneDrive\\Desktop\\Semantic Planner\\demo"
serve -l 8080
```

Open a browser and navigate to `http://localhost:8080/index.html`.

---

## License

The code is released for educational purposes under the MIT License. Feel free to adapt, share, and experiment.
