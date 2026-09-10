# Orchestration Algorithm Workbench - LPA* Pathfinding Visualizer

An interactive Web Application & C++ Core Engine for hazard-aware state-space pathfinding using **Lifelong Planning A* (LPA*)** based on [`cpp/orchestration.c++`](cpp/orchestration.c++).

---

## 🌐 Live Web Demo

🌐 **Live Deployed App**: [https://orchestration-algorithm-4ot1.vercel.app/](https://orchestration-algorithm-4ot1.vercel.app/)

---

## 🚀 Quick Start Guide

### 1. Web Application Visualizer
No installation or build steps required!
- **Live Online Version**: Access [https://orchestration-algorithm-4ot1.vercel.app/](https://orchestration-algorithm-4ot1.vercel.app/) directly.
- **Direct Launch**: Open [`web/index.html`](web/index.html) in any web browser.
- **Local HTTP Server** (Recommended):
  ```bash
  python -m http.server 8080
  # Open http://localhost:8080/web/index.html
  ```

### 2. Native C++ Engine
Compile and run using any C++17 compliant compiler:
```bash
cd cpp
g++ -O3 orchestration.c++ -o orchestration
./orchestration
```

---

## 📁 Repository Structure

```
Orchestration-Algorithm/
├── cpp/                          # Native C++ Core Source Files & Experiments
│   ├── orchestration.c++         # C++ Core Library (LPA* Pathfinding)
│   └── run_experiments.js        # Node.js Benchmark Experiment Runner
├── web/                          # Web Application Visualizer Files
│   ├── index.html                # Main Visualizer HTML Layout
│   ├── styles.css                # Glassmorphism Styling & Design Tokens
│   └── app.js                    # Web Canvas & LPA* JavaScript Pathfinder
├── docs/                         # Project Documentation Reports
│   ├── DESIGN_REPORT.md          # Technical Design Report & Math Formulation
│   ├── EXPERIMENTAL_RESULTS.md   # Benchmark Data & Scalability Analysis
│   ├── USER_MANUAL.md            # Universal Operating & Setup Manual
│   └── DEMONSTRATION.md          # Step-by-Step Test Demonstration & Visual Walkthrough
├── DESIGN.md                     # Top-level Design alias
└── README.md                     # Top-level Repository Overview & Quick Start
```

---

## 📚 Documentation Reports

- 📖 **[User Manual (`docs/USER_MANUAL.md`)](docs/USER_MANUAL.md)**: Operating instructions, shortcuts, and troubleshooting.
- 📐 **[Design Report (`docs/DESIGN_REPORT.md`)](docs/DESIGN_REPORT.md)**: System architecture, algorithm formulation, and data structures.
- 📊 **[Experimental Results (`docs/EXPERIMENTAL_RESULTS.md`)](docs/EXPERIMENTAL_RESULTS.md)**: Scaling benchmarks and parameter sweep data.
- 🎬 **[Demonstration (`docs/DEMONSTRATION.md`)](docs/DEMONSTRATION.md)**: Step-by-step test walkthrough and visual evidence.
