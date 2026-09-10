# Building on Windows

Two easy options. Either works with the same source files — nothing in the
code is Linux-specific except one small `#ifdef`-guarded block in
`experiment.cpp` (already handled).

## Option A: MSYS2 / MinGW-w64 (recommended, closest to how it was built here)

1. Install MSYS2 from https://www.msys2.org/
2. Open the "MSYS2 MinGW64" terminal and install the compiler:
   ```
   pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-make
   ```
3. `cd` into the `safe_planner` folder (e.g. `cd /c/Users/<you>/Downloads/safe_planner`)
4. Build:
   ```
   mingw32-make
   ```
   or without the Makefile:
   ```
   g++ -std=c++17 -O2 -Iinclude src/LPAStarPlanner.cpp src/main.cpp -o safe_planner_test.exe
   g++ -std=c++17 -O2 -Iinclude src/LPAStarPlanner.cpp src/experiment.cpp -o experiment.exe
   ```
5. Run: `./safe_planner_test.exe` and `./experiment.exe`

## Option B: Visual Studio (MSVC)

1. Install "Desktop development with C++" workload from Visual Studio Installer.
2. Open the "Developer Command Prompt for VS".
3. `cd` into `safe_planner`, then:
   ```
   cl /std:c++17 /O2 /I include src\LPAStarPlanner.cpp src\main.cpp /Fe:safe_planner_test.exe
   cl /std:c++17 /O2 /I include src\LPAStarPlanner.cpp src\experiment.cpp /Fe:experiment.exe
   ```
4. Run: `safe_planner_test.exe` and `experiment.exe`

## VS Code (either option)

Open the `safe_planner` folder in VS Code with the C/C++ extension installed,
select the MinGW or MSVC compiler as your kit, and use the built-in
"Run/Debug" on `src/main.cpp` — or just run the `g++`/`cl` commands above in
the integrated terminal.

## Redirecting experiment output to a CSV (for the report / Excel)

```
./experiment.exe > experiment_results.csv
```
This is already included as `experiment_results.csv` in this package, generated
from a Linux run — re-running it on your machine will reproduce the same
pattern (timings will differ slightly by CPU, but the incremental-vs-cold
speedup trend will hold).
