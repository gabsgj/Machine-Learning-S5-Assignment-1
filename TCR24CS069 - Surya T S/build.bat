@echo off
REM ═══════════════════════════════════════════════════════════════
REM  Build script for Safe Semantic Planner
REM  Uses g++ (MinGW) directly
REM ═══════════════════════════════════════════════════════════════

echo Building Safe Semantic Planner...

if not exist "build" mkdir build

echo [1/2] Compiling safe_planner...
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude ^
    src/main.cpp ^
    src/d_star_lite_planner.cpp ^
    src/safety_utils.cpp ^
    -o build/safe_planner.exe

if %ERRORLEVEL% NEQ 0 (
    echo BUILD FAILED: safe_planner
    exit /b 1
)

echo [2/2] Compiling test_planner...
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude ^
    tests/test_planner.cpp ^
    src/d_star_lite_planner.cpp ^
    src/safety_utils.cpp ^
    -o build/test_planner.exe

if %ERRORLEVEL% NEQ 0 (
    echo BUILD FAILED: test_planner
    exit /b 1
)

echo.
echo Build successful!
echo   build/safe_planner.exe  - Main test driver with all 6 test cases
echo   build/test_planner.exe  - Automated test suite
