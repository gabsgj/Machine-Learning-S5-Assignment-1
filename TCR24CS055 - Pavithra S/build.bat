@echo off
echo =======================================================
echo   Building Safe Semantic Planner (C++14 / MinGW GCC)
echo =======================================================

if not exist visualizer mkdir visualizer

g++ -std=c++14 -O3 -Wall -Iinclude src\main.cpp -o safe_planner.exe

if %ERRORLEVEL% EQU 0 (
    echo [SUCCESS] Build succeeded: safe_planner.exe created.
) else (
    echo [ERROR] Build failed.
)
