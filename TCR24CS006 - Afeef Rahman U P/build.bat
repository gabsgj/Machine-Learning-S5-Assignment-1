@echo off
echo =======================================================
echo Building Safe Semantic Planner (C++17 / MSYS2 GCC)
echo =======================================================

if not exist bin mkdir bin

g++ -std=c++17 -O3 -Wall -Wextra -Iinclude src/main.cpp -o bin/planner.exe

if %ERRORLEVEL% EQU 0 (
    echo [SUCCESS] Build succeeded! Executable generated at bin/planner.exe
) else (
    echo [ERROR] Build failed!
)
