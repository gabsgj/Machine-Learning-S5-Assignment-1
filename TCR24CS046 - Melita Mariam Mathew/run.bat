@echo off
setlocal enabledelayedexpansion

echo ======================================================================
echo  PCCST503: Safe Semantic Planner - Automated Build & Test Runner
echo ======================================================================

set GCC_PATH=C:\Users\LENOVO\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin
set PATH=%GCC_PATH%;%PATH%

if not exist data mkdir data

echo [1/2] Compiling C++ Sources (C++17, -O3)...
g++ -std=c++17 -O3 -Iinclude src\*.cpp -o SafeSemanticPlanner.exe

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed!
    exit /b %ERRORLEVEL%
)

echo [2/2] Running Safe Semantic Planner Executable...
echo ======================================================================
SafeSemanticPlanner.exe
echo ======================================================================
pause
