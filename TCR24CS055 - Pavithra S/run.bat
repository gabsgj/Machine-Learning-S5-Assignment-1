@echo off
echo =======================================================
echo   Running Safe Semantic Planner Test Suite & Benchmarks
echo =======================================================

call build.bat
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo.
echo Executing Planner Test Suite...
.\safe_planner.exe

echo.
echo Launching Interactive Visualizer...
start "" "%~dp0visualizer\index.html"
