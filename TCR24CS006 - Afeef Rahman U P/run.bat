@echo off
if not exist bin\planner.exe (
    echo Binaries not found, compiling first...
    call build.bat
)

bin\planner.exe %*
