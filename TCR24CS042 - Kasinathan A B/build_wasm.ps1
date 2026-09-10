# ============================================================
# build_wasm.ps1 — Build C++ core into WebAssembly with Emscripten
# ============================================================

$PSScriptRootPath = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location $PSScriptRootPath

$env:EMSDK = "$PSScriptRootPath\emsdk"
$env:PATH = "$PSScriptRootPath\emsdk;$PSScriptRootPath\emsdk\upstream\emscripten;$PSScriptRootPath\emsdk\upstream\bin;$PSScriptRootPath\emsdk\node\24.19.0_64bit\bin;" + $env:PATH

$emscriptenPy = "$PSScriptRootPath\emsdk\upstream\emscripten\em++.py"

Write-Host "Compiling C++ Safe Semantic Planner to WebAssembly..."

New-Item -ItemType Directory -Path web/src/wasm/bin -Force 2>$null

python $emscriptenPy --bind -sMODULARIZE=1 -sEXPORT_ES6=1 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web -O2 -Iinclude src/graph.cpp src/safety.cpp src/heuristic.cpp src/lpa_star.cpp src/bindings.cpp -o web/src/wasm/bin/planner.js

Copy-Item web/src/wasm/bin/planner.wasm web/public/planner.wasm -Force 2>$null

if ($LASTEXITCODE -eq 0) {
    Write-Host "WASM compilation succeeded!"
    Write-Host "Generated: web/src/wasm/bin/planner.js and web/public/planner.wasm"
} else {
    Write-Host "WASM compilation failed with code $LASTEXITCODE"
}
