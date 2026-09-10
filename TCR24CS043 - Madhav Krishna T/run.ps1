# PCCST503 Safe Semantic Planner - One-Shot Run Script
# Usage:
#   .\run.ps1              (run tests, then open dashboard)
#   .\run.ps1 -TestOnly    (run tests only)
#   .\run.ps1 -DashOnly    (skip tests, open dashboard immediately)
#   .\run.ps1 -Port 9000   (use a different port)

param(
    [switch]$TestOnly,
    [switch]$DashOnly,
    [int]$Port = 8080
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

function Print-Header($msg) {
    Write-Host ""
    Write-Host "=== $msg ===" -ForegroundColor Cyan
}

function Print-Pass($msg) {
    Write-Host "  [PASS] $msg" -ForegroundColor Green
}

function Print-Fail($msg) {
    Write-Host "  [FAIL] $msg" -ForegroundColor Red
}

# ── 1. C++ TEST SUITE ────────────────────────────────────────────────────────
if (-not $DashOnly) {
    Print-Header "C++ Test Suite"

    $tests = @(
        @{ exe = "bin\test_stage1.exe";          label = "Stage 1 - Core Model and ProblemLoader" },
        @{ exe = "bin\test_dstar_lite.exe";       label = "TC1-TC6 - D* Lite Integration" },
        @{ exe = "bin\test_metrics.exe";          label = "Metrics - Timing and Instrumentation" },
        @{ exe = "bin\test_wasm_api_native.exe";  label = "WASM C API - JSON Boundary Smoke Test" }
    )

    $allPassed = $true
    foreach ($t in $tests) {
        $exePath = Join-Path $root $t.exe
        if (-not (Test-Path $exePath)) {
            Print-Fail "$($t.label) - binary missing: $($t.exe)"
            $allPassed = $false
            continue
        }
        & $exePath 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Print-Pass $t.label
        } else {
            Print-Fail $t.label
            & $exePath
            $allPassed = $false
        }
    }

    if (-not $allPassed) {
        Write-Host ""
        Write-Host "One or more tests FAILED." -ForegroundColor Red
        exit 1
    }

    Write-Host ""
    Write-Host "All tests passed successfully." -ForegroundColor Green
}

# ── 2. DASHBOARD ─────────────────────────────────────────────────────────────
if (-not $TestOnly) {
    Print-Header "Safe Semantic Planner Dashboard"

    # Check if port is already listening (e.g. server already running)
    $already = $false
    try {
        $conn = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
        if ($conn) { $already = $true }
    } catch {}

    $url = "http://localhost:$Port"

    if ($already) {
        Write-Host "  Port $Port already in use - attaching to existing server." -ForegroundColor Yellow
        Write-Host "  Opening $url" -ForegroundColor Cyan
        Start-Process $url
    } else {
        Write-Host "  Starting HTTP server on port $Port..." -ForegroundColor DarkGray

        # Launch Python server as a background job
        $serverJob = Start-Job -ScriptBlock {
            param($dir, $port)
            Set-Location $dir
            python -m http.server $port 2>&1
        } -ArgumentList $root, $Port

        # Wait for the server to start
        Start-Sleep -Milliseconds 1200

        Write-Host "  Opening $url" -ForegroundColor Cyan
        Start-Process $url

        Write-Host ""
        Write-Host "  Press Ctrl+C to stop the server." -ForegroundColor DarkGray
        Write-Host ""

        # Stream server output until the user hits Ctrl+C
        try {
            while ($true) {
                $out = Receive-Job $serverJob
                if ($out) { Write-Host "  [http] $out" -ForegroundColor DarkGray }
                Start-Sleep -Milliseconds 500
            }
        } finally {
            Stop-Job  $serverJob -ErrorAction SilentlyContinue
            Remove-Job $serverJob -ErrorAction SilentlyContinue
            Write-Host "  Server stopped." -ForegroundColor DarkGray
        }
    }
}
