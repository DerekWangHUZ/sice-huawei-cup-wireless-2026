# ============================================================================
# File    : run_offline_tests.ps1
# Purpose : One-shot runner for the fall_core offline self-test.
#           No board, no GUI. Regenerates synthetic frames, compiles the
#           harness against the SAME fall_core.c the firmware uses, runs all
#           scenarios, tees the log into logs/.
#
# Usage   : powershell -ExecutionPolicy Bypass -File run_offline_tests.ps1
#
# Exit    : 0 = all scenarios passed ; 1 = any failure
#
# Host compiler:
#   This machine has no MSVC / MinGW. It uses lcc64 shipped with MATLAB
#   (pure C99, sufficient for the platform-independent fall_core.c).
#   If MATLAB moves, override the two paths below with any C99 compiler.
#   (Alternative: install MinGW via `winget install BrechtSanders.Mingw`)
#
# NOTE: ASCII-only file so Windows PowerShell 5.1 parses it correctly.
# ============================================================================

$ErrorActionPreference = 'Continue'

$Here    = Split-Path -Parent $MyInvocation.MyCommand.Path
$MainDir = Join-Path (Split-Path -Parent (Split-Path -Parent $Here)) 'main'
$LogDir  = Join-Path $Here 'logs'

# --- host compiler (MATLAB lcc64) ---
$LccRoot = 'C:\Program Files\MATLAB\R2026a\sys\lcc64\lcc64'
$Lcc     = Join-Path $LccRoot 'bin\lcc64.exe'
$LccLnk  = Join-Path $LccRoot 'bin\lcclnk64.exe'
$LccInc  = Join-Path $LccRoot 'include64'
$LccLib  = Join-Path $LccRoot 'lib64'

# --- python for the generator ---
$Py = 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe'

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
foreach ($p in @($Lcc, $LccLnk, $Py, (Join-Path $MainDir 'fall_core.c'))) {
    if (-not (Test-Path $p)) {
        Write-Host "OFFLINE TEST RESULT: FAIL"
        Write-Host "  missing prerequisite: $p"
        exit 1
    }
}

Push-Location $Here

# ---------------------------------------------------------------------------
# [1/3] Generate synthetic frames
# ---------------------------------------------------------------------------
Write-Host '=== [1/3] generate synthetic posture frames ==='
& $Py 'gen_frames.py' 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'gen_frames.log')
if ($LASTEXITCODE -ne 0) {
    Write-Host 'OFFLINE TEST RESULT: FAIL (gen_frames.py)'
    Pop-Location
    exit 1
}

# ---------------------------------------------------------------------------
# [2/3] Compile harness (includes ../../main/fall_core.c directly)
# ---------------------------------------------------------------------------
Write-Host '=== [2/3] compile test harness (host C99) ==='
Remove-Item 'test_harness.obj' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness.exe' -Force -ErrorAction SilentlyContinue

& $Lcc 'test_harness.c' -I"$LccInc" 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'compile.log')
if ($LASTEXITCODE -ne 0) {
    Write-Host 'OFFLINE TEST RESULT: FAIL (compile)'
    Pop-Location
    exit 1
}

& $LccLnk -L"$LccLib" 'test_harness.obj' -o 'test_harness.exe' 2>&1 |
    Tee-Object -FilePath (Join-Path $LogDir 'link.log') -Append
if (-not (Test-Path 'test_harness.exe')) {
    Write-Host 'OFFLINE TEST RESULT: FAIL (link)'
    Pop-Location
    exit 1
}

# ---------------------------------------------------------------------------
# [3/3] Run all scenarios
# ---------------------------------------------------------------------------
Write-Host '=== [3/3] run scenarios ==='
& '.\test_harness.exe' 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'offline_test.log')
$runExit = $LASTEXITCODE
Pop-Location

$logFile = Join-Path $LogDir 'offline_test.log'
$passed  = Select-String -Path $logFile -Pattern 'ALL \d+ SCENARIOS PASSED' -Quiet

if ($passed -and ($runExit -eq 0)) {
    Write-Host ''
    Write-Host 'OFFLINE TEST RESULT: PASS'
    Write-Host "  log: $logFile"
    exit 0
} else {
    Write-Host ''
    Write-Host 'OFFLINE TEST RESULT: FAIL'
    Write-Host "  log: $logFile"
    exit 1
}
