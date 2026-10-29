# ============================================================================
# File    : run_offline_tests.ps1
# Purpose : One-shot runner for the fall_core offline self-test.
#           No board, no GUI. Regenerates synthetic frames, compiles the
#           harness against the SAME fall_core.c the firmware uses, runs all
#           scenarios in TWO builds, tees the logs into logs/.
#
# Usage   : powershell -ExecutionPolicy Bypass -File run_offline_tests.ps1
#           Env overrides: OFFLINE_PY (python path),
#                          OFFLINE_CC (may be 'gcc', 'cl', or a full path
#                          to a compiler)
#
# Two binaries from the same test_harness.c:
#   test_harness.exe          default build        (posture gate off,  19 scenes)
#   test_harness_posture.exe  -DPOSTURE_CHECK_ENABLE=1 (gate on, 22 scenes;
#                             gate is fail-open until calibrated, so the
#                             shared scenes must behave identically in both
#                             builds)
# All three supported compilers (gcc / cl / lcc64) accept -D style defines;
# lcc64 uses the same compile-then-link two-step flow as its default build.
#
# Exit    : 0 = all scenarios passed in both builds ; 1 = any failure
#
# Compiler detection order:
#   1. OFFLINE_CC env override
#   2. 'gcc' and 'python' found on PATH
#   3. MSVC cl.exe via vcvars64.bat (VS2022 BuildTools standard location)
#   4. Legacy fallback: lcc64 shipped with MATLAB (original dev machine)
#
# NOTE: ASCII-only file so Windows PowerShell 5.1 parses it correctly.
# ============================================================================

$ErrorActionPreference = 'Continue'

$Here    = Split-Path -Parent $MyInvocation.MyCommand.Path
$MainDir = Join-Path (Split-Path -Parent (Split-Path -Parent $Here)) 'main'
$LogDir  = Join-Path $Here 'logs'

# --- python for the generator -----------------------------------------------
$Py = $null
if ($env:OFFLINE_PY -and (Test-Path $env:OFFLINE_PY)) {
    $Py = $env:OFFLINE_PY
} elseif (Get-Command 'python' -ErrorAction SilentlyContinue) {
    $Py = 'python'
} else {
    $Py = 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe'  # legacy dev machine
}

# --- host compiler -----------------------------------------------------------
$CcKind = $null   # 'gcc' | 'cl' | 'lcc64'

if ($env:OFFLINE_CC) {
    if ($env:OFFLINE_CC -eq 'cl') {
        $CcKind = 'cl'
    } elseif ($env:OFFLINE_CC -eq 'lcc64') {
        $CcKind = 'lcc64'
    } else {
        $CcKind = 'gcc'; $CcExe = $env:OFFLINE_CC   # 'gcc' or full path
    }
} elseif (Get-Command 'gcc' -ErrorAction SilentlyContinue) {
    $CcKind = 'gcc'; $CcExe = 'gcc'
} else {
    # MSVC: look for the standard BuildTools/Community vcvars64.bat locations
    $VsRoots = @(
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\18"
    )
    foreach ($root in $VsRoots) {
        $vcvars = Get-ChildItem -Path $root -Filter 'vcvars64.bat' `
                    -Recurse -ErrorAction SilentlyContinue |
                  Where-Object { $_.FullName -like '*VC\Auxiliary\Build*' } |
                  Select-Object -First 1
        if ($vcvars) { $CcKind = 'cl'; $Vcvars = $vcvars.FullName; break }
    }
}

# legacy lcc64 fallback (original dev machine)
if (-not $CcKind) { $CcKind = 'lcc64' }
if ($CcKind -eq 'lcc64') {
    $LccRoot = 'C:\Program Files\MATLAB\R2026a\sys\lcc64\lcc64'
    $Lcc     = Join-Path $LccRoot 'bin\lcc64.exe'
    $LccLnk  = Join-Path $LccRoot 'bin\lcclnk64.exe'
    $LccInc  = Join-Path $LccRoot 'include64'
    $LccLib  = Join-Path $LccRoot 'lib64'
}

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
foreach ($p in @((Join-Path $MainDir 'fall_core.c'), (Join-Path $MainDir 'csi_stats.c'))) {
    if (-not (Test-Path $p)) {
        Write-Host 'OFFLINE TEST RESULT: FAIL'
        Write-Host "  missing prerequisite: $p"
        exit 1
    }
}
if ($Py -and -not (Get-Command $Py -ErrorAction SilentlyContinue) -and -not (Test-Path $Py)) {
    Write-Host 'OFFLINE TEST RESULT: FAIL'
    Write-Host "  python not found: $Py"
    exit 1
}
if ($CcKind -eq 'lcc64' -and -not (Test-Path $Lcc)) {
    Write-Host 'OFFLINE TEST RESULT: FAIL'
    Write-Host "  lcc64 not found: $Lcc (set OFFLINE_CC to override)"
    exit 1
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
#
# Two binaries from the same test_harness.c:
#   test_harness.exe          default build        (posture gate off,  19 scenes)
#   test_harness_posture.exe  -DPOSTURE_CHECK_ENABLE=1 (gate on, 22 scenes;
#                             gate is fail-open until calibrated, so the
#                             shared scenes must behave identically in both
#                             builds)
# All three supported compilers (gcc / cl / lcc64) accept -D style defines;
# lcc64 uses the same compile-then-link two-step flow as its default build.
# ---------------------------------------------------------------------------
Write-Host "=== [2/3] compile test harness (host C99, compiler=$CcKind) ==="
Remove-Item 'test_harness.obj' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness.exe' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness_posture.obj' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness_posture.exe' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness.ilk' -Force -ErrorAction SilentlyContinue
Remove-Item 'test_harness.pdb' -Force -ErrorAction SilentlyContinue
Remove-Item (Join-Path $LogDir 'compile.log') -Force -ErrorAction SilentlyContinue
Remove-Item (Join-Path $LogDir 'link.log') -Force -ErrorAction SilentlyContinue

function Invoke-Compile([string]$kind, [string[]]$ccArgs, [string]$vcvarsBat,
                        [string]$lcc, [string]$lccInc, [string]$lccLnk,
                        [string]$lccLib, [string]$outExe) {
    if ($kind -eq 'gcc') {
        & $ccArgs[0] $ccArgs[1..($ccArgs.Length - 1)] 2>&1 |
            Tee-Object -FilePath (Join-Path $LogDir 'compile.log') -Append
        return ($LASTEXITCODE -eq 0) -and (Test-Path $outExe)
    } elseif ($kind -eq 'cl') {
        $cmd = '"' + $vcvarsBat + '" >nul && cl /nologo /std:c11 /O2 /W4 ' +
               ($ccArgs -join ' ')
        cmd /c $cmd 2>&1 |
            Tee-Object -FilePath (Join-Path $LogDir 'compile.log') -Append
        return ($LASTEXITCODE -eq 0) -and (Test-Path $outExe)
    } else {
        & $lcc $ccArgs[0] $ccArgs[1..($ccArgs.Length - 1)] 2>&1 |
            Tee-Object -FilePath (Join-Path $LogDir 'compile.log') -Append
        if ($LASTEXITCODE -ne 0) { return $false }
        $obj = [IO.Path]::GetFileNameWithoutExtension($ccArgs[0]) + '.obj'
        & $lccLnk -L"$lccLib" $obj -o $outExe 2>&1 |
            Tee-Object -FilePath (Join-Path $LogDir 'link.log') -Append
        return (Test-Path $outExe)
    }
}

$compileOk = $false
if ($CcKind -eq 'gcc') {
    $compileOk = Invoke-Compile 'gcc' @($CcExe, '-std=c99', '-O2', '-Wall',
                                        'test_harness.c', '-o', 'test_harness.exe') `
                                 $null $null $null $null $null 'test_harness.exe'
} elseif ($CcKind -eq 'cl') {
    $compileOk = Invoke-Compile 'cl' @('test_harness.c', '/Fe:test_harness.exe') `
                                 $Vcvars $null $null $null $null 'test_harness.exe'
} else {
    $compileOk = Invoke-Compile 'lcc64' @('test_harness.c', "-I$LccInc") `
                                 $null $Lcc $LccInc $LccLnk $LccLib 'test_harness.exe'
}

if (-not $compileOk) {
    Write-Host 'OFFLINE TEST RESULT: FAIL (compile)'
    Pop-Location
    exit 1
}

# Second build: posture gate enabled
$postureBuilt = $false
if ($CcKind -eq 'gcc') {
    $postureBuilt = Invoke-Compile 'gcc' @($CcExe, '-std=c99', '-O2', '-Wall',
                                            '-DPOSTURE_CHECK_ENABLE=1',
                                            'test_harness.c', '-o',
                                            'test_harness_posture.exe') `
                                     $null $null $null $null $null 'test_harness_posture.exe'
} elseif ($CcKind -eq 'cl') {
    $postureBuilt = Invoke-Compile 'cl' @('/DPOSTURE_CHECK_ENABLE=1',
                                           'test_harness.c',
                                           '/Fe:test_harness_posture.exe') `
                                   $Vcvars $null $null $null $null 'test_harness_posture.exe'
} elseif ($CcKind -eq 'lcc64') {
    # Source file stays ccArgs[0] (Invoke-Compile derives the .obj name from it);
    # lcc64 recognizes -D/-I options at any argv position.
    $postureBuilt = Invoke-Compile 'lcc64' @('test_harness.c',
                                             '-DPOSTURE_CHECK_ENABLE=1',
                                             "-I$LccInc") `
                                   $null $Lcc $LccInc $LccLnk $LccLib 'test_harness_posture.exe'
}
if ($postureBuilt) {
    Write-Host '  posture-gate build: test_harness_posture.exe'
} else {
    Write-Host '  posture-gate build: SKIPPED or FAILED (see compile.log)'
}

# ---------------------------------------------------------------------------
# [3/3] Run all scenarios (both builds must pass)
# ---------------------------------------------------------------------------
Write-Host '=== [3/3] run scenarios ==='
& '.\test_harness.exe' 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'offline_test.log')
$runExit = $LASTEXITCODE

if ($postureBuilt) {
    Write-Host ''
    Write-Host '--- posture-gate build ---'
    & '.\test_harness_posture.exe' 2>&1 |
        Tee-Object -FilePath (Join-Path $LogDir 'offline_test_posture.log')
    if ($LASTEXITCODE -ne 0) { $runExit = 1 }
}
Pop-Location

$logFile = Join-Path $LogDir 'offline_test.log'
$passed  = Select-String -Path $logFile -Pattern 'ALL \d+ SCENARIOS PASSED' -Quiet

$posturePassed = $true
if ($postureBuilt) {
    $postureLog   = Join-Path $LogDir 'offline_test_posture.log'
    $posturePassed = Select-String -Path $postureLog `
                        -Pattern 'ALL \d+ SCENARIOS PASSED' -Quiet
}

# Both binaries must exist and report ALL n SCENARIOS PASSED for overall PASS.
if ($passed -and $postureBuilt -and $posturePassed -and ($runExit -eq 0)) {
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
