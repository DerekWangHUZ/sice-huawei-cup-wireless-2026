# ============================================================================
# File    : run_sim.ps1
# Purpose : Headless simulation runner for the coprocessor (Artix-7) project.
#           No Vivado GUI. Calls xvlog / xelab / xsim command-line versions.
#
# Usage   : powershell -ExecutionPolicy Bypass -File run_sim.ps1
#           (or: pwsh -File run_sim.ps1)
#           Location is auto-detected; no cd required.
#
# Outputs :
#   sim/logs/xvlog.log   compile log
#   sim/logs/xelab.log   elaborate log
#   sim/logs/xsim.log    run log (contains TB PASS/FAIL prints)
#   sim/wave/tb_top.wdb  waveform database
#
# Exit    : 0 = all checks passed ; 1 = any stage failed
#
# NOTE: This file is intentionally ASCII-only so that both Windows PowerShell
#       5.1 (parses BOM-less files as ANSI) and PowerShell 7 work correctly.
# ============================================================================

$ErrorActionPreference = 'Continue'

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
$SimDir    = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjRoot  = Split-Path -Parent $SimDir
$SrcDir    = Join-Path $ProjRoot 'rtl'
$LogDir    = Join-Path $SimDir 'logs'
$WaveDir   = Join-Path $SimDir 'wave'
$TbFile    = Join-Path $SimDir 'tb_top.v'

$VivadoBin = 'C:\AMDDesignTools\2026.1\Vivado\bin'
$Xvlog     = Join-Path $VivadoBin 'xvlog.bat'
$Xelab     = Join-Path $VivadoBin 'xelab.bat'
$Xsim      = Join-Path $VivadoBin 'xsim.bat'

# ---------------------------------------------------------------------------
# Prepare directories
# ---------------------------------------------------------------------------
New-Item -ItemType Directory -Path $LogDir  -Force | Out-Null
New-Item -ItemType Directory -Path $WaveDir -Force | Out-Null

# Clean stale artifacts so an old snapshot can never be reused
if (Test-Path (Join-Path $SimDir 'xsim.dir')) {
    Remove-Item -Recurse -Force (Join-Path $SimDir 'xsim.dir')
}
Remove-Item -Path (Join-Path $SimDir '*.wdb') -Force -ErrorAction SilentlyContinue
Remove-Item -Path (Join-Path $SimDir '*.jou') -Force -ErrorAction SilentlyContinue
Remove-Item -Path (Join-Path $SimDir '*.log') -Force -ErrorAction SilentlyContinue
Remove-Item -Path (Join-Path $SimDir '*.pb')  -Force -ErrorAction SilentlyContinue

# ---------------------------------------------------------------------------
# Source list (explicit, no glob - see docs/CHANGELOG.md for why glob was
# rejected for this repo's Vivado workflow)
# ---------------------------------------------------------------------------
$Sources = @(
    (Join-Path $SrcDir 'uart_rx.v'),
    (Join-Path $SrcDir 'cmd_decoder.v'),
    (Join-Path $SrcDir 'seg_display.v'),
    (Join-Path $SrcDir 'buzzer_ctrl.v'),
    (Join-Path $SrcDir 'led_ctrl.v'),
    (Join-Path $SrcDir 'top.v'),
    $TbFile
)

foreach ($f in $Sources) {
    if (-not (Test-Path $f)) {
        Write-Host 'SIM RESULT: FAIL'
        Write-Host "  missing source file: $f"
        exit 1
    }
}

Write-Host '=== [1/3] xvlog: compile sources ==='
Push-Location $SimDir
& $Xvlog $Sources 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'xvlog.log')
$xvlogOk = ($LASTEXITCODE -eq 0) -and
           (-not (Select-String -Path (Join-Path $LogDir 'xvlog.log') -Pattern 'ERROR' -Quiet))
Pop-Location

if (-not $xvlogOk) {
    Write-Host 'SIM RESULT: FAIL (xvlog)'
    exit 1
}

Write-Host '=== [2/3] xelab: elaborate tb_top ==='
Push-Location $SimDir
& $Xelab tb_top -s tb_top -debug typical 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'xelab.log')
$xelabOk = ($LASTEXITCODE -eq 0) -and
           (-not (Select-String -Path (Join-Path $LogDir 'xelab.log') -Pattern 'ERROR' -Quiet))
Pop-Location

if (-not $xelabOk) {
    Write-Host 'SIM RESULT: FAIL (xelab)'
    exit 1
}

# ---------------------------------------------------------------------------
# [3/3] Run
#
# IMPORTANT: running xsim directly inside this repo's deep path crashes with
#   "Could not open file mapping object (2)" + EXCEPTION_ACCESS_VIOLATION.
# The very same snapshot runs fine from %TEMP% (verified), so this is a
# path-related shared-memory mapping issue, not an RTL/testbench problem.
# Strategy: elaborate here, copy the snapshot to a temp dir, run it there,
# then copy logs and waveform back into the repo.
# ---------------------------------------------------------------------------
Write-Host '=== [3/3] xsim: run simulation ==='

$RunDir = Join-Path $env:TEMP 'coprocessor_sim_run'
if (Test-Path $RunDir) { Remove-Item -Recurse -Force $RunDir }
New-Item -ItemType Directory -Path $RunDir | Out-Null
Copy-Item (Join-Path $SimDir 'xsim.dir') $RunDir -Recurse -Force

Push-Location $RunDir
& $Xsim tb_top -runall 2>&1 | Tee-Object -FilePath (Join-Path $LogDir 'xsim.log')
$xsimExit = $LASTEXITCODE
Pop-Location

Remove-Item -Path (Join-Path $RunDir 'hs_err_pid*') -Force -ErrorAction SilentlyContinue

$wdb = Join-Path $RunDir 'tb_top.wdb'
if (Test-Path $wdb) {
    Copy-Item $wdb (Join-Path $WaveDir 'tb_top.wdb') -Force
    Write-Host "  waveform: $(Join-Path $WaveDir 'tb_top.wdb')"
} else {
    Write-Host '  note: no wdb produced in run dir'
}

# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
$LogFile = Join-Path $LogDir 'xsim.log'
$passed  = Select-String -Path $LogFile -Pattern 'TB: ALL CHECKS PASSED' -Quiet
$failed  = (Select-String -Path $LogFile -Pattern 'TB: [0-9]+ CHECK\(S\) FAILED' -Quiet) -or
           (Select-String -Path $LogFile -Pattern 'FATAL' -Quiet) -or
           ($xsimExit -ne 0)

if ($passed -and -not $failed) {
    Write-Host ''
    Write-Host 'SIM RESULT: PASS'
    exit 0
} else {
    Write-Host ''
    Write-Host 'SIM RESULT: FAIL'
    Write-Host "  see $LogFile"
    exit 1
}
