<#
.SYNOPSIS
    Compile and run all host-side unit tests and full closed-loop 2D simulation
    for HighLevel/ modules using PC GCC (-Wall -Wextra -Werror).
#>
$ErrorActionPreference = "Stop"
$TestsDir = $PSScriptRoot
$RootDir  = Split-Path -Parent $TestsDir

$HighLevelSrc = @(
    "$RootDir\HighLevel\Src\ekf_localization.c",
    "$RootDir\HighLevel\Src\arena_map.c",
    "$RootDir\HighLevel\Src\obstacle_mapper.c",
    "$RootDir\HighLevel\Src\astar_planner.c",
    "$RootDir\HighLevel\Src\pure_pursuit.c",
    "$RootDir\HighLevel\Src\mission_fsm.c"
)

$IncFlags = @("-I$RootDir\Interface\Inc", "-I$RootDir\HighLevel\Inc")
$CFlags   = @("-std=c11", "-O2", "-Wall", "-Wextra", "-Werror")

$OutDir = Join-Path $TestsDir "bin"
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null

$Suites = @(
    @{ Name = "test_ekf_odometry";     Src = "$TestsDir\test_ekf_odometry.c" },
    @{ Name = "test_obstacle_mapper";  Src = "$TestsDir\test_obstacle_mapper.c" },
    @{ Name = "test_navigation_e2e";   Src = "$TestsDir\test_navigation_e2e.c" }
)

$AnyFailed = $false
foreach ($s in $Suites) {
    $exe = Join-Path $OutDir "$($s.Name).exe"
    Write-Host "`n>>> Building $($s.Name) ..."
    & gcc @CFlags @IncFlags $s.Src @HighLevelSrc -lm -o $exe
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[BUILD FAILED] $($s.Name)" -ForegroundColor Red
        $AnyFailed = $true
        continue
    }
    Write-Host ">>> Executing $($s.Name) ..."
    & $exe
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[TEST FAILED] $($s.Name)" -ForegroundColor Red
        $AnyFailed = $true
    }
}

if ($AnyFailed) {
    Write-Host "`n=== HOST TEST SUITE FAILED ===" -ForegroundColor Red
    exit 1
} else {
    Write-Host "`n=== ALL HOST TESTS PASSED 100% ===" -ForegroundColor Green
    exit 0
}
