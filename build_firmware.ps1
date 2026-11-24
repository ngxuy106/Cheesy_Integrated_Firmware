<#
.SYNOPSIS
    Build flashable STM32F411CEU6 firmware (.elf, .bin, .hex) using STM32CubeIDE's
    bundled arm-none-eabi-gcc, CMake, and Ninja (or tools already in PATH).
#>
param(
    [ValidateSet("Release", "Debug")]
    [string]$Preset = "Release",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$ProjectDir = $PSScriptRoot

function Add-ToolDirToPath([string]$ExeName) {
    if (Get-Command $ExeName -ErrorAction SilentlyContinue) { return }
    $found = Get-ChildItem "C:\ST" -Recurse -Filter $ExeName -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($found) {
        $dir = Split-Path -Parent $found.FullName
        $env:PATH = "$dir;$env:PATH"
        Write-Host "[TOOLCHAIN] Added $dir"
    } else {
        throw "Required tool '$ExeName' not found in PATH or C:\ST"
    }
}

Add-ToolDirToPath "arm-none-eabi-gcc.exe"
Add-ToolDirToPath "cmake.exe"
Add-ToolDirToPath "ninja.exe"

$BuildDir = Join-Path $ProjectDir "build\$Preset"
if ($Clean -and (Test-Path $BuildDir)) {
    Remove-Item -Recurse -Force $BuildDir
}

Push-Location $ProjectDir
try {
    Write-Host "`n=== Configuring CMake Preset: $Preset ==="
    & cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host "`n=== Building STM32F411CEU6 Firmware ($Preset) ==="
    & cmake --build --preset $Preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    Write-Host "`n=== Build Artifacts ==="
    Get-ChildItem $BuildDir -Include "cheesy_integrated.elf","cheesy_integrated.bin","cheesy_integrated.hex" -Recurse |
        Select-Object FullName, Length | Format-Table -AutoSize
} finally {
    Pop-Location
}
