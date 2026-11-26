@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"

echo ============================================================
echo   CHEESY INTEGRATED FIRMWARE - BUILD ^& FLASH (STM32F411CEU6)
echo ============================================================

call :add_tool "arm-none-eabi-gcc.exe" || goto :fail
call :add_tool "cmake.exe" || goto :fail
call :add_tool "ninja.exe" || goto :fail

echo.
echo [1/3] Configuring CMake (Release)...
cmake --preset Release
if errorlevel 1 goto :fail

echo.
echo [2/3] Building firmware...
cmake --build --preset Release
if errorlevel 1 goto :fail

echo.
echo [OK] Built artifacts:
echo   - build\Release\cheesy_integrated.bin
echo   - build\Release\cheesy_integrated.hex

if /I "%~1"=="--build-only" (
    echo [OK] Build finished ^(--build-only^).
    exit /b 0
)

call :find_programmer || goto :fail

echo.
echo [3/3] Ready to flash via ST-Link V2 (SWDIO=PA13, SWCLK=PA14, GND, 3V3).
echo Lift wheels off the floor before flashing!
pause

"%CLI_EXE%" -c port=SWD mode=UR -w "build\Release\cheesy_integrated.bin" 0x08000000 -v -rst
if errorlevel 1 goto :fail

echo.
echo [OK] Flash & verify succeeded! Keep robot still for 5s on boot for IMU calibration.
pause
exit /b 0

:add_tool
set "EXE=%~1"
where "%EXE%" >nul 2>&1
if not errorlevel 1 exit /b 0
if exist "C:\ST" (
    for /f "delims=" %%F in ('dir /b /s "C:\ST\%EXE%" 2^>nul') do (
        set "PATH=%%~dpF;!PATH!"
        echo [TOOLCHAIN] Added %%~dpF
        exit /b 0
    )
)
echo [ERROR] Could not find %EXE% in PATH or C:\ST.
exit /b 1

:find_programmer
set "CLI_EXE="
for /f "delims=" %%F in ('where STM32_Programmer_CLI.exe 2^>nul') do (
    set "CLI_EXE=%%F"
    exit /b 0
)
if exist "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" (
    set "CLI_EXE=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    exit /b 0
)
if exist "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" (
    set "CLI_EXE=C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    exit /b 0
)
echo [ERROR] STM32_Programmer_CLI.exe not found. Please install STM32CubeProgrammer.
exit /b 1

:fail
echo.
echo [ERROR] Operation failed. Check messages above.
if /I not "%~1"=="--build-only" pause
exit /b 1
