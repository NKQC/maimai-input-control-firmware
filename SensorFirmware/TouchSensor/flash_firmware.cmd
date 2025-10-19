@echo off
setlocal

echo ========================================
echo Firmware Flash Script
echo ========================================

set HEX_FILE=build\TouchSensor\Debug\TouchSensor.hex
set ABS_HEX_FILE=%~dp0%HEX_FILE%
set ABS_HEX_FILE_UNIX=%ABS_HEX_FILE:\=/%

echo Checking hex file...
if exist "%HEX_FILE%" (
    echo Found hex file: %HEX_FILE%
    echo Absolute path: %ABS_HEX_FILE%
    echo Normalized path: %ABS_HEX_FILE_UNIX%
) else (
    echo Error: Hex file not found: %HEX_FILE%
    echo Please build the project first
    pause
    exit /b 1
)

echo ========================================
echo Flashing firmware...
echo ========================================

openocd -f interface/cmsis-dap.cfg -f target/psoc4.cfg -c "gdb_port disabled; transport select swd; adapter speed 1000; init; halt; flash write_image erase {%ABS_HEX_FILE_UNIX%}; reset; shutdown"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Firmware flash successful!
    echo ========================================
) else (
    echo.
    echo ========================================
    echo Flash failed, error code: %ERRORLEVEL%
    echo ========================================
)

pause