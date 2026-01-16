@echo off
for /f "delims=" %%i in ('dir /b /od ".\Output\*.hex"') do set HEXFILE=.\Output\%%i

if "%HEXFILE%"=="" (
    echo [Keil Build] No HEX file found in .\Output\
    exit /b 1
)

echo [Keil Build] HEX file: %HEXFILE%
powershell -ExecutionPolicy Bypass -File "%~dp0calc_crc32.ps1" "%HEXFILE%"