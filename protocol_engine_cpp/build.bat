@echo off
REM ============================================================
REM  One-click build + run all unit tests under tests\*.cpp
REM  Usage: double click this file, or run "build.bat" in this folder.
REM  Requires g++ (MinGW-w64) in PATH.
REM  Note: uses %~dp0 so Chinese paths are handled by cmd correctly.
REM  Note: tests\test_win32_serial_live.cpp is skipped on purpose --
REM        it needs a real COM port and a compiler that defines _WIN32
REM        (MinGW-w64); build it with build_win32.sh instead.
REM ============================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

where g++ >nul 2>nul
if errorlevel 1 (
    echo [ERROR] g++ not found in PATH. Install MinGW-w64 or use Qt Creator / CMake instead.
    exit /b 1
)

if not exist build mkdir build

set FAILED=0
for /f "delims=" %%F in ('dir /b /a-d "tests\*.cpp" ^| findstr /v /i "test_win32_serial_live"') do (
    set NAME=%%~nF
    echo.
    echo === building !NAME! ...
    g++ -std=c++17 -Wall -Wextra -pthread -Iinclude "tests\!NAME!.cpp" -o "build\!NAME!.exe"
    if errorlevel 1 (
        echo [ERROR] compile failed: tests\!NAME!.cpp
        set FAILED=1
    ) else (
        "build\!NAME!.exe"
        if errorlevel 1 (
            echo [FAIL] !NAME! had failing checks.
            set FAILED=1
        )
    )
)
echo.
echo [INFO] skipped tests\test_win32_serial_live.cpp (needs real COM port + MinGW-w64, see build_win32.sh)

echo.
echo === building armctl (CLI) ...
g++ -std=c++17 -Wall -Wextra -pthread -Iinclude src\main.cpp -o build\armctl.exe
if errorlevel 1 (
    echo [ERROR] compile failed: src\main.cpp
    set FAILED=1
)

echo.
if "%FAILED%"=="1" (
    echo [FAILED] see messages above.
    exit /b 1
)
echo [OK] all tests passed, armctl built.
endlocal
