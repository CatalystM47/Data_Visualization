@echo off
REM Drone Flight Log Analyzer - Windows launcher
REM Usage: double-click this file, or run it from Command Prompt.
chcp 65001 >nul
setlocal

cd /d "%~dp0"

echo ==========================================================
echo   Drone Flight Log Analyzer - setup and run
echo ==========================================================

REM --- Find a working Python launcher ---
set "PY="
where py >nul 2>nul && set "PY=py"
if not defined PY (
    where python >nul 2>nul && set "PY=python"
)
if not defined PY (
    echo [ERROR] Python not found. Install it from https://www.python.org
    echo During install, make sure to check "Add Python to PATH".
    pause
    exit /b 1
)
echo Using Python launcher: %PY%

REM --- Create virtual environment (first run only) ---
if not exist ".venv\Scripts\python.exe" (
    echo [1/3] Creating virtual environment...
    %PY% -m venv .venv
    if errorlevel 1 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
)

REM --- Use the venv python directly (no activation needed) ---
set "VENV_PY=.venv\Scripts\python.exe"

REM --- Install packages ---
echo [2/3] Installing required packages (first run may take a few minutes)...
"%VENV_PY%" -m pip install --upgrade pip
"%VENV_PY%" -m pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] Package installation failed. Check your internet connection.
    pause
    exit /b 1
)

REM --- Run ---
echo [3/3] Starting the analyzer...
echo If your browser does not open automatically, go to http://localhost:10917
"%VENV_PY%" app.py

pause
endlocal
