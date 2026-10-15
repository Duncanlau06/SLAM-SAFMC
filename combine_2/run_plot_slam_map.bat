@echo off
setlocal
echo ========================================================
echo   SAFMC Swarm 2D SLAM Map Visualizer ^& Live Plotter
echo ========================================================
echo.

:: Detect Python executable
where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set PY_CMD=python
    goto RUN
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set PY_CMD=py
    goto RUN
)

if exist "C:\Espressif\tools\python\v5.5.5\venv\Scripts\python.exe" (
    set PY_CMD="C:\Espressif\tools\python\v5.5.5\venv\Scripts\python.exe"
    goto RUN
)

echo [!] Error: Python was not found in PATH or at default Espressif directory.
echo     Please run this script from the ESP-IDF Command Prompt.
pause
exit /b 1

:RUN
if "%~1"=="" (
    echo [1] Automatic Live Serial Mode (COM4)
    echo [2] Plot from flight_log.txt
    echo [3] Plot from clipboard
    echo.
    set /p choice="Select mode [1, 2, or 3] (default: 1): "
    if "%choice%"=="2" goto PLOT_FILE
    if "%choice%"=="3" goto PLOT_CLIP
    goto PLOT_LIVE
)

if "%~1"=="--live" goto PLOT_LIVE
if "%~1"=="--clipboard" goto PLOT_CLIP

:PLOT_FILE
set TARGET_FILE=%~1
if "%TARGET_FILE%"=="" set TARGET_FILE=flight_log.txt
echo [+] Reading %TARGET_FILE%...
%PY_CMD% "%~dp0plot_slam_map.py" "%TARGET_FILE%"
goto DONE

:PLOT_CLIP
echo [+] Reading SLAM graph from clipboard...
%PY_CMD% "%~dp0plot_slam_map.py" --clipboard
goto DONE

:PLOT_LIVE
set PORT=COM4
if not "%~2"=="" set PORT=%~2
echo [+] Connecting live to ESP32 on %PORT%...
echo     (Image will automatically generate and display upon landing)
%PY_CMD% "%~dp0plot_slam_map.py" --live --port %PORT%
goto DONE

:DONE
pause
