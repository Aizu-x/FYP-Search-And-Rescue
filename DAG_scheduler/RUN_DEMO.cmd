@echo off
setlocal
cd /d "%~dp0"
set "DAG_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%DAG_PYTHON%" goto bundled
where py >nul 2>nul
if not errorlevel 1 goto launcher
where python >nul 2>nul
if not errorlevel 1 goto pythonpath
echo Python 3.10 or newer is required. Open README.md for instructions.
pause
exit /b 2
:bundled
"%DAG_PYTHON%" -B run_mission.py
goto finished
:launcher
py -3 -B run_mission.py
goto finished
:pythonpath
python -B run_mission.py
:finished
set "DAG_EXIT=%ERRORLEVEL%"
echo.
echo Finished with exit code %DAG_EXIT%. Results are in the runs folder.
pause
exit /b %DAG_EXIT%
