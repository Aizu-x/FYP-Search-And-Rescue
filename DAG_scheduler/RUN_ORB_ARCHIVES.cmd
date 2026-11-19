@echo off
setlocal
cd /d "%~dp0"
set "DAG_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%DAG_PYTHON%" goto bundled
where py >nul 2>nul
if not errorlevel 1 goto launcher
where python >nul 2>nul
if not errorlevel 1 goto pythonpath
echo Python 3.10 or newer is required. See ORB_INTEGRATION.md.
pause
exit /b 2
:bundled
"%DAG_PYTHON%" -B run_orb.py --source archive --sequence all
goto finished
:launcher
py -3 -B run_orb.py --source archive --sequence all
goto finished
:pythonpath
python -B run_orb.py --source archive --sequence all
:finished
set "DAG_EXIT=%ERRORLEVEL%"
echo.
echo Exit code %DAG_EXIT%. These are saved-result imports, not fresh SLAM runs.
pause
exit /b %DAG_EXIT%
