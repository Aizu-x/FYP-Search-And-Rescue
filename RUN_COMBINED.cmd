@echo off
setlocal
cd /d "%~dp0"
rem Python: the environment named in FYP_SRM.ini (the one FYP_SRM.exe uses), else .venv in this folder.
set "PY="
if exist "FYP_SRM.ini" for /f "usebackq eol=; tokens=1,* delims== " %%a in ("FYP_SRM.ini") do if /i "%%a"=="python" set "PY=%%b"
if defined PY if not exist "%PY%" set "PY="
if not defined PY if exist ".venv\Scripts\python.exe" set "PY=.venv\Scripts\python.exe"
if not defined PY goto nopython
"%PY%" -B DAG_scheduler\run_combined.py %*
set "DAG_EXIT=%ERRORLEVEL%"
echo.
echo This checks components using saved inputs. It does not move the robot.
pause
exit /b %DAG_EXIT%
:nopython
echo No Python environment was found.
echo Put the path of its python.exe in FYP_SRM.ini (python=...), or follow EDGE_SETUP.md to create .venv here.
pause
exit /b 2
