@echo off
rem Double-click to start the UGV console (version 2). Keep this window open while you use it.
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" nav_main.py gui
) else (
  python nav_main.py gui
)
echo.
echo The console program has ended.
pause
