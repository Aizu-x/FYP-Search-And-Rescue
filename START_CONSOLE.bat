@echo off
rem Double-click to start the UGV console. Keep this window open while you use it.
cd /d "%~dp0"
if exist "runtime\python.exe" (
  rem the Python bundled with the Windows package from the Releases page
  "runtime\python.exe" nav_main.py gui
) else if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" nav_main.py gui
) else (
  python nav_main.py gui
)
echo.
echo The console program has ended.
pause
