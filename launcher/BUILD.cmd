@echo off
rem Rebuilds FYP_SRM.exe. Its picture is icon.png in this folder: for a new design, replace that file
rem (keep the name), close the console, then double-click this file.
setlocal
cd /d "%~dp0"
set "CSC=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
if not exist "%CSC%" set "CSC=%WINDIR%\Microsoft.NET\Framework\v4.0.30319\csc.exe"
if not exist "%CSC%" goto nocompiler
if not exist "icon.png" goto nopicture
"%CSC%" /nologo /target:exe /optimize+ /out:MakeIcon.exe /r:System.Drawing.dll MakeIcon.cs
if errorlevel 1 goto failed
"%~dp0MakeIcon.exe" icon.png FYP_SRM.ico
if errorlevel 1 goto failed
"%CSC%" /nologo /target:winexe /optimize+ /win32icon:FYP_SRM.ico /out:..\FYP_SRM.exe /r:System.Windows.Forms.dll /r:System.Drawing.dll FYP_SRM.cs
if errorlevel 1 goto locked
"%~dp0MakeIcon.exe" --refresh ..\FYP_SRM.exe
del MakeIcon.exe FYP_SRM.ico
echo.
echo Done: FYP_SRM.exe was built with the picture from launcher\icon.png.
echo If Windows still shows the old picture on the file for a while, that is only Windows
echo remembering it: the .exe already has the new one.
pause
exit /b 0
:locked
echo.
echo If the console is open, close it, wait half a minute and run this again.
goto failed
:nopicture
echo The picture launcher\icon.png is missing. Put a square PNG picture with that name in this folder.
goto failed
:nocompiler
echo The C# compiler of the .NET Framework 4 was not found on this computer.
:failed
if exist MakeIcon.exe del MakeIcon.exe
if exist FYP_SRM.ico del FYP_SRM.ico
echo.
echo FYP_SRM.exe was NOT rebuilt.
pause
exit /b 1
