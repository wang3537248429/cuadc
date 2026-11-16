@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_windows.ps1" %*
if errorlevel 1 (
  echo.
  echo Windows package build failed.
  pause
  exit /b 1
)
echo.
echo Windows package build completed.
pause
