@echo off
REM Double-click installs the addon and the kernel into Blender, then verifies
REM the fingerprints. All the work is in install_to_blender.ps1 - batch can do
REM neither hashing nor path discovery.
chcp 65001 >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install_to_blender.ps1" %*
if errorlevel 1 (
  echo.
  echo Установка НЕ завершена. Смотрите строки выше.
)
pause
