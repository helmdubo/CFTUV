@echo off
REM Полевой цикл на building.002. Вся работа в field_cycle.ps1 — батник только
REM запускает его с теми же аргументами. Density обязана быть явной:
REM field_cycle.bat -Density 1 -Install
chcp 65001 >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0field_cycle.ps1" %*
if errorlevel 1 (
  echo.
  echo Полевой прогон НЕ прошёл. Смотрите строки выше.
)
pause
