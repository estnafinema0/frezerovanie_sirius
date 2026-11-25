@echo off
cd /d "%~dp0"
if "%~1"=="" (
  MillingAnalyzer.exe --serve
) else (
  MillingAnalyzer.exe --serve --step "%~f1"
)
pause
