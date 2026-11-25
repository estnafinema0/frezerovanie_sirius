@echo off
cd /d "%~dp0"
MillingAnalyzer.exe --serve --ethernet-profile Ethernet-IP.txt
pause
