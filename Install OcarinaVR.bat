@echo off
rem OcarinaVR: builds the game from YOUR copy, with the VR mode (see README.md). 10 to 45 minutes, about 60 GB free.
rem A window asks for your game file (.nup, version 1.1.1). Run it again after an update of OcarinaVR.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
echo.
pause
