@echo off
chcp 65001 >nul
title 4-DOF Robotic Arm Digital Twin Server
echo ========================================================
echo   4-DOF Robotic Arm Digital Twin (Hand Tracking + 3D)
echo   Starting Local Server at http://localhost:8080 ...
echo ========================================================
start "" "http://localhost:8080"
python -m http.server 8080 --bind 127.0.0.1
pause
