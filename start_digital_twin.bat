@echo off
chcp 65001 >nul
REM ============================================================================
REM @file        start_digital_twin.bat
REM @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
REM @description สคริปต์สำหรับเปิด Local HTTP Server บน Windows เพื่อรันหน้าเว็บ
REM ----------------------------------------------------------------------------
REM @why_needed  เบราว์เซอร์กำหนดด้านความปลอดภัย (Secure Context) ว่าต้องเปิดผ่าน
REM              localhost หรือ HTTPS จึงจะอนุญาตให้ใช้งานกล้องเว็บแคม (getUserMedia)
REM              และ WebSerial API หากเปิด index.html ตรง ๆ ผ่าน file:// จะถูกบล็อก
REM @data_flow   เปิด Python HTTP Server ที่ 127.0.0.1:8080 (เข้าถึงได้เฉพาะเครื่องนี้)
REM              -> เปิดเบราว์เซอร์อัตโนมัติ -> ให้บริการไฟล์ index.html และ Assets
REM ============================================================================
title 4-DOF Robotic Arm Digital Twin Server (ISY49303 IoT)
echo ========================================================
echo   ISY49303 IoT - Group01_Project (4-DOF Digital Twin)
echo   Starting Local Server at http://localhost:8080 ...
echo ========================================================
start "" "http://localhost:8080"
python -m http.server 8080 --bind 127.0.0.1
pause
