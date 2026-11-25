@echo off
chcp 65001 >nul
cd /d "%~dp0"
REM ============================================================================
REM @file        start_digital_twin.bat
REM @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
REM @description สคริปต์สำหรับเปิด Local HTTP Server บน Windows เพื่อรันหน้าเว็บ
REM ----------------------------------------------------------------------------
REM @why_needed  เบราว์เซอร์กำหนดด้านความปลอดภัย (Secure Context) ว่าต้องเปิดผ่าน
REM              localhost หรือ HTTPS จึงจะอนุญาตให้ใช้งานกล้องเว็บแคม (getUserMedia)
REM              และ WebSerial API หากเปิด index.html ตรง ๆ ผ่าน file:// จะถูกบล็อก
REM @data_flow   ย้ายโฟลเดอร์ทำงานไปยังที่ตั้งไฟล์นี้ -> ตรวจหา Python 3
REM              -> เปิด Python HTTP Server ที่ 127.0.0.1:8080 -> เปิดเบราว์เซอร์อัตโนมัติ
REM ============================================================================
title 4-DOF Robotic Arm Digital Twin Server (ISY49303 IoT)
echo ========================================================
echo   ISY49303 IoT - Group01_Project (4-DOF Digital Twin)
echo   Starting Local Server at http://localhost:8080 ...
echo ========================================================

set "PY_CMD="
where python >nul 2>&1
if %errorlevel%==0 (
  set "PY_CMD=python"
) else (
  where py >nul 2>&1
  if %errorlevel%==0 (
    set "PY_CMD=py -3"
  )
)

if "%PY_CMD%"=="" (
  echo [ERROR] ไม่พบ Python 3 ในระบบ กรุณาติดตั้ง Python 3 แล้วลองใหม่อีกครั้ง
  pause
  exit /b 1
)

REM หน่วงเวลา 2 วินาทีก่อนเปิดเบราว์เซอร์ เพื่อให้ Local Server พร้อมรับการเชื่อมต่อ
start "" /b cmd /c "timeout /t 2 /nobreak >nul && start "" "http://localhost:8080""
%PY_CMD% -m http.server 8080 --bind 127.0.0.1
pause
