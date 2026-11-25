/**
 * =============================================================================
 * @file        servo_test_esp32/servo_test_esp32.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description เฟิร์มแวร์สำหรับทดสอบการทำงานเบื้องต้นของ Servo Motor 4 ตัวบน ESP32
 *              (MG90S จำนวน 2 ตัว + SG90 จำนวน 2 ตัว) ก่อนประกอบเข้ากับโครงแขนกลจริง
 * -----------------------------------------------------------------------------
 * @pinout (ตรงกับ arduino_servo_bridge.ino)
 *   - Servo 1 (MG90S #1 — Base Yaw / ฐาน)        : ESP32 GPIO 13
 *   - Servo 2 (MG90S #2 — Shoulder / ไหล่)       : ESP32 GPIO 12 (หากบูตมีปัญหา เปลี่ยนเป็น GPIO 26)
 *   - Servo 3 (SG90  #1 — Elbow / ศอก)           : ESP32 GPIO 14
 *   - Servo 4 (SG90  #2 — Gripper / หัวคีบ)      : ESP32 GPIO 27
 * @wiring_note
 *   - สายสัญญาณ (ส้ม/เหลือง) -> ขา GPIO ของ ESP32 ตามตารางด้านบน
 *   - สายไฟเลี้ยง (แดง)       -> แหล่งจ่ายไฟ 5V ภายนอก (2A ขึ้นไป) ห้ามดึงจากขา 3.3V/5V ของบอร์ด
 *   - สายกราวด์ (น้ำตาล/ดำ)   -> ต่อเข้า GND ของแหล่งจ่าย 5V และต่อร่วม (Common GND) กับ GND ของ ESP32
 * @usage
 *   1. ติดตั้งไลบรารี "ESP32Servo" ผ่าน Arduino IDE Library Manager
 *   2. อัปโหลดลงบอร์ด ESP32 แล้วเปิด Serial Monitor ที่ 115200 baud
 *   3. ระบบจะเริ่มทดสอบอัตโนมัติ (หมุนทีละตัว -> หมุนพร้อมกัน 4 ตัว)
 *   4. สามารถพิมพ์คำสั่งผ่าน Serial Monitor เพื่อควบคุมเองได้ตลอดเวลา:
 *      - พิมพ์ "1 90" (หรือ 2/3/4 ตามด้วยองศา 0-180) : สั่งหมุน Servo ตัวที่เลือก (หยุดโหมดอัตโนมัติชั่วคราว)
 *      - พิมพ์ "home" : สั่งทุกตัวกลับไปที่ 90 องศา (กึ่งกลาง)
 *      - พิมพ์ "stop" : หยุดการทดสอบอัตโนมัติ
 *      - พิมพ์ "auto" : กลับมาเริ่มการทดสอบอัตโนมัติ
 * =============================================================================
 */

#include <ESP32Servo.h>

// -----------------------------------------------------------------------------
// SECTION 01: กำหนดขา Pin และพารามิเตอร์ของ Servo
// -----------------------------------------------------------------------------
const int PIN_MG90S_1 = 13; // Servo 1: MG90S #1 (Base)
const int PIN_MG90S_2 = 12; // Servo 2: MG90S #2 (Shoulder) - หมายเหตุ: GPIO 12 เป็น Strapping Pin (ทางเลือก: 26)
const int PIN_SG90_1  = 14; // Servo 3: SG90  #1 (Elbow)
const int PIN_SG90_2  = 27; // Servo 4: SG90  #2 (Gripper)

// ช่วงพัลส์มาตรฐานสำหรับ MG90S และ SG90 เพื่อให้หมุนได้เต็มช่วง 0-180 องศา
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SERVO_FREQ_HZ = 50;
const unsigned long SERIAL_BAUD = 115200;

Servo servos[4];
const int servoPins[4] = { PIN_MG90S_1, PIN_MG90S_2, PIN_SG90_1, PIN_SG90_2 };
const char* servoNames[4] = {
  "Servo 1 (MG90S #1 - Base     | GPIO 13)",
  "Servo 2 (MG90S #2 - Shoulder | GPIO 12)",
  "Servo 3 (SG90  #1 - Elbow    | GPIO 14)",
  "Servo 4 (SG90  #2 - Gripper  | GPIO 27)"
};

int currentAngle[4] = { 90, 90, 90, 90 };
bool autoTestEnabled = true;

// -----------------------------------------------------------------------------
// SECTION 02: ฟังก์ชันช่วยรับคำสั่งจาก Serial Monitor และควบคุมการเคลื่อนที่
// -----------------------------------------------------------------------------

void printHelp() {
  Serial.println("------------------------------------------------------------");
  Serial.println("คำสั่ง Serial Monitor (115200 baud):");
  Serial.println("  <1-4> <0-180>  เช่น '1 45' เพื่อสั่งหมุน Servo ตัวที่ 1 ไป 45 องศา");
  Serial.println("  home           สั่ง Servo ทุกตัวไปที่ 90 องศา (กึ่งกลาง)");
  Serial.println("  stop           หยุดโหมดทดสอบอัตโนมัติ");
  Serial.println("  auto           เปิดโหมดทดสอบอัตโนมัติอีกครั้ง");
  Serial.println("------------------------------------------------------------");
}

void setServoAngle(int index, int angle) {
  if (index < 0 || index >= 4) return;
  angle = constrain(angle, 0, 180);
  servos[index].write(angle);
  currentAngle[index] = angle;
}

void setAllServos(int angle) {
  angle = constrain(angle, 0, 180);
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, angle);
  }
}

/**
 * @brief ตรวจสอบคำสั่งจาก Serial Monitor แบบไม่บล็อกการทำงาน
 * @return true หากมีการสั่งเปลี่ยนโหมดที่ควรหยุดสเต็ปอัตโนมัติปัจจุบันทันที
 */
bool checkSerialCommand() {
  if (Serial.available() <= 0) return false;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return false;

  if (cmd == "stop") {
    autoTestEnabled = false;
    Serial.println("[CMD] หยุดโหมดทดสอบอัตโนมัติแล้ว (พิมพ์ 'auto' เพื่อเริ่มใหม่)");
    return true;
  }

  if (cmd == "auto") {
    autoTestEnabled = true;
    Serial.println("[CMD] เปิดโหมดทดสอบอัตโนมัติ");
    return false;
  }

  if (cmd == "home") {
    autoTestEnabled = false;
    setAllServos(90);
    Serial.println("[CMD] ตั้ง Servo ทุกตัวไปที่ 90 องศา (Home) และหยุดโหมดอัตโนมัติ");
    return true;
  }

  if (cmd == "help") {
    printHelp();
    return false;
  }

  // รูปแบบ "<หมายเลข 1-4> <องศา 0-180>"
  int id = 0;
  int deg = 90;
  if (sscanf(cmd.c_str(), "%d %d", &id, &deg) == 2) {
    if (id >= 1 && id <= 4) {
      autoTestEnabled = false;
      deg = constrain(deg, 0, 180);
      setServoAngle(id - 1, deg);
      Serial.print("[CMD] ");
      Serial.print(servoNames[id - 1]);
      Serial.print(" -> ");
      Serial.print(deg);
      Serial.println(" องศา");
      return true;
    }
  }

  Serial.println("[WARN] คำสั่งไม่ถูกต้อง พิมพ์ 'help' เพื่อดูวิธีใช้งาน");
  return false;
}

/**
 * @brief หน่วงเวลาพร้อมเช็กคำสั่ง Serial ไปด้วย เพื่อตอบสนองทันทีเมื่อผู้ใช้พิมพ์คำสั่ง
 */
bool waitWithSerial(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    if (checkSerialCommand() || !autoTestEnabled) {
      return false;
    }
    delay(10);
  }
  return true;
}

/**
 * @brief หมุน Servo ตัวเดียวอย่างนุ่มนวลจากมุมเดิมไปยังมุมเป้าหมาย
 */
bool moveSmooth(int index, int toDeg, int stepDelayMs = 10) {
  toDeg = constrain(toDeg, 0, 180);
  int fromDeg = currentAngle[index];
  int step = (fromDeg < toDeg) ? 1 : -1;

  for (int pos = fromDeg; pos != toDeg; pos += step) {
    setServoAngle(index, pos);
    if (!waitWithSerial(stepDelayMs)) return false;
  }
  setServoAngle(index, toDeg);
  return true;
}

/**
 * @brief ทดสอบ Servo ทีละตัว: 90 -> 0 -> 180 -> 90 องศา
 */
bool testSingleServo(int index) {
  Serial.print(">> กำลังทดสอบ: ");
  Serial.println(servoNames[index]);

  Serial.println("   - ไปที่ 90 องศา (กึ่งกลาง)");
  setServoAngle(index, 90);
  if (!waitWithSerial(500)) return false;

  Serial.println("   - หมุนไป 0 องศา");
  if (!moveSmooth(index, 0, 10)) return false;
  if (!waitWithSerial(400)) return false;

  Serial.println("   - หมุนไป 180 องศา");
  if (!moveSmooth(index, 180, 10)) return false;
  if (!waitWithSerial(400)) return false;

  Serial.println("   - กลับมากึ่งกลาง 90 องศา");
  if (!moveSmooth(index, 90, 10)) return false;
  if (!waitWithSerial(500)) return false;

  return true;
}

// -----------------------------------------------------------------------------
// SECTION 03: ตั้งค่าเริ่มต้น (setup) และลูปทดสอบ (loop)
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);
  delay(1000);

  Serial.println("\n============================================================");
  Serial.println("  ESP32 Servo Basic Test (2x MG90S + 2x SG90)");
  Serial.println("============================================================");
  printHelp();

  // จอง PWM Timer 4 ช่องสำหรับขับ Servo 4 ตัวบน ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(SERVO_FREQ_HZ);
    servos[i].attach(servoPins[i], SERVO_MIN_US, SERVO_MAX_US);
    setServoAngle(i, 90);
    Serial.print("พร้อมใช้งาน: ");
    Serial.println(servoNames[i]);
  }

  Serial.println("\nตั้งท่าเริ่มต้นทุกตัวที่ 90 องศา รอ 1.5 วินาทีก่อนเริ่มทดสอบ...\n");
  delay(1500);
}

void loop() {
  checkSerialCommand();

  if (!autoTestEnabled) {
    delay(20);
    return;
  }

  // ขั้นตอนที่ 1: ทดสอบหมุนทีละตัว (เช็กสายสัญญาณ ทิศทาง และเฟือง)
  for (int i = 0; i < 4; i++) {
    if (!testSingleServo(i)) return;
  }

  // ขั้นตอนที่ 2: ทดสอบหมุนพร้อมกันทั้ง 4 ตัว (เช็กเสถียรภาพของแหล่งจ่ายไฟ 5V)
  Serial.println(">> ทดสอบหมุนพร้อมกันทั้ง 4 ตัว (90 -> 45 -> 135 -> 90 องศา)");
  for (int pos = 90; pos >= 45; pos--) {
    setAllServos(pos);
    if (!waitWithSerial(15)) return;
  }
  if (!waitWithSerial(400)) return;

  for (int pos = 45; pos <= 135; pos++) {
    setAllServos(pos);
    if (!waitWithSerial(15)) return;
  }
  if (!waitWithSerial(400)) return;

  for (int pos = 135; pos >= 90; pos--) {
    setAllServos(pos);
    if (!waitWithSerial(15)) return;
  }

  Serial.println("=== จบรอบการทดสอบ รอ 3 วินาทีก่อนเริ่มรอบถัดไป ===\n");
  waitWithSerial(3000);
}
