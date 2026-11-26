/**
 * =============================================================================
 * @file        servo_test_esp32/servo_test_esp32.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @board       GOOUUU ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8)
 * @description เฟิร์มแวร์สำหรับทดสอบการทำงานเบื้องต้นของ Servo Motor จริง 3 ตัว
 *              (MG996R จำนวน 1 ตัวที่ฐาน + MG90S จำนวน 2 ตัวที่มือหนีบซ้าย/ขวา)
 *              ก่อนประกอบเข้ากับโครงแขนกลและเชื่อมต่อ Dashboard
 * -----------------------------------------------------------------------------
 * @pinout (ตรงกับ arduino_servo_bridge.ino สำหรับ GOOUUU ESP32-S3-CAM)
 *   - Servo 1 (MG996R  — Base Yaw / หมุนฐาน)             : GPIO 1  [LEDC CH 0]
 *   - Servo 2 (MG90S#1 — Gripper Left Claw / ก้ามหนีบซ้าย) : GPIO 2  [LEDC CH 1]
 *   - Servo 3 (MG90S#2 — Gripper Right Claw / ก้ามหนีบขวา) : GPIO 14 [LEDC CH 2]
 * @pwm_architecture
 *   - ใช้ Native ESP32 Hardware LEDC (50 Hz, 14-bit) ล็อก Channel แยกอิสระ 0/1/2
 *     พร้อม gpio_reset_pin() เลี่ยงปัญหาขา JTAG (GPIO 41) และปัญหา ESP32Servo ชน Channel 0
 *   - รองรับทั้ง Arduino-ESP32 Core v3.x และ v2.x โดยไม่ต้องติดตั้งไลบรารี ESP32Servo
 * @wiring_note
 *   - สายสัญญาณ (ส้ม/เหลือง) -> ขา GPIO 1, GPIO 2, GPIO 14 บนบอร์ด GOOUUU ESP32-S3-CAM
 *   - สายไฟเลี้ยง (แดง)       -> แหล่งจ่ายไฟ 5V ภายนอก (3A–5A ขึ้นไป) ห้ามดึงจากบอร์ด/USB
 *   - สายกราวด์ (น้ำตาล/ดำ)   -> ต่อเข้า GND ของแหล่งจ่าย 5V และต่อร่วม (Common GND) กับบอร์ด
 * @usage
 *   1. ติดตั้งบอร์ด "esp32" by Espressif Systems (v2.x หรือ v3.x) ใน Arduino IDE
 *   2. เลือก Board = "ESP32S3 Dev Module", Flash = "16MB (128Mb)", PSRAM = "OPI PSRAM"
 *   3. เสียบสาย USB-C ช่อง "UART" อัปโหลดแล้วเปิด Serial Monitor ที่ 115200 baud
 *   4. คำสั่งควบคุมผ่าน Serial Monitor:
 *      - "<1-3> <0-180>" เช่น "1 90", "2 45", "3 135" : สั่งหมุน Servo แยกทีละตัว
 *      - "grip <0-100>"  เช่น "grip 0" (หนีบ), "grip 100" (อ้า) : ทดสอบมือหนีบ 2 ข้างพร้อมกัน
 *      - "home" : ตั้งทุกตัวไปที่กึ่งกลาง 90 องศา (สำหรับขันนอต Servo Horn)
 *      - "stop" / "auto" / "help"
 * =============================================================================
 */

#include <driver/gpio.h>

// -----------------------------------------------------------------------------
// SECTION 01: กำหนดขา Pin และพารามิเตอร์ Native LEDC PWM (GOOUUU ESP32-S3-CAM)
// -----------------------------------------------------------------------------
const int PIN_BASE_MG996R   = 1;  // Servo 1: MG996R  — Base Yaw (หมุนฐาน)        [LEDC CH 0]
const int PIN_GRIP_L_MG90S  = 2;  // Servo 2: MG90S#1 — Gripper Left (ก้ามซ้าย)   [LEDC CH 1]
const int PIN_GRIP_R_MG90S  = 14; // Servo 3: MG90S#2 — Gripper Right (ก้ามขวา)  [LEDC CH 2]

// ช่วงพัลส์มาตรฐานสำหรับ MG996R และ MG90S (Native LEDC 50 Hz, 14-bit, 500..2400 us)
const uint32_t SERVO_FREQ_HZ   = 50;
const uint8_t  SERVO_PWM_RES   = 14;
const uint32_t SERVO_PWM_MAX   = (1UL << SERVO_PWM_RES) - 1; // 16383
const uint32_t SERVO_PERIOD_US = 20000UL;
const int      SERVO_MIN_US    = 500;
const int      SERVO_MAX_US    = 2400;
const unsigned long SERIAL_BAUD = 115200;

// ช่วงปลอดภัยสำหรับทดสอบมือหนีบซ้าย/ขวา (กลับทิศฝั่งขวาเพื่อให้หนีบเข้าหากันแบบสมมาตร)
const int GRIP_HW_MIN = 20;
const int GRIP_HW_MAX = 120;

const int servoPins[3] = { PIN_BASE_MG996R, PIN_GRIP_L_MG90S, PIN_GRIP_R_MG90S };
const uint8_t servoChannels[3] = { 0, 1, 2 };
const char* servoNames[3] = {
  "Servo 1 (MG996R  - Base Yaw      | GPIO 1  | CH 0)",
  "Servo 2 (MG90S#1 - Gripper Left  | GPIO 2  | CH 1)",
  "Servo 3 (MG90S#2 - Gripper Right | GPIO 14 | CH 2)"
};

int currentAngle[3] = { 90, 90, 90 };
bool autoTestEnabled = true;

// -----------------------------------------------------------------------------
// SECTION 02: ฟังก์ชันช่วยขับ Native LEDC และรับคำสั่งจาก Serial Monitor
// -----------------------------------------------------------------------------

bool attachServoChannel(int pin, uint8_t channel) {
  gpio_reset_pin((gpio_num_t)pin);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  return ledcAttachChannel(pin, SERVO_FREQ_HZ, SERVO_PWM_RES, channel);
#else
  ledcSetup(channel, SERVO_FREQ_HZ, SERVO_PWM_RES);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void writeServoChannel(uint8_t channel, int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  uint32_t pulseUs = (uint32_t)map(angleDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (uint32_t)(((uint64_t)pulseUs * SERVO_PWM_MAX + (SERVO_PERIOD_US / 2)) / SERVO_PERIOD_US);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void printHelp() {
  Serial.println("------------------------------------------------------------");
  Serial.println("คำสั่ง Serial Monitor (115200 baud) — GOOUUU ESP32-S3-CAM:");
  Serial.println("  <1-3> <0-180>  เช่น '1 45' (Base GPIO1), '2 60' (GripL GPIO2), '3 120' (GripR GPIO14)");
  Serial.println("  grip <0-100>   ทดสอบมือหนีบ 2 ข้างพร้อมกันแบบสมมาตร (0=ปิด, 100=อ้า)");
  Serial.println("  home           สั่ง Servo ทั้ง 3 ตัวไปที่ 90 องศา (กึ่งกลาง)");
  Serial.println("  stop           หยุดโหมดทดสอบอัตโนมัติ");
  Serial.println("  auto           เปิดโหมดทดสอบอัตโนมัติอีกครั้ง");
  Serial.println("------------------------------------------------------------");
}

void setServoAngle(int index, int angle) {
  if (index < 0 || index >= 3) return;
  angle = constrain(angle, 0, 180);
  writeServoChannel(servoChannels[index], angle);
  currentAngle[index] = angle;
}

void setAllServos(int angle) {
  angle = constrain(angle, 0, 180);
  for (int i = 0; i < 3; i++) {
    setServoAngle(i, angle);
  }
}

void setSymmetricGripPercent(int pct) {
  pct = constrain(pct, 0, 100);
  int leftDeg = map(100 - pct, 0, 100, GRIP_HW_MIN, GRIP_HW_MAX);
  int rightDeg = map(pct, 0, 100, GRIP_HW_MIN, GRIP_HW_MAX);
  setServoAngle(1, leftDeg);
  setServoAngle(2, rightDeg);
}

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
    Serial.println("[CMD] ตั้ง Servo ทั้ง 3 ตัวไปที่ 90 องศา (Home) และหยุดโหมดอัตโนมัติ");
    return true;
  }

  if (cmd == "help") {
    printHelp();
    return false;
  }

  // รูปแบบ "grip <0-100>"
  int gripPct = 0;
  if (sscanf(cmd.c_str(), "grip %d", &gripPct) == 1) {
    autoTestEnabled = false;
    gripPct = constrain(gripPct, 0, 100);
    setSymmetricGripPercent(gripPct);
    Serial.print("[CMD] Dual Gripper -> ");
    Serial.print(gripPct);
    Serial.print("% (Left=");
    Serial.print(currentAngle[1]);
    Serial.print("°, Right=");
    Serial.print(currentAngle[2]);
    Serial.println("°)");
    return true;
  }

  // รูปแบบ "<หมายเลข 1-3> <องศา 0-180>"
  int id = 0;
  int deg = 90;
  if (sscanf(cmd.c_str(), "%d %d", &id, &deg) == 2) {
    if (id >= 1 && id <= 3) {
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

bool moveSmooth(int index, int toDeg, int stepDelayMs) {
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

bool testSingleServo(int index) {
  Serial.print(">> กำลังทดสอบ: ");
  Serial.println(servoNames[index]);

  int minTest = (index == 0) ? 30 : GRIP_HW_MIN;
  int maxTest = (index == 0) ? 150 : GRIP_HW_MAX;

  Serial.println("   - ไปที่กึ่งกลาง 90 องศา");
  if (!moveSmooth(index, 90, 10)) return false;
  if (!waitWithSerial(400)) return false;

  Serial.print("   - หมุนไป ");
  Serial.print(minTest);
  Serial.println(" องศา");
  if (!moveSmooth(index, minTest, 12)) return false;
  if (!waitWithSerial(400)) return false;

  Serial.print("   - หมุนไป ");
  Serial.print(maxTest);
  Serial.println(" องศา");
  if (!moveSmooth(index, maxTest, 12)) return false;
  if (!waitWithSerial(400)) return false;

  Serial.println("   - กลับมากึ่งกลาง 90 องศา");
  if (!moveSmooth(index, 90, 10)) return false;
  if (!waitWithSerial(400)) return false;

  return true;
}

// -----------------------------------------------------------------------------
// SECTION 03: ตั้งค่าเริ่มต้น (setup) และลูปทดสอบ (loop)
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);
  delay(800);

  Serial.println("\n============================================================");
  Serial.println("  GOOUUU ESP32-S3-CAM Servo Test (Native LEDC CH0/CH1/CH2)");
  Serial.println("============================================================");
  printHelp();

  for (int i = 0; i < 3; i++) {
    bool ok = attachServoChannel(servoPins[i], servoChannels[i]);
    setServoAngle(i, 90);
    Serial.print(ok ? "พร้อมใช้งาน: " : "[ERROR] ผูกขาไม่สำเร็จ: ");
    Serial.println(servoNames[i]);
  }

  Serial.println("\nตั้งท่าเริ่มต้นทั้ง 3 ตัวที่ 90 องศา รอ 1.5 วินาทีก่อนเริ่มทดสอบ...\n");
  delay(1500);
}

void loop() {
  checkSerialCommand();

  if (!autoTestEnabled) {
    delay(20);
    return;
  }

  // ขั้นตอนที่ 1: ทดสอบหมุนทีละตัว (Base MG996R -> Gripper Left -> Gripper Right)
  for (int i = 0; i < 3; i++) {
    if (!testSingleServo(i)) return;
  }

  // ขั้นตอนที่ 2: ทดสอบมือหนีบ 2 ข้างทำงานสมมาตรพร้อมกัน (0% หนีบ -> 100% อ้า -> 35% Home)
  Serial.println(">> ทดสอบมือหนีบคู่ซ้าย-ขวาสมมาตรพร้อมกัน (0% -> 100% -> 35%)");
  for (int pct = 0; pct <= 100; pct += 2) {
    setSymmetricGripPercent(pct);
    if (!waitWithSerial(15)) return;
  }
  if (!waitWithSerial(400)) return;

  for (int pct = 100; pct >= 0; pct -= 2) {
    setSymmetricGripPercent(pct);
    if (!waitWithSerial(15)) return;
  }
  if (!waitWithSerial(400)) return;

  setSymmetricGripPercent(35);
  Serial.println("=== จบรอบการทดสอบ รอ 3 วินาทีก่อนเริ่มรอบถัดไป ===\n");
  waitWithSerial(3000);
}
