/**
 * =============================================================================
 * @file        arduino_servo_bridge/arduino_servo_bridge.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @board       GOOUUU ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8)
 * @description เฟิร์มแวร์ตัวกลางรับคำสั่งผ่าน USB WebSerial เพื่อขับ Servo จริง 3 ตัว
 *              (1x MG996R ที่ฐาน + 2x MG90S ที่มือหนีบซ้าย/ขวาแบบแยกข้างอิสระ)
 *              ส่วนแกน Shoulder และ Elbow คงการทำงานแบบ Virtual Digital Twin บนเว็บ
 * -----------------------------------------------------------------------------
 * @protocol
 *   - การสื่อสาร    : USB Serial (UART) ความเร็ว 115200 baud, 8N1
 *                     (แนะนำให้เสียบสาย USB-C ที่ช่อง "UART" ของบอร์ด GOOUUU ESP32-S3-CAM)
 *   - แพ็กเก็ตหลัก  : "BASE,GRIP_L,GRIP_R\n" (จำนวนเต็ม 0..180 องศา สำหรับ 3 มอเตอร์จริง)
 *   - ซิงก์ขอบเขต  : "LIMIT:minB,maxB,minGL,maxGL,minGR,maxGR\n" -> ตอบกลับ "OK:LIMITS"
 *   - Handshake     : "PING\n" -> ตอบกลับ "READY:3SERVO_S3CAM" (รองรับ ESP32-S3 ที่ไม่ Auto-Reset)
 * @pinout (GOOUUU ESP32-S3-CAM — เลี่ยงขากล้อง 4-18, PSRAM 26-37, SD 38-40, Strapping 0/3/45/46)
 *   - Servo 1 — Base Yaw (MG996R หมุนฐาน)             : GPIO 1  (Fallback ESP32: 13 | Arduino: D3)
 *   - Servo 2 — Gripper Left Claw (MG90S ก้ามหนีบซ้าย) : GPIO 2  (Fallback ESP32: 26 | Arduino: D5)
 *   - Servo 3 — Gripper Right Claw (MG90S ก้ามหนีบขวา) : GPIO 41 (Fallback ESP32: 14 | Arduino: D6)
 * @wiring_note
 *   - ต้องใช้แหล่งจ่ายไฟภายนอก 5V DC (กระแส 3A–5A ขึ้นไป) เลี้ยง MG996R + 2x MG90S
 *     พร้อมตัวเก็บประจุ 1000uF คร่อมรางไฟ และต่อสายกราวด์ร่วม (Common GND) กับบอร์ดเสมอ
 *   - Arduino IDE Settings: Board = "ESP32S3 Dev Module", Flash = "16MB (128Mb)",
 *     PSRAM = "OPI PSRAM", Library = "ESP32Servo" (v3.0.5+)
 * @data_flow
 *   [เว็บ Digital Twin (js/app.js ส่งผ่าน WebSerial API ทุก ~40 ms / 25 Hz)]
 *     -> สาย USB-C UART ("BASE,GRIP_L,GRIP_R\n")
 *     -> loop(): Non-blocking char buffer reader -> PING / LIMIT / sscanf()
 *     -> ปรับทิศทาง (Invert) / ชดเชยศูนย์ (Offset) / จำกัดช่วงปลอดภัย -> Servo.write() 3 ช่อง
 * =============================================================================
 */

// -----------------------------------------------------------------------------
// SECTION 01: เลือกไลบรารีและขา Pin อัตโนมัติตามชนิดบอร์ด
// -----------------------------------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // บอร์ดหลักของโปรเจกต์: GOOUUU ESP32-S3-CAM (ขาปลอดการชนกับกล้อง, SD, PSRAM, USB, Strapping)
  #include <ESP32Servo.h>
  const int PIN_SERVO_BASE   = 1;  // Servo 1: MG996R  — Base Yaw (หมุนฐาน)
  const int PIN_SERVO_GRIP_L = 2;  // Servo 2: MG90S#1 — Gripper Left Claw (ก้ามหนีบซ้าย)
  const int PIN_SERVO_GRIP_R = 41; // Servo 3: MG90S#2 — Gripper Right Claw (ก้ามหนีบขวา)
  const int SERVO_FREQ_HZ    = 50;
  const int SERVO_MIN_US     = 500;
  const int SERVO_MAX_US     = 2400;
#elif defined(ESP32)
  // Fallback สำหรับ ESP32 Dev Module รุ่นเดิม
  #include <ESP32Servo.h>
  const int PIN_SERVO_BASE   = 13;
  const int PIN_SERVO_GRIP_L = 26;
  const int PIN_SERVO_GRIP_R = 14;
  const int SERVO_FREQ_HZ    = 50;
  const int SERVO_MIN_US     = 500;
  const int SERVO_MAX_US     = 2400;
#else
  // Fallback สำหรับ Arduino UNO / Nano
  #include <Servo.h>
  const int PIN_SERVO_BASE   = 3;
  const int PIN_SERVO_GRIP_L = 5;
  const int PIN_SERVO_GRIP_R = 6;
#endif

// ค่าคงที่การสื่อสารและท่าเริ่มต้นมาตรฐาน (Home Pose ตรงกับ HOME_POSE ใน js/app.js)
const unsigned long SERIAL_BAUD = 115200;
const int HOME_CMD_BASE   = 90; // j1  = 0°  -> 90°
const int HOME_CMD_GRIP_L = 63; // j4L = 35% -> round(0.35 * 180) = 63°
const int HOME_CMD_GRIP_R = 63; // j4R = 35% -> round(0.35 * 180) = 63°

// -----------------------------------------------------------------------------
// SECTION 02: ค่าปรับเทียบและขอบเขตความปลอดภัยทางกลไก (Calibration & Safety Limits)
// ปรับ INVERT (true/false), OFFSET (องศา) และช่วง HW MIN/MAX ให้เข้ากับชุดมือหนีบจริง
// -----------------------------------------------------------------------------
const bool INVERT_SERVO_BASE   = false;
const bool INVERT_SERVO_GRIP_L = false; // ก้ามหนีบซ้าย: 0% ปิด -> 100% อ้า
const bool INVERT_SERVO_GRIP_R = true;  // ก้ามหนีบขวา: กลับทิศให้สมมาตรกับฝั่งซ้าย (ปรับ false ได้หากติดทิศเดียวกัน)

const int OFFSET_SERVO_BASE   = 0;
const int OFFSET_SERVO_GRIP_L = 0;
const int OFFSET_SERVO_GRIP_R = 0;

// ขอบเขตองศาปลอดภัยขั้นเด็ดขาดทางกลไก (Hard Mechanical Safety Limits — ห้ามเกินเด็ดขาด)
const int HARD_MIN_BASE   = 0;  const int HARD_MAX_BASE   = 180; // Base Yaw (-90°..+90°)
const int HARD_MIN_GRIP_L = 0;  const int HARD_MAX_GRIP_L = 180; // Gripper Left Command (0..180)
const int HARD_MIN_GRIP_R = 0;  const int HARD_MAX_GRIP_R = 180; // Gripper Right Command (0..180)

// ช่วงองศาปลอดภัยบนฮาร์ดแวร์จริงของมือหนีบแต่ละข้าง (ป้องกันเฟือง MG90S ชนสุดระยะ)
const int GRIP_L_HW_MIN = 20;  const int GRIP_L_HW_MAX = 120;
const int GRIP_R_HW_MIN = 20;  const int GRIP_R_HW_MAX = 120;

// ขอบเขตความปลอดภัยแบบปรับได้จากหน้าเว็บผ่านคำสั่ง "LIMIT:minB,maxB,minGL,maxGL,minGR,maxGR\n"
int softMinBase  = HARD_MIN_BASE;   int softMaxBase  = HARD_MAX_BASE;
int softMinGripL = HARD_MIN_GRIP_L; int softMaxGripL = HARD_MAX_GRIP_L;
int softMinGripR = HARD_MIN_GRIP_R; int softMaxGripR = HARD_MAX_GRIP_R;

// ออบเจกต์ควบคุมมอเตอร์ Servo จริงทั้ง 3 ตัว
Servo servo1_BaseYaw;   // MG996R
Servo servo2_GripLeft;  // MG90S #1
Servo servo3_GripRight; // MG90S #2

// บัฟเฟอร์รับข้อมูล Serial แบบขนาดคงที่ (64 ไบต์ รองรับ "BASE,GRIP_L,GRIP_R", "LIMIT:...", "PING")
const size_t RX_BUF_SIZE = 64;
char rxBuf[RX_BUF_SIZE];
size_t rxLen = 0;

// -----------------------------------------------------------------------------
// SECTION 03: ฟังก์ชันช่วยคำนวณและสั่งขับมอเตอร์อย่างปลอดภัย
// -----------------------------------------------------------------------------

/**
 * @brief อัปเดตขอบเขตความปลอดภัยแบบซอฟต์แวร์ โดยบังคับให้อยู่ภายใน Hard Mechanical Limits เสมอ
 */
void applySoftLimitPair(int reqMin, int reqMax, int hardMin, int hardMax, int &outMin, int &outMax) {
  int cMin = constrain(reqMin, hardMin, hardMax - 5);
  int cMax = constrain(reqMax, hardMin + 5, hardMax);
  if (cMin > cMax - 5) {
    cMax = constrain(cMin + 5, hardMin + 5, hardMax);
    if (cMin > cMax - 5) {
      cMin = constrain(cMax - 5, hardMin, hardMax - 5);
    }
  }
  outMin = cMin;
  outMax = cMax;
}

/**
 * @brief แปลงคำสั่งองศาของแกนหมุนฐาน (0..180) พร้อมกลับทิศ ชดเชยศูนย์ และจำกัดช่วงปลอดภัย
 */
int computeSafeBaseAngle(int rawDeg) {
  int deg = constrain(rawDeg, softMinBase, softMaxBase);
  if (INVERT_SERVO_BASE) {
    deg = 180 - deg;
  }
  deg += OFFSET_SERVO_BASE;
  int effMin = max(HARD_MIN_BASE, softMinBase);
  int effMax = min(HARD_MAX_BASE, softMaxBase);
  return constrain(deg, effMin, effMax);
}

/**
 * @brief แปลงคำสั่งองศามือหนีบ (0..180) เข้าช่วงปลอดภัยฮาร์ดแวร์ของ MG90S แต่ละข้าง
 */
int computeSafeGripperAngle(int rawCmd, bool invert, int offset,
                            int softMin, int softMax, int hardMin, int hardMax,
                            int hwMin, int hwMax) {
  int clamped = constrain(rawCmd, softMin, softMax);
  clamped = constrain(clamped, hardMin, hardMax);
  if (invert) {
    clamped = 180 - clamped;
  }
  int mapped = map(clamped, 0, 180, hwMin, hwMax) + offset;
  return constrain(mapped, hwMin, hwMax);
}

/**
 * @brief สั่งขับ Servo จริงทั้ง 3 ตัว (Base MG996R, Gripper Left MG90S, Gripper Right MG90S)
 */
void writeAllServos(int baseCmd, int gripLCmd, int gripRCmd) {
  int aBase = computeSafeBaseAngle(baseCmd);
  int aGripL = computeSafeGripperAngle(
    gripLCmd, INVERT_SERVO_GRIP_L, OFFSET_SERVO_GRIP_L,
    softMinGripL, softMaxGripL, HARD_MIN_GRIP_L, HARD_MAX_GRIP_L,
    GRIP_L_HW_MIN, GRIP_L_HW_MAX
  );
  int aGripR = computeSafeGripperAngle(
    gripRCmd, INVERT_SERVO_GRIP_R, OFFSET_SERVO_GRIP_R,
    softMinGripR, softMaxGripR, HARD_MIN_GRIP_R, HARD_MAX_GRIP_R,
    GRIP_R_HW_MIN, GRIP_R_HW_MAX
  );

  servo1_BaseYaw.write(aBase);
  servo2_GripLeft.write(aGripL);
  servo3_GripRight.write(aGripR);
}

// -----------------------------------------------------------------------------
// SECTION 04: ตั้งค่าเริ่มต้นของระบบ (setup)
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300); // รอพอร์ต UART / USB CDC บน ESP32-S3 พร้อมรับ-ส่งข้อมูล

#if defined(ESP32)
  // จอง PWM Timer 3 ช่องสำหรับขับ Servo จริง 3 ตัวที่ความถี่ 50 Hz (500..2400 us)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servo1_BaseYaw.setPeriodHertz(SERVO_FREQ_HZ);
  servo2_GripLeft.setPeriodHertz(SERVO_FREQ_HZ);
  servo3_GripRight.setPeriodHertz(SERVO_FREQ_HZ);

  servo1_BaseYaw.attach(PIN_SERVO_BASE, SERVO_MIN_US, SERVO_MAX_US);
  servo2_GripLeft.attach(PIN_SERVO_GRIP_L, SERVO_MIN_US, SERVO_MAX_US);
  servo3_GripRight.attach(PIN_SERVO_GRIP_R, SERVO_MIN_US, SERVO_MAX_US);
#else
  servo1_BaseYaw.attach(PIN_SERVO_BASE);
  servo2_GripLeft.attach(PIN_SERVO_GRIP_L);
  servo3_GripRight.attach(PIN_SERVO_GRIP_R);
#endif

  writeAllServos(HOME_CMD_BASE, HOME_CMD_GRIP_L, HOME_CMD_GRIP_R);
  Serial.println("READY:3SERVO_S3CAM");
}

// -----------------------------------------------------------------------------
// SECTION 05: ลูปรับคำสั่งและขับมอเตอร์แบบ Real-Time (Non-Blocking loop)
// -----------------------------------------------------------------------------

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue; // ข้าม Carriage Return
    }

    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) {
        if (strcmp(rxBuf, "PING") == 0) {
          // ตอบกลับ Handshake ทันที (สำคัญสำหรับ ESP32-S3 ที่ไม่ Auto-Reset เมื่อเปิด WebSerial)
          Serial.println("READY:3SERVO_S3CAM");
        } else if (strncmp(rxBuf, "LIMIT:", 6) == 0) {
          int mnB, mxB, mnGL, mxGL, mnGR, mxGR, unused1, unused2;
          int parsed = sscanf(rxBuf + 6, "%d,%d,%d,%d,%d,%d,%d,%d",
                              &mnB, &mxB, &mnGL, &mxGL, &mnGR, &mxGR, &unused1, &unused2);
          if (parsed == 6) {
            applySoftLimitPair(mnB, mxB, HARD_MIN_BASE, HARD_MAX_BASE, softMinBase, softMaxBase);
            applySoftLimitPair(mnGL, mxGL, HARD_MIN_GRIP_L, HARD_MAX_GRIP_L, softMinGripL, softMaxGripL);
            applySoftLimitPair(mnGR, mxGR, HARD_MIN_GRIP_R, HARD_MAX_GRIP_R, softMinGripR, softMaxGripR);
            Serial.println("OK:LIMITS");
          } else if (parsed == 8) {
            // รองรับคำสั่ง LIMIT แบบเดิม 8 ค่า (ใช้ Base และ Gripper)
            applySoftLimitPair(mnB, mxB, HARD_MIN_BASE, HARD_MAX_BASE, softMinBase, softMaxBase);
            applySoftLimitPair(unused1, unused2, HARD_MIN_GRIP_L, HARD_MAX_GRIP_L, softMinGripL, softMaxGripL);
            applySoftLimitPair(unused1, unused2, HARD_MIN_GRIP_R, HARD_MAX_GRIP_R, softMinGripR, softMaxGripR);
            Serial.println("OK:LIMITS");
          }
        } else {
          int v1, v2, v3, v4;
          int parsed = sscanf(rxBuf, "%d,%d,%d,%d", &v1, &v2, &v3, &v4);
          if (parsed == 3) {
            // โปรโตคอลมาตรฐาน 3 ช่อง: BASE, GRIP_L, GRIP_R
            writeAllServos(v1, v2, v3);
          } else if (parsed == 4) {
            // รองรับแพ็กเก็ตเดิม 4 ช่อง (Base = v1, GripL/R = v4)
            writeAllServos(v1, v4, v4);
          }
        }
      }
      rxLen = 0;
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // ป้องกันบัฟเฟอร์ล้น: รีเซ็ตและรอขึ้นบรรทัดใหม่ของแพ็กเก็ตถัดไป
      rxLen = 0;
    }
  }
}
