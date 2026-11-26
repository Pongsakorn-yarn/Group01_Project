/**
 * =============================================================================
 * @file        arduino_servo_bridge/arduino_servo_bridge.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description เฟิร์มแวร์ตัวกลางรับคำสั่งผ่าน USB Serial เพื่อขับ Servo 4 แกน (Arduino / ESP32)
 * -----------------------------------------------------------------------------
 * @protocol
 *   - การสื่อสาร  : USB Serial (UART) ความเร็ว 115200 baud, 8N1
 *   - รูปแบบข้อมูล : ข้อความ ASCII CSV ปิดท้ายด้วยขึ้นบรรทัดใหม่ -> "S1,S2,S3,S4\n"
 *   - ช่วงค่าองศา  : จำนวนเต็ม 0..180 องศาต่อแกน (จำกัดช่วงความปลอดภัยทางกลไกในโค้ด)
 * @pinout
 *   - Servo 1 (Base Yaw / หมุนฐาน)        : Arduino D3  | ESP32 GPIO 13
 *   - Servo 2 (Shoulder Pitch / ไหล่)     : Arduino D5  | ESP32 GPIO 26 (เลี่ยง GPIO 12 Strapping Pin)
 *   - Servo 3 (Elbow Pitch / ศอก 4-Bar)   : Arduino D6  | ESP32 GPIO 14
 *   - Servo 4 (Gripper Claws / หัวคีบ)    : Arduino D9  | ESP32 GPIO 27
 * @wiring_note
 *   - แนะนำให้ใช้แหล่งจ่ายไฟ 5V ภายนอก (2A ขึ้นไป) เลี้ยงมอเตอร์ Servo ทั้ง 4 ตัว
 *     และต่อสายกราวด์ร่วม (Common GND) กับบอร์ด Arduino / ESP32 เพื่อป้องกันบอร์ดรีเซ็ต
 * @data_flow
 *   [เว็บ Digital Twin (js/app.js ส่งผ่าน WebSerial API ทุก ~40 ms / 25 Hz)]
 *     -> สาย USB Serial ("S1,S2,S3,S4\n")
 *     -> loop(): Non-blocking char buffer reader -> sscanf()
 *     -> ปรับทิศทาง/ชดเชยศูนย์/จำกัดช่วงปลอดภัย -> Servo.write() ขับมอเตอร์ Servo 4 ตัว
 * =============================================================================
 */

// -----------------------------------------------------------------------------
// SECTION 01: เลือกไลบรารีและขา Pin อัตโนมัติตามชนิดบอร์ด (ESP32 หรือ Arduino)
// -----------------------------------------------------------------------------
#ifdef ESP32
  #include <ESP32Servo.h>
  const int PIN_SERVO_1 = 13; // แกนฐาน (Base Yaw)
  const int PIN_SERVO_2 = 26; // แกนไหล่ (Shoulder Pitch — ย้ายจาก GPIO 12 เพื่อเลี่ยง Strapping Pin)
  const int PIN_SERVO_3 = 14; // แกนศอก (Elbow Pitch กลไก 4-Bar)
  const int PIN_SERVO_4 = 27; // แกนหัวคีบ (Gripper Claws)
  const int SERVO_FREQ_HZ = 50;
  const int SERVO_MIN_US  = 500;
  const int SERVO_MAX_US  = 2400;
#else
  #include <Servo.h>
  const int PIN_SERVO_1 = 3;  // แกนฐาน (Base Yaw)
  const int PIN_SERVO_2 = 5;  // แกนไหล่ (Shoulder Pitch)
  const int PIN_SERVO_3 = 6;  // แกนศอก (Elbow Pitch กลไก 4-Bar)
  const int PIN_SERVO_4 = 9;  // แกนหัวคีบ (Gripper Claws)
#endif

// ค่าคงที่การสื่อสารและท่าเริ่มต้นมาตรฐาน (Home Pose ตรงกับ HOME_POSE ใน js/app.js)
const unsigned long SERIAL_BAUD = 115200;
const int HOME_CMD_1 = 90; // j1 = 0°   -> 90°
const int HOME_CMD_2 = 90; // j2 = 0°   -> 90°
const int HOME_CMD_3 = 90; // j3 = 0°   -> 90°
const int HOME_CMD_4 = 63; // j4 = 35%  -> round(0.35 * 180) = 63°

// -----------------------------------------------------------------------------
// SECTION 02: ค่าปรับเทียบและขอบเขตความปลอดภัยทางกลไก (Calibration & Safety Limits)
// ปรับค่า INVERT (true/false), OFFSET (องศา) และ MIN/MAX ให้เข้ากับแขนกลที่ประกอบจริง
// -----------------------------------------------------------------------------
const bool INVERT_SERVO_1 = false;
const bool INVERT_SERVO_2 = false;
const bool INVERT_SERVO_3 = false;
const bool INVERT_SERVO_4 = false;

const int OFFSET_SERVO_1 = 0;
const int OFFSET_SERVO_2 = 0;
const int OFFSET_SERVO_3 = 0;
const int OFFSET_SERVO_4 = 0;

// ขอบเขตองศาปลอดภัยขั้นเด็ดขาดทางกลไก (Hard Mechanical Safety Limits — ห้ามเกินเด็ดขาด)
const int HARD_MIN_SERVO_1 = 0;   const int HARD_MAX_SERVO_1 = 180; // Base Yaw (-90°..+90°)
const int HARD_MIN_SERVO_2 = 45;  const int HARD_MAX_SERVO_2 = 155; // Shoulder (-45°..+65°)
const int HARD_MIN_SERVO_3 = 35;  const int HARD_MAX_SERVO_3 = 145; // Elbow (-55°..+55°)
const int HARD_MIN_SERVO_4 = 0;   const int HARD_MAX_SERVO_4 = 180; // Gripper Command (0..180)
const int GRIPPER_HW_MIN   = 20;  const int GRIPPER_HW_MAX   = 120; // ช่วงองศาหัวคีบจริงบนฮาร์ดแวร์

// ขอบเขตความปลอดภัยแบบปรับได้จากหน้าเว็บผ่านคำสั่ง "LIMIT:min1,max1,min2,max2,min3,max3,min4,max4\n"
int softMinServo1 = HARD_MIN_SERVO_1; int softMaxServo1 = HARD_MAX_SERVO_1;
int softMinServo2 = HARD_MIN_SERVO_2; int softMaxServo2 = HARD_MAX_SERVO_2;
int softMinServo3 = HARD_MIN_SERVO_3; int softMaxServo3 = HARD_MAX_SERVO_3;
int softMinServo4 = HARD_MIN_SERVO_4; int softMaxServo4 = HARD_MAX_SERVO_4;

// ออบเจกต์ควบคุมมอเตอร์ Servo ทั้ง 4 แกนอิสระ (4-DOF)
Servo servo1_BaseYaw;
Servo servo2_Shoulder;
Servo servo3_Elbow;
Servo servo4_Gripper;

// บัฟเฟอร์รับข้อมูล Serial แบบขนาดคงที่ (64 ไบต์ รองรับทั้ง "S1,S2,S3,S4" และ "LIMIT:..." โดยไม่บล็อกลูป)
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
 * @brief แปลงคำสั่งองศา (0..180) ด้วยการกลับทิศ, ชดเชยศูนย์ และจำกัดช่วงปลอดภัย (Soft ∩ Hard)
 */
int computeSafeAngle(int rawDeg, bool invert, int offset, int softMin, int softMax, int hardMin, int hardMax) {
  int deg = constrain(rawDeg, softMin, softMax);
  if (invert) {
    deg = 180 - deg;
  }
  deg += offset;
  int effMin = max(hardMin, softMin);
  int effMax = min(hardMax, softMax);
  return constrain(deg, effMin, effMax);
}

/**
 * @brief สั่งขับ Servo ทั้ง 4 ตัวจากค่าคำสั่งดิบ (0..180)
 *        หมายเหตุ: Servo 4 (Gripper) จะถูกจำกัดช่วงด้วย softMinServo4..softMaxServo4
 *        ก่อนแมปเข้าช่วงปลอดภัยฮาร์ดแวร์ GRIPPER_HW_MIN..GRIPPER_HW_MAX
 */
void writeAllServos(int s1, int s2, int s3, int s4) {
  int a1 = computeSafeAngle(s1, INVERT_SERVO_1, OFFSET_SERVO_1, softMinServo1, softMaxServo1, HARD_MIN_SERVO_1, HARD_MAX_SERVO_1);
  int a2 = computeSafeAngle(s2, INVERT_SERVO_2, OFFSET_SERVO_2, softMinServo2, softMaxServo2, HARD_MIN_SERVO_2, HARD_MAX_SERVO_2);
  int a3 = computeSafeAngle(s3, INVERT_SERVO_3, OFFSET_SERVO_3, softMinServo3, softMaxServo3, HARD_MIN_SERVO_3, HARD_MAX_SERVO_3);

  int s4Clamped = constrain(s4, softMinServo4, softMaxServo4);
  s4Clamped = constrain(s4Clamped, HARD_MIN_SERVO_4, HARD_MAX_SERVO_4);
  if (INVERT_SERVO_4) {
    s4Clamped = 180 - s4Clamped;
  }
  int a4 = map(s4Clamped, 0, 180, GRIPPER_HW_MIN, GRIPPER_HW_MAX) + OFFSET_SERVO_4;
  a4 = constrain(a4, GRIPPER_HW_MIN, GRIPPER_HW_MAX);

  servo1_BaseYaw.write(a1);
  servo2_Shoulder.write(a2);
  servo3_Elbow.write(a3);
  servo4_Gripper.write(a4);
}

// -----------------------------------------------------------------------------
// SECTION 04: ตั้งค่าเริ่มต้นของระบบ (setup)
// -----------------------------------------------------------------------------

/**
 * @brief เปิดการสื่อสาร USB Serial, จอง PWM Timer (สำหรับ ESP32), ผูกขา PWM ของ Servo ทั้ง 4 ช่อง
 *        และเคลื่อนที่ไปยังท่าเริ่มต้น (Home Pose) อย่างปลอดภัย
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

#ifdef ESP32
  // จอง PWM Timer 4 ช่องและกำหนดพัลส์ 500..2400 us ที่ 50 Hz เพื่อให้หมุนได้เต็มช่วง 0..180°
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1_BaseYaw.setPeriodHertz(SERVO_FREQ_HZ);
  servo2_Shoulder.setPeriodHertz(SERVO_FREQ_HZ);
  servo3_Elbow.setPeriodHertz(SERVO_FREQ_HZ);
  servo4_Gripper.setPeriodHertz(SERVO_FREQ_HZ);

  servo1_BaseYaw.attach(PIN_SERVO_1, SERVO_MIN_US, SERVO_MAX_US);
  servo2_Shoulder.attach(PIN_SERVO_2, SERVO_MIN_US, SERVO_MAX_US);
  servo3_Elbow.attach(PIN_SERVO_3, SERVO_MIN_US, SERVO_MAX_US);
  servo4_Gripper.attach(PIN_SERVO_4, SERVO_MIN_US, SERVO_MAX_US);
#else
  servo1_BaseYaw.attach(PIN_SERVO_1);
  servo2_Shoulder.attach(PIN_SERVO_2);
  servo3_Elbow.attach(PIN_SERVO_3);
  servo4_Gripper.attach(PIN_SERVO_4);
#endif

  writeAllServos(HOME_CMD_1, HOME_CMD_2, HOME_CMD_3, HOME_CMD_4);
  Serial.println("READY:4DOF_BRIDGE");
}

// -----------------------------------------------------------------------------
// SECTION 05: ลูปรับคำสั่งและขับมอเตอร์แบบ Real-Time (Non-Blocking loop)
// -----------------------------------------------------------------------------

/**
 * @brief อ่านไบต์จาก USB Serial แบบไม่บล็อกจนครบหนึ่งบรรทัด ("S1,S2,S3,S4\n")
 *        แยกตัวเลข 4 แกน ตรวจสอบความปลอดภัย และสั่งขับ Servo ทั้ง 4 ตัวทันที
 */
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue; // ข้าม Carriage Return
    }

    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) {
        if (strncmp(rxBuf, "LIMIT:", 6) == 0) {
          int mn1, mx1, mn2, mx2, mn3, mx3, mn4, mx4;
          if (sscanf(rxBuf + 6, "%d,%d,%d,%d,%d,%d,%d,%d",
                     &mn1, &mx1, &mn2, &mx2, &mn3, &mx3, &mn4, &mx4) == 8) {
            applySoftLimitPair(mn1, mx1, HARD_MIN_SERVO_1, HARD_MAX_SERVO_1, softMinServo1, softMaxServo1);
            applySoftLimitPair(mn2, mx2, HARD_MIN_SERVO_2, HARD_MAX_SERVO_2, softMinServo2, softMaxServo2);
            applySoftLimitPair(mn3, mx3, HARD_MIN_SERVO_3, HARD_MAX_SERVO_3, softMinServo3, softMaxServo3);
            applySoftLimitPair(mn4, mx4, HARD_MIN_SERVO_4, HARD_MAX_SERVO_4, softMinServo4, softMaxServo4);
            Serial.println("OK:LIMITS");
          }
        } else {
          int s1, s2, s3, s4;
          if (sscanf(rxBuf, "%d,%d,%d,%d", &s1, &s2, &s3, &s4) == 4) {
            writeAllServos(s1, s2, s3, s4);
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
