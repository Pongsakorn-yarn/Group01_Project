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

// ขอบเขตองศาปลอดภัยของแต่ละแกน (ป้องกันแขนชนโครงหรือเฟือง SG90 รูด)
const int MIN_SERVO_1 = 0;   const int MAX_SERVO_1 = 180; // Base Yaw (-90°..+90°)
const int MIN_SERVO_2 = 45;  const int MAX_SERVO_2 = 155; // Shoulder (-45°..+65°)
const int MIN_SERVO_3 = 35;  const int MAX_SERVO_3 = 145; // Elbow (-55°..+55°)
const int MIN_SERVO_4 = 20;  const int MAX_SERVO_4 = 120; // Gripper Safe Range (แมปจากคำสั่ง 0..180)

// ออบเจกต์ควบคุมมอเตอร์ Servo ทั้ง 4 แกนอิสระ (4-DOF)
Servo servo1_BaseYaw;
Servo servo2_Shoulder;
Servo servo3_Elbow;
Servo servo4_Gripper;

// บัฟเฟอร์รับข้อมูล Serial แบบขนาดคงที่ (ไม่ใช้ String เพื่อลด Heap Fragmentation และไม่บล็อกลูป)
const size_t RX_BUF_SIZE = 32;
char rxBuf[RX_BUF_SIZE];
size_t rxLen = 0;

// -----------------------------------------------------------------------------
// SECTION 03: ฟังก์ชันช่วยคำนวณและสั่งขับมอเตอร์อย่างปลอดภัย
// -----------------------------------------------------------------------------

/**
 * @brief แปลงคำสั่งองศา (0..180) ด้วยการกลับทิศ, ชดเชยศูนย์ และจำกัดช่วงปลอดภัย
 */
int computeSafeAngle(int rawDeg, bool invert, int offset, int minDeg, int maxDeg) {
  int deg = constrain(rawDeg, 0, 180);
  if (invert) {
    deg = 180 - deg;
  }
  deg += offset;
  return constrain(deg, minDeg, maxDeg);
}

/**
 * @brief สั่งขับ Servo ทั้ง 4 ตัวจากค่าคำสั่งดิบ (0..180)
 *        หมายเหตุ: Servo 4 (Gripper) จะถูก map จาก 0..180 เข้าสู่ช่วงปลอดภัย MIN_SERVO_4..MAX_SERVO_4
 *        เพื่อให้สไลเดอร์ 0..100% ใช้งานได้เต็มช่วงโดยไม่ชนกลไกหัวคีบ
 */
void writeAllServos(int s1, int s2, int s3, int s4) {
  int a1 = computeSafeAngle(s1, INVERT_SERVO_1, OFFSET_SERVO_1, MIN_SERVO_1, MAX_SERVO_1);
  int a2 = computeSafeAngle(s2, INVERT_SERVO_2, OFFSET_SERVO_2, MIN_SERVO_2, MAX_SERVO_2);
  int a3 = computeSafeAngle(s3, INVERT_SERVO_3, OFFSET_SERVO_3, MIN_SERVO_3, MAX_SERVO_3);

  int s4Clamped = constrain(s4, 0, 180);
  if (INVERT_SERVO_4) {
    s4Clamped = 180 - s4Clamped;
  }
  int a4 = map(s4Clamped, 0, 180, MIN_SERVO_4, MAX_SERVO_4) + OFFSET_SERVO_4;
  a4 = constrain(a4, MIN_SERVO_4, MAX_SERVO_4);

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
        int s1, s2, s3, s4;
        if (sscanf(rxBuf, "%d,%d,%d,%d", &s1, &s2, &s3, &s4) == 4) {
          writeAllServos(s1, s2, s3, s4);
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
