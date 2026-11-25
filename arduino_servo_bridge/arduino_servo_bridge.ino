/**
 * =============================================================================
 * @file        arduino_servo_bridge/arduino_servo_bridge.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description เฟิร์มแวร์ตัวกลางรับคำสั่งผ่าน USB Serial เพื่อขับ Servo 4 แกน (Arduino / ESP32)
 * -----------------------------------------------------------------------------
 * @protocol
 *   - การสื่อสาร  : USB Serial (UART) ความเร็ว 115200 baud, 8N1
 *   - รูปแบบข้อมูล : ข้อความ ASCII CSV ปิดท้ายด้วยขึ้นบรรทัดใหม่ -> "S1,S2,S3,S4\n"
 *   - ช่วงค่าองศา  : จำนวนเต็ม 0..180 องศาต่อแกน (จำกัดช่วงความปลอดภัยด้วย constrain)
 * @pinout
 *   - Servo 1 (Base Yaw / หมุนฐาน)        : Arduino D3  | ESP32 GPIO 13
 *   - Servo 2 (Shoulder Pitch / ไหล่)     : Arduino D5  | ESP32 GPIO 12
 *   - Servo 3 (Elbow Pitch / ศอก 4-Bar)   : Arduino D6  | ESP32 GPIO 14
 *   - Servo 4 (Gripper Claws / หัวคีบ)    : Arduino D9  | ESP32 GPIO 27
 * @wiring_note
 *   - แนะนำให้ใช้แหล่งจ่ายไฟ 5V ภายนอก (2A ขึ้นไป) เลี้ยงมอเตอร์ Servo ทั้ง 4 ตัว
 *     และต่อสายกราวด์ร่วม (Common GND) กับบอร์ด Arduino / ESP32 เพื่อป้องกันบอร์ดรีเซ็ต
 * @data_flow
 *   [เว็บ Digital Twin (js/app.js ส่งผ่าน WebSerial API ทุก ~40 ms / 25 Hz)]
 *     -> สาย USB Serial ("S1,S2,S3,S4\n")
 *     -> loop(): Serial.readStringUntil('\n') -> sscanf() -> constrain(0, 180)
 *     -> Servo.write() สร้างสัญญาณ PWM -> ขับมอเตอร์ Servo จริงทั้ง 4 ตัว
 * =============================================================================
 */

// -----------------------------------------------------------------------------
// SECTION 01: เลือกไลบรารีและขา Pin อัตโนมัติตามชนิดบอร์ด (ESP32 หรือ Arduino)
// -----------------------------------------------------------------------------
#ifdef ESP32
  #include <ESP32Servo.h>
  const int PIN_SERVO_1 = 13; // แกนฐาน (Base Yaw)
  const int PIN_SERVO_2 = 12; // แกนไหล่ (Shoulder Pitch)
  const int PIN_SERVO_3 = 14; // แกนศอก (Elbow Pitch กลไก 4-Bar)
  const int PIN_SERVO_4 = 27; // แกนหัวคีบ (Gripper Claws)
#else
  #include <Servo.h>
  const int PIN_SERVO_1 = 3;  // แกนฐาน (Base Yaw)
  const int PIN_SERVO_2 = 5;  // แกนไหล่ (Shoulder Pitch)
  const int PIN_SERVO_3 = 6;  // แกนศอก (Elbow Pitch กลไก 4-Bar)
  const int PIN_SERVO_4 = 9;  // แกนหัวคีบ (Gripper Claws)
#endif

// ค่าคงที่การสื่อสารและท่าเริ่มต้นมาตรฐาน (Home Pose)
const unsigned long SERIAL_BAUD = 115200;
const int HOME_SERVO_1 = 90; // ฐานตรงกลาง
const int HOME_SERVO_2 = 90; // ไหล่ท่ากลาง
const int HOME_SERVO_3 = 90; // ศอกท่ากลาง
const int HOME_SERVO_4 = 60; // หัวคีบอ้าเล็กน้อย (~35%)

// ออบเจกต์ควบคุมมอเตอร์ Servo ทั้ง 4 แกนอิสระ (4-DOF)
Servo servo1_BaseYaw;
Servo servo2_Shoulder;
Servo servo3_Elbow;
Servo servo4_Gripper;

// -----------------------------------------------------------------------------
// SECTION 02: ตั้งค่าเริ่มต้นของระบบ (setup)
// -----------------------------------------------------------------------------

/**
 * @brief เปิดการสื่อสาร USB Serial, ผูกขา PWM ของ Servo ทั้ง 4 ช่อง
 *        และเคลื่อนที่ไปยังท่าเริ่มต้น (Home Pose) อย่างปลอดภัย
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50); // ตั้ง Timeout สั้นเพื่อไม่ให้ลูปค้างระหว่างรอแพ็กเก็ต

  servo1_BaseYaw.attach(PIN_SERVO_1);
  servo2_Shoulder.attach(PIN_SERVO_2);
  servo3_Elbow.attach(PIN_SERVO_3);
  servo4_Gripper.attach(PIN_SERVO_4);

  servo1_BaseYaw.write(HOME_SERVO_1);
  servo2_Shoulder.write(HOME_SERVO_2);
  servo3_Elbow.write(HOME_SERVO_3);
  servo4_Gripper.write(HOME_SERVO_4);
}

// -----------------------------------------------------------------------------
// SECTION 03: ลูปรับคำสั่งและขับมอเตอร์แบบ Real-Time (loop)
// -----------------------------------------------------------------------------

/**
 * @brief อ่านข้อมูลทีละบรรทัดจาก USB Serial แยกค่าองศา 4 แกน ("S1,S2,S3,S4\n")
 *        จำกัดช่วงปลอดภัย [0, 180] และสั่งขับ Servo ทั้ง 4 ตัวทันที
 */
void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int s1, s2, s3, s4;

    // สั่งขับมอเตอร์เฉพาะเมื่อแยกตัวเลขจำนวนเต็มครบทั้ง 4 ค่าอย่างถูกต้อง
    if (sscanf(line.c_str(), "%d,%d,%d,%d", &s1, &s2, &s3, &s4) == 4) {
      servo1_BaseYaw.write(constrain(s1, 0, 180));
      servo2_Shoulder.write(constrain(s2, 0, 180));
      servo3_Elbow.write(constrain(s3, 0, 180));
      servo4_Gripper.write(constrain(s4, 0, 180));
    }
  }
}
