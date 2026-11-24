/*
 * ISY493_IOT - Group 01 Project
 * 4-DOF Robotic Arm WebSerial Bridge for Arduino / ESP32
 * Receives: "S1,S2,S3,S4\n" (angles 0..180 degrees) at 115200 baud
 * from the Web 3D Digital Twin Hand-Tracking App.
 */

#ifdef ESP32
  #include <ESP32Servo.h>
  const int PIN_SERVO_1 = 13;
  const int PIN_SERVO_2 = 12;
  const int PIN_SERVO_3 = 14;
  const int PIN_SERVO_4 = 27;
#else
  #include <Servo.h>
  const int PIN_SERVO_1 = 3;
  const int PIN_SERVO_2 = 5;
  const int PIN_SERVO_3 = 6;
  const int PIN_SERVO_4 = 9;
#endif

Servo servo1_BaseYaw;
Servo servo2_Shoulder;
Servo servo3_Elbow;
Servo servo4_Gripper;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  servo1_BaseYaw.attach(PIN_SERVO_1);
  servo2_Shoulder.attach(PIN_SERVO_2);
  servo3_Elbow.attach(PIN_SERVO_3);
  servo4_Gripper.attach(PIN_SERVO_4);

  // Home Pose
  servo1_BaseYaw.write(90);
  servo2_Shoulder.write(90);
  servo3_Elbow.write(90);
  servo4_Gripper.write(60);
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int s1, s2, s3, s4;
    if (sscanf(line.c_str(), "%d,%d,%d,%d", &s1, &s2, &s3, &s4) == 4) {
      servo1_BaseYaw.write(constrain(s1, 0, 180));
      servo2_Shoulder.write(constrain(s2, 0, 180));
      servo3_Elbow.write(constrain(s3, 0, 180));
      servo4_Gripper.write(constrain(s4, 0, 180));
    }
  }
}
