/**
 * =============================================================================
 * @file        arduino_servo_bridge/arduino_servo_bridge.ino
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @board       GOOUUU ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8)
 * @description เฟิร์มแวร์ Dual-Mode รับคำสั่งได้ทั้งผ่าน USB WebSerial และ
 *              EMQX Cloud Serverless (MQTT over TLS :8883) เพื่อขับ Servo จริง 3 ตัว
 *              (1x MG996R ที่ฐาน + 2x MG90S ที่มือหนีบซ้าย/ขวาแบบแยกข้างอิสระ)
 *              พร้อม Smooth Servo Interpolation (50 Hz), LWT (ONLINE/OFFLINE),
 *              Remote E-STOP และ Watchdog Fail-Safe
 * -----------------------------------------------------------------------------
 * @protocol
 *   1) USB Serial (UART) ความเร็ว 115200 baud, 8N1 (เสียบช่อง "UART" ของบอร์ด)
 *      - แพ็กเก็ตหลัก : "BASE,GRIP_L,GRIP_R\n" (0..180 องศา)
 *      - ซิงก์ขอบเขต : "LIMIT:minB,maxB,minGL,maxGL,minGR,maxGR\n" -> "OK:LIMITS"
 *      - Handshake    : "PING\n" -> "READY:3SERVO_S3CAM"
 *      - ตรวจสถานะ    : "STATUS\n" -> แสดง GPIO, องศาจริง, สถานะ Wi-Fi & MQTT
 *   2) Cloud MQTT over TLS (EMQX Cloud Serverless, Port 8883) — เปิดอัตโนมัติเมื่อมี secrets.h
 *      - Subscribe : {PREFIX}/cmd     ("BASE,GRIP_L,GRIP_R")
 *                    {PREFIX}/limit   ("LIMIT:mnB,mxB,mnGL,mxGL,mnGR,mxGR")
 *                    {PREFIX}/estop   ("1"/"0" หรือ JSON {"estop":true})
 *      - Publish   : {PREFIX}/status  ("ONLINE" / LWT "OFFLINE", retain=true)
 *                    {PREFIX}/state   (JSON มุมจริงหลัง Safety Clamp + RSSI + Uptime)
 * @libraries (สำหรับโหมด Cloud MQTT บน Arduino IDE)
 *   - Board Package : esp32 by Espressif Systems (v3.x) -> "ESP32S3 Dev Module"
 *   - Library       : "PubSubClient" by Nick O'Leary (ติดตั้งผ่าน Library Manager)
 *   - Credentials   : คัดลอก secrets.h.example เป็น secrets.h และใส่ข้อมูล Wi-Fi/EMQX/CA Cert
 * @pinout (GOOUUU ESP32-S3-CAM — Native Hardware LEDC CH0/CH1/CH2)
 *   - Servo 1 — Base Yaw (MG996R หมุนฐาน)             : GPIO 1  [LEDC CH 0]
 *   - Servo 2 — Gripper Left Claw (MG90S ก้ามหนีบซ้าย) : GPIO 2  [LEDC CH 1]
 *   - Servo 3 — Gripper Right Claw (MG90S ก้ามหนีบขวา) : GPIO 14 [LEDC CH 2]
 * =============================================================================
 */

// -----------------------------------------------------------------------------
// SECTION 01: กำหนดขา Pin, LEDC PWM และตรวจจับไฟล์ตั้งค่า Cloud MQTT (secrets.h)
// -----------------------------------------------------------------------------
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #include <driver/gpio.h>
  const int PIN_SERVO_BASE   = 1;  // Servo 1: MG996R  — Base Yaw (หมุนฐาน)        [LEDC CH 0]
  const int PIN_SERVO_GRIP_L = 2;  // Servo 2: MG90S#1 — Gripper Left (ก้ามซ้าย)   [LEDC CH 1]
  const int PIN_SERVO_GRIP_R = 14; // Servo 3: MG90S#2 — Gripper Right (ก้ามขวา)  [LEDC CH 2]
#elif defined(ESP32)
  #include <driver/gpio.h>
  const int PIN_SERVO_BASE   = 13;
  const int PIN_SERVO_GRIP_L = 26;
  const int PIN_SERVO_GRIP_R = 14;
#else
  #include <Servo.h>
  const int PIN_SERVO_BASE   = 3;
  const int PIN_SERVO_GRIP_L = 5;
  const int PIN_SERVO_GRIP_R = 6;
  Servo servo1_BaseYaw;
  Servo servo2_GripLeft;
  Servo servo3_GripRight;
#endif

#if defined(ESP32)
  // Native ESP32 Hardware LEDC: 50 Hz, 14-bit (0..16383 -> ความละเอียด ~1.22 us/step)
  const uint32_t SERVO_FREQ_HZ   = 50;
  const uint8_t  SERVO_PWM_RES   = 14;
  const uint32_t SERVO_PWM_MAX   = (1UL << SERVO_PWM_RES) - 1;
  const uint32_t SERVO_PERIOD_US = 20000UL;
  const int      SERVO_MIN_US    = 500;
  const int      SERVO_MAX_US    = 2400;

  const uint8_t CH_SERVO_BASE   = 0;
  const uint8_t CH_SERVO_GRIP_L = 1;
  const uint8_t CH_SERVO_GRIP_R = 2;

  // เปิดโหมด Cloud MQTT อัตโนมัติเมื่อมีไฟล์ secrets.h และตั้ง ENABLE_CLOUD_MQTT = 1
  #if defined(__has_include)
    #if __has_include("secrets.h")
      #include "secrets.h"
      #if defined(ENABLE_CLOUD_MQTT) && (ENABLE_CLOUD_MQTT == 1)
        #define USE_CLOUD_MQTT 1
        #include <WiFi.h>
        #include <WiFiClientSecure.h>
        #include <PubSubClient.h>
      #endif
    #endif
  #endif
#endif

// ค่าคงที่การสื่อสารและท่าเริ่มต้นมาตรฐาน (ตรงกับ HOME_POSE ใน js/app.js)
const unsigned long SERIAL_BAUD = 115200;
const int HOME_CMD_BASE   = 90; // j1  = 0°  -> 90°
const int HOME_CMD_GRIP_L = 63; // j4L = 35% -> round(0.35 * 180) = 63°
const int HOME_CMD_GRIP_R = 63; // j4R = 35% -> round(0.35 * 180) = 63°

// -----------------------------------------------------------------------------
// SECTION 02: ค่าปรับเทียบ, ขอบเขตความปลอดภัย และ Smooth Interpolation
// -----------------------------------------------------------------------------
const bool INVERT_SERVO_BASE   = false;
const bool INVERT_SERVO_GRIP_L = false;
const bool INVERT_SERVO_GRIP_R = true;

const int OFFSET_SERVO_BASE   = 0;
const int OFFSET_SERVO_GRIP_L = 0;
const int OFFSET_SERVO_GRIP_R = 0;

// ขอบเขตองศาปลอดภัยขั้นเด็ดขาดทางกลไก (Hard Mechanical Safety Limits)
const int HARD_MIN_BASE   = 0;  const int HARD_MAX_BASE   = 180;
const int HARD_MIN_GRIP_L = 0;  const int HARD_MAX_GRIP_L = 180;
const int HARD_MIN_GRIP_R = 0;  const int HARD_MAX_GRIP_R = 180;

// ช่วงองศาปลอดภัยบนฮาร์ดแวร์จริงของมือหนีบแต่ละข้าง (ป้องกันเฟือง MG90S ชนสุดระยะ)
const int GRIP_L_HW_MIN = 20;  const int GRIP_L_HW_MAX = 120;
const int GRIP_R_HW_MIN = 20;  const int GRIP_R_HW_MAX = 120;

// ขอบเขตความปลอดภัยแบบซอฟต์แวร์ (ปรับได้จากหน้าเว็บผ่าน LIMIT:...)
int softMinBase  = HARD_MIN_BASE;   int softMaxBase  = HARD_MAX_BASE;
int softMinGripL = HARD_MIN_GRIP_L; int softMaxGripL = HARD_MAX_GRIP_L;
int softMinGripR = HARD_MIN_GRIP_R; int softMaxGripR = HARD_MAX_GRIP_R;

// ค่าเป้าหมายและค่าปัจจุบันสำหรับ Smooth Interpolation (ช่วยให้แขนกลนุ่มนวลแม้คำสั่ง Cloud มาที่ 6-8 Hz)
float curCmdBase  = HOME_CMD_BASE;
float curCmdGripL = HOME_CMD_GRIP_L;
float curCmdGripR = HOME_CMD_GRIP_R;
float tgtCmdBase  = HOME_CMD_BASE;
float tgtCmdGripL = HOME_CMD_GRIP_L;
float tgtCmdGripR = HOME_CMD_GRIP_R;

const unsigned long SERVO_TICK_MS = 20; // อัปเดต Smoothing ทุก 20 ms (50 Hz ตรงคาบ PWM)
const float CLOUD_SMOOTH_ALPHA    = 0.35f; // ค่าความนุ่มนวลเมื่อรับคำสั่งผ่าน Cloud
unsigned long lastServoTickMs     = 0;
bool smoothMotionActive           = false;

// สถานะความปลอดภัยและค่าองศาฮาร์ดแวร์ล่าสุด
bool estopLatched = false;
int lastHwBase    = HOME_CMD_BASE;
int lastHwGripL   = 90;
int lastHwGripR   = 90;

// บัฟเฟอร์รับข้อมูล Serial แบบขนาดคงที่
const size_t RX_BUF_SIZE = 96;
char rxBuf[RX_BUF_SIZE];
size_t rxLen = 0;

#if defined(USE_CLOUD_MQTT)
  WiFiClientSecure tlsClient;
  PubSubClient mqttClient(tlsClient);

  String topicCmd;
  String topicLimit;
  String topicEstop;
  String topicState;
  String topicStatus;

  unsigned long lastWifiAttemptMs   = 0;
  unsigned long lastMqttAttemptMs   = 0;
  unsigned long lastStatePublishMs  = 0;
  unsigned long lastCloudCmdMs      = 0;
  bool wifiWasConnected             = false;
  bool stateDirty                   = true;

  const unsigned long WIFI_RETRY_MS        = 15000; // รอ 15 วินาที ไม่ตัดกลางคันขณะรอ DHCP
  const unsigned long MQTT_RETRY_MS        = 5000;
  const unsigned long STATE_PUBLISH_MIN_MS = 400;  // สูงสุด ~2.5 Hz (ประหยัดโควตา EMQX Serverless)
  const unsigned long STATE_HEARTBEAT_MS   = 3000; // ส่ง Heartbeat อย่างน้อยทุก 3 วินาที
#endif

// -----------------------------------------------------------------------------
// SECTION 03: ฟังก์ชันช่วยขับ PWM แยก Channel และคำนวณองศาอย่างปลอดภัย
// -----------------------------------------------------------------------------

#if defined(ESP32)
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
#endif

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

void writeAllServosImmediate(int baseCmd, int gripLCmd, int gripRCmd) {
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

  bool changed = (aBase != lastHwBase || aGripL != lastHwGripL || aGripR != lastHwGripR);
  lastHwBase  = aBase;
  lastHwGripL = aGripL;
  lastHwGripR = aGripR;

#if defined(ESP32)
  writeServoChannel(CH_SERVO_BASE, aBase);
  writeServoChannel(CH_SERVO_GRIP_L, aGripL);
  writeServoChannel(CH_SERVO_GRIP_R, aGripR);
#else
  servo1_BaseYaw.write(aBase);
  servo2_GripLeft.write(aGripL);
  servo3_GripRight.write(aGripR);
#endif

#if defined(USE_CLOUD_MQTT)
  if (changed) {
    stateDirty = true;
  }
#endif
}

void setServoTargets(int baseCmd, int gripLCmd, int gripRCmd, bool useSmoothing) {
  if (estopLatched) {
    return; // ระงับการขยับทุกกรณีเมื่อ E-STOP ทำงาน
  }
  tgtCmdBase  = (float)constrain(baseCmd, 0, 180);
  tgtCmdGripL = (float)constrain(gripLCmd, 0, 180);
  tgtCmdGripR = (float)constrain(gripRCmd, 0, 180);

  if (!useSmoothing) {
    curCmdBase  = tgtCmdBase;
    curCmdGripL = tgtCmdGripL;
    curCmdGripR = tgtCmdGripR;
    smoothMotionActive = false;
    writeAllServosImmediate((int)roundf(curCmdBase), (int)roundf(curCmdGripL), (int)roundf(curCmdGripR));
  } else {
    smoothMotionActive = true;
  }
}

void stepSmoothServos(unsigned long nowMs) {
  if (!smoothMotionActive || estopLatched) return;
  if (nowMs - lastServoTickMs < SERVO_TICK_MS) return;
  lastServoTickMs = nowMs;

  curCmdBase  += (tgtCmdBase  - curCmdBase)  * CLOUD_SMOOTH_ALPHA;
  curCmdGripL += (tgtCmdGripL - curCmdGripL) * CLOUD_SMOOTH_ALPHA;
  curCmdGripR += (tgtCmdGripR - curCmdGripR) * CLOUD_SMOOTH_ALPHA;

  if (fabsf(tgtCmdBase - curCmdBase) < 0.4f &&
      fabsf(tgtCmdGripL - curCmdGripL) < 0.4f &&
      fabsf(tgtCmdGripR - curCmdGripR) < 0.4f) {
    curCmdBase  = tgtCmdBase;
    curCmdGripL = tgtCmdGripL;
    curCmdGripR = tgtCmdGripR;
    smoothMotionActive = false;
  }

  writeAllServosImmediate((int)roundf(curCmdBase), (int)roundf(curCmdGripL), (int)roundf(curCmdGripR));
}

void setEstopState(bool active) {
  estopLatched = active;
  if (estopLatched) {
    // Safe Hold: หยุดค้างตำแหน่งปัจจุบันทันที ไม่กระชากกลับ
    tgtCmdBase  = curCmdBase;
    tgtCmdGripL = curCmdGripL;
    tgtCmdGripR = curCmdGripR;
    smoothMotionActive = false;
    Serial.println("ESTOP:ACTIVE");
  } else {
    Serial.println("ESTOP:RELEASED");
  }
#if defined(USE_CLOUD_MQTT)
  stateDirty = true;
#endif
}

void printBridgeStatus() {
  Serial.print("STATUS:BASE=GPIO");
  Serial.print(PIN_SERVO_BASE);
  Serial.print("(");
  Serial.print(lastHwBase);
  Serial.print("),GRIP_L=GPIO");
  Serial.print(PIN_SERVO_GRIP_L);
  Serial.print("(");
  Serial.print(lastHwGripL);
  Serial.print("),GRIP_R=GPIO");
  Serial.print(PIN_SERVO_GRIP_R);
  Serial.print("(");
  Serial.print(lastHwGripR);
  Serial.print("),ESTOP=");
  Serial.print(estopLatched ? "1" : "0");
#if defined(USE_CLOUD_MQTT)
  Serial.print(",WIFI=");
  Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
  Serial.print(",RSSI=");
  Serial.print(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  Serial.print(",MQTT=");
  Serial.print(mqttClient.connected() ? "ONLINE" : "OFFLINE");
#else
  Serial.print(",MQTT=DISABLED");
#endif
  Serial.println();
}

/**
 * @brief ตัวประมวลผลคำสั่งกลาง ใช้ร่วมกันทั้ง USB Serial และ Cloud MQTT
 */
void handleBridgeCommand(const char* cmd, bool fromCloud) {
  if (!cmd || cmd[0] == '\0') return;

  if (strcmp(cmd, "PING") == 0) {
    Serial.println("READY:3SERVO_S3CAM");
  } else if (strcmp(cmd, "STATUS") == 0) {
    printBridgeStatus();
  } else if (strncmp(cmd, "ESTOP:", 6) == 0) {
    bool active = (cmd[6] == '1' || cmd[6] == 't' || cmd[6] == 'T');
    setEstopState(active);
  } else if (strncmp(cmd, "LIMIT:", 6) == 0) {
    int mnB, mxB, mnGL, mxGL, mnGR, mxGR, unused1, unused2;
    int parsed = sscanf(cmd + 6, "%d,%d,%d,%d,%d,%d,%d,%d",
                        &mnB, &mxB, &mnGL, &mxGL, &mnGR, &mxGR, &unused1, &unused2);
    if (parsed == 6) {
      applySoftLimitPair(mnB, mxB, HARD_MIN_BASE, HARD_MAX_BASE, softMinBase, softMaxBase);
      applySoftLimitPair(mnGL, mxGL, HARD_MIN_GRIP_L, HARD_MAX_GRIP_L, softMinGripL, softMaxGripL);
      applySoftLimitPair(mnGR, mxGR, HARD_MIN_GRIP_R, HARD_MAX_GRIP_R, softMinGripR, softMaxGripR);
      Serial.println("OK:LIMITS");
    } else if (parsed == 8) {
      applySoftLimitPair(mnB, mxB, HARD_MIN_BASE, HARD_MAX_BASE, softMinBase, softMaxBase);
      applySoftLimitPair(unused1, unused2, HARD_MIN_GRIP_L, HARD_MAX_GRIP_L, softMinGripL, softMaxGripL);
      applySoftLimitPair(unused1, unused2, HARD_MIN_GRIP_R, HARD_MAX_GRIP_R, softMinGripR, softMaxGripR);
      Serial.println("OK:LIMITS");
    }
  } else {
    int v1, v2, v3, v4;
    int parsed = sscanf(cmd, "%d,%d,%d,%d", &v1, &v2, &v3, &v4);
    if (parsed == 3) {
      setServoTargets(v1, v2, v3, fromCloud);
    } else if (parsed == 4) {
      setServoTargets(v1, v4, v4, fromCloud);
    }
  }
}

// -----------------------------------------------------------------------------
// SECTION 04: ระบบสื่อสาร EMQX Cloud Serverless (Wi-Fi + MQTT over TLS :8883)
// -----------------------------------------------------------------------------
#if defined(USE_CLOUD_MQTT)

void publishCloudState(bool forceNow) {
  if (!mqttClient.connected()) return;
  unsigned long now = millis();
  if (!forceNow) {
    bool dueHeartbeat = (now - lastStatePublishMs >= STATE_HEARTBEAT_MS);
    bool dueChange    = stateDirty && (now - lastStatePublishMs >= STATE_PUBLISH_MIN_MS);
    if (!dueHeartbeat && !dueChange) return;
  }

  lastStatePublishMs = now;
  stateDirty = false;

  char payload[180];
  snprintf(
    payload,
    sizeof(payload),
    "{\"b\":%d,\"gl\":%d,\"gr\":%d,\"estop\":%s,\"rssi\":%d,\"uptime\":%lu,\"fw\":\"3SERVO_S3CAM\"}",
    lastHwBase,
    lastHwGripL,
    lastHwGripR,
    estopLatched ? "true" : "false",
    WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
    (unsigned long)(now / 1000UL)
  );
  mqttClient.publish(topicState.c_str(), payload, true);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  char msgBuf[RX_BUF_SIZE];
  size_t copyLen = (length < RX_BUF_SIZE - 1) ? length : (RX_BUF_SIZE - 1);
  memcpy(msgBuf, payload, copyLen);
  msgBuf[copyLen] = '\0';

  // ตัดช่องว่าง/ขึ้นบรรทัดใหม่ท้ายข้อความ
  while (copyLen > 0 && (msgBuf[copyLen - 1] == '\n' || msgBuf[copyLen - 1] == '\r' || msgBuf[copyLen - 1] == ' ')) {
    msgBuf[--copyLen] = '\0';
  }

  if (topicEstop.equals(topic)) {
    bool isStop = (strstr(msgBuf, "true") != nullptr || strcmp(msgBuf, "1") == 0 || strstr(msgBuf, "ON") != nullptr);
    setEstopState(isStop);
    publishCloudState(true);
    return;
  }

  if (topicLimit.equals(topic)) {
    handleBridgeCommand(msgBuf, true);
    publishCloudState(true);
    return;
  }

  if (topicCmd.equals(topic)) {
    lastCloudCmdMs = millis();
    handleBridgeCommand(msgBuf, true);
  }
}

void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t r = info.wifi_sta_disconnected.reason;
    Serial.print("WIFI:DISCONNECTED_REASON=");
    Serial.print(r);
    if (r == 201) {
      Serial.println(" (NO_AP_FOUND: หาชื่อ Wi-Fi ไม่เจอ / คลื่นเป็น 5GHz)");
    } else if (r == 15 || r == 202 || r == 204) {
      Serial.println(" (AUTH/HANDSHAKE_FAIL: รหัสผ่านผิด หรือ WPA3-Only)");
    } else if (r == 2 || r == 205) {
      Serial.println(" (AUTH_EXPIRE/CONN_FAIL: สัญญาณ/WPA3/PMF ไม่เข้ากัน)");
    } else {
      Serial.println();
    }
  }
}

void setupCloudMqtt() {
  String prefix = String(MQTT_TOPIC_PREFIX);
  topicCmd    = prefix + "/cmd";
  topicLimit  = prefix + "/limit";
  topicEstop  = prefix + "/estop";
  topicState  = prefix + "/state";
  topicStatus = prefix + "/status";

  WiFi.onEvent(onWifiEvent);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(150);
  WiFi.setTxPower(WIFI_POWER_11dBm); // แก้ปัญหาเฉพาะของ ESP32-S3 ที่กำลังส่งสูงสุดทำให้ต่อ Wi-Fi หลุด (status=6)
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); // ลด Latency ของ Wi-Fi เพื่อความลื่นไหลของ Digital Twin
  Serial.print("WIFI:CONNECTING_TO=");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiAttemptMs = millis();

  // ตั้งค่า TLS Root CA Certificate สำหรับ EMQX Cloud Serverless (Port 8883)
  if (strstr(EMQX_CA_CERT, "PASTE_YOUR_EMQXSL_CA_CRT_CONTENT_HERE") == nullptr) {
    tlsClient.setCACert(EMQX_CA_CERT);
    Serial.println("MQTT:TLS_CA_VERIFIED");
  } else {
    Serial.println("MQTT:WARN_MISSING_CA_CERT");
  }

  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(15); // ตรวจจับการหลุดเชื่อมต่อเร็วขึ้นสำหรับ LWT
  mqttClient.setCallback(onMqttMessage);
}

const char* wifiStatusName(wl_status_t st) {
  switch (st) {
    case WL_NO_SSID_AVAIL:  return "NO_SSID_AVAIL (check 2.4GHz/SSID)";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED (check Password/WPA2)";
    case WL_CONNECTION_LOST:return "CONNECTION_LOST";
    case WL_DISCONNECTED:   return "DISCONNECTED";
    case WL_IDLE_STATUS:    return "IDLE";
    default:                return "CONNECTING";
  }
}

void maintainCloudMqtt(unsigned long nowMs) {
  // 1. ดูแลการเชื่อมต่อ Wi-Fi แบบ Non-Blocking
  wl_status_t st = WiFi.status();
  if (st != WL_CONNECTED) {
    wifiWasConnected = false;
    if (nowMs - lastWifiAttemptMs >= WIFI_RETRY_MS) {
      lastWifiAttemptMs = nowMs;
      Serial.print("WIFI:RECONNECTING (status=");
      Serial.print((int)st);
      Serial.print(":");
      Serial.print(wifiStatusName(st));
      Serial.println(")");
      WiFi.disconnect(false);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  if (!wifiWasConnected) {
    wifiWasConnected = true;
    Serial.print("WIFI:CONNECTED IP=");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI=");
    Serial.println(WiFi.RSSI());
  }

  // 2. ดูแลการเชื่อมต่อ EMQX Cloud MQTT over TLS แบบ Non-Blocking
  if (!mqttClient.connected()) {
    if (nowMs - lastMqttAttemptMs >= MQTT_RETRY_MS) {
      lastMqttAttemptMs = nowMs;
      // เชื่อมต่อพร้อมตั้งค่า LWT: หากบอร์ดไฟดับ/หลุดเน็ต Broker จะส่ง "OFFLINE" เข้า topicStatus ทันที
      bool ok = mqttClient.connect(
        MQTT_CLIENT_ID,
        MQTT_USERNAME,
        MQTT_PASSWORD,
        topicStatus.c_str(),
        1,       // Will QoS = 1
        true,    // Will Retain = true
        "OFFLINE"
      );
      if (ok) {
        Serial.println("MQTT:CONNECTED_EMQX");
        mqttClient.publish(topicStatus.c_str(), "ONLINE", true);
        mqttClient.subscribe(topicCmd.c_str(), 0);
        mqttClient.subscribe(topicLimit.c_str(), 1);
        mqttClient.subscribe(topicEstop.c_str(), 1);
        publishCloudState(true);
      } else {
        Serial.print("MQTT:ERR_STATE=");
        Serial.println(mqttClient.state());
      }
    }
    return;
  }

  mqttClient.loop();
  publishCloudState(false);
}

#endif // USE_CLOUD_MQTT

// -----------------------------------------------------------------------------
// SECTION 05: ตั้งค่าเริ่มต้นของระบบ (setup)
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

#if defined(ESP32)
  attachServoChannel(PIN_SERVO_BASE, CH_SERVO_BASE);
  attachServoChannel(PIN_SERVO_GRIP_L, CH_SERVO_GRIP_L);
  attachServoChannel(PIN_SERVO_GRIP_R, CH_SERVO_GRIP_R);
#else
  servo1_BaseYaw.attach(PIN_SERVO_BASE);
  servo2_GripLeft.attach(PIN_SERVO_GRIP_L);
  servo3_GripRight.attach(PIN_SERVO_GRIP_R);
#endif

  writeAllServosImmediate(HOME_CMD_BASE, HOME_CMD_GRIP_L, HOME_CMD_GRIP_R);
  Serial.println("READY:3SERVO_S3CAM");

#if defined(USE_CLOUD_MQTT)
  setupCloudMqtt();
#endif
}

// -----------------------------------------------------------------------------
// SECTION 06: ลูปหลัก Non-Blocking (USB Serial + Cloud MQTT + Smooth Servo 50 Hz)
// -----------------------------------------------------------------------------

void loop() {
  unsigned long nowMs = millis();

  // 1. รับคำสั่งจาก USB WebSerial (ทำงานได้ตลอดเวลาแม้ไม่มี Wi-Fi)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) {
        handleBridgeCommand(rxBuf, false);
      }
      rxLen = 0;
    } else if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;
    }
  }

  // 2. ดูแลการเชื่อมต่อและรับ-ส่งข้อมูล EMQX Cloud (Non-Blocking)
#if defined(USE_CLOUD_MQTT)
  maintainCloudMqtt(nowMs);
#endif

  // 3. ขับ Smoothing สำหรับคำสั่งที่มาจาก Cloud (50 Hz)
  stepSmoothServos(nowMs);
}
