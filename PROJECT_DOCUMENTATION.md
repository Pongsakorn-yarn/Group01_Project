# เอกสารสถาปัตยกรรมและเส้นทางข้อมูลเชิงเทคนิค — Group01_Project

> **รายวิชา:** ISY49303 IoT | **โปรเจกต์:** 4-DOF Robotic Arm Digital Twin (Hand Tracking & IoT WebSerial)
> **วัตถุประสงค์เอกสารนี้:** เป็นคู่มืออ้างอิงกลาง (Single Source of Truth) สำหรับนักพัฒนาและผู้รับช่วงงานต่อ เพื่อเข้าใจโครงสร้างไฟล์ เส้นทางข้อมูลระดับฟังก์ชัน ระบบพิกัด 4-Bar Kinematics โปรโตคอลสื่อสาร และขั้นตอนส่งมอบงาน *(สำหรับวิธีติดตั้งและใช้งานเบื้องต้น ดูที่ [`README.md`](./README.md))*

---

## 1. โครงสร้างโฟลเดอร์และหน้าที่ของแต่ละไฟล์

```text
Group01_Project/
├── README.md                                  # คู่มือเริ่มต้นใช้งานตั้งแต่ต้นจนจบ + Pinout + Troubleshooting
├── PROJECT_DOCUMENTATION.md                   # เอกสารสถาปัตยกรรมและเส้นทางข้อมูลเชิงเทคนิค (ไฟล์นี้)
├── start_digital_twin.bat                     # ตัวเปิด Local HTTP Server (127.0.0.1:8080) บน Windows
├── index.html                                 # โครงสร้างหน้าเว็บ HUD 5 ส่วน + โหลดไลบรารี CDN
├── css/
│   └── style.css                              # ธีม Dark-Mode Glassmorphism + จัดลำดับชั้น z-index
├── js/
│   ├── arm_cad_data.js                        # ข้อมูลเรขาคณิต CAD 39 ชิ้น (Base64 Float32Array) [Auto-generated]
│   └── app.js                                 # สมองหลักของระบบ แบ่งเป็น SECTION 01–12 (JSDoc ภาษาไทย)
├── arduino_servo_bridge/
│   └── arduino_servo_bridge.ino               # เฟิร์มแวร์รับค่า CSV ผ่าน USB Serial แล้วขับ Servo 4 ตัว
├── models/
│   ├── Robotic_Arm_4DOF_Assembly.sc           # ไฟล์ต้นฉบับ Parametric CAD (SolidCraft)
│   ├── Robotic_Arm_4DOF_Assembly.glb          # โมเดล 3D มาตรฐาน glTF 2.0 Binary
│   ├── Robotic_Arm_4DOF.urdf                  # ไฟล์อธิบายหุ่นยนต์ XML สำหรับ ROS / PyBullet / Gazebo
│   └── stl/                                   # ไฟล์ .stl ชิ้นส่วนแยก 39 ชิ้น (00_–38_) + ไฟล์ประกอบเต็ม 1 ชิ้น
└── .gitignore                                 # กฎยกเว้นไฟล์ระบบ/ไฟล์ขยะไม่ให้นำขึ้น Git
```

| ลำดับ | ไฟล์ / โฟลเดอร์ | หมวดหมู่ | หน้าที่หลัก | ควรแก้ไขด้วยมือหรือไม่ |
| :---: | :--- | :--- | :--- | :---: |
| 1 | `start_digital_twin.bat` | Launcher | เปิด Python HTTP Server ที่ `127.0.0.1:8080` เพื่อให้ได้ Secure Context (`localhost`) | แก้ได้ (พอร์ต) |
| 2 | `index.html` | Web UI | จัดวางหน้าจอ 3D + HUD 5 ส่วน (`01`–`05`) และกำหนด DOM IDs ทั้งหมด | แก้ได้ |
| 3 | `css/style.css` | Styling | ตกแต่ง UI และคลาสสถานะ Real-Time (`.active`, `.warn`, `.danger`, `.selected`) | แก้ได้ |
| 4 | `js/arm_cad_data.js` | CAD Data | เก็บ `window.ARM_CAD_DATA` (39 ชิ้น, 18,080 สามเหลี่ยม) สกัดจาก `.sc` | ❌ ห้ามแก้ด้วยมือ |
| 5 | `js/app.js` | Core Engine | ควบคุม Three.js, FK/IK, Hand Tracking, Pick & Place, Recorder, WebSerial | แก้ได้ |
| 6 | `arduino_servo_bridge/arduino_servo_bridge.ino` | IoT Firmware | รับ `"S1,S2,S3,S4\n"` ที่ `115200 baud` และสั่งขับ Servo 4 แกน | แก้ได้ |
| 7 | `models/` | CAD Assets | ไฟล์ต้นฉบับ `.sc` และไฟล์ส่งออก `.glb`, `.urdf`, `stl/*.stl` (40 ไฟล์) | แก้ได้เฉพาะ `.urdf` |

---

## 2. เส้นทางข้อมูลแบบละเอียดระดับฟังก์ชัน (End-to-End Data Flow)

### 2.1 ลำดับการทำงานแบบ Real-Time (Runtime Control Pipeline)

```mermaid
sequenceDiagram
    autonumber
    participant User as ผู้ใช้ / กล้องเว็บแคม
    participant UI as index.html (DOM & Events)
    participant AI as js/app.js (MediaPipe + solveIK)
    participant Loop as js/app.js (animate 60 FPS)
    participant Render as Three.js WebGL & HUD
    participant Serial as WebSerial API (25 Hz)
    participant MCU as arduino_servo_bridge.ino

    User->>UI: ขยับมือหน้ากล้อง / เลื่อน Slider / กดโหมด Demo
    UI->>AI: ส่งเฟรมภาพ 640×480 หรือค่า Slider
    AI->>AI: onHandResults() → คำนวณจุดฝ่ามือ & Pinch → solveIK(tx, ty, tz)
    AI->>Loop: เขียนค่าเป้าหมายลง state.target {j1, j2, j3, j4, wrist}
    loop ทุกเฟรม (~16.6 ms / 60 FPS)
        Loop->>Loop: EMA Smoothing Filter (state.target → state.current)
        Loop->>Render: updateArmKinematics() อัปเดตกลไก 4-Bar 12 กลุ่ม + TCP + หยิบกล่อง
        Loop->>UI: updateJointLabels() อัปเดตตัวเลของศา/PWM/TCP บน HUD
        opt เมื่อเชื่อมต่อ Serial และครบช่วง ~40 ms
            Loop->>Serial: sendSerialTelemetry() ส่ง "S1,S2,S3,S4\n"
            Serial->>MCU: USB UART (115200 baud, 8N1)
            MCU->>MCU: loop() → readStringUntil('\n') → sscanf() → constrain(0,180) → Servo.write()
        end
    end
```

### 2.2 ตารางแยกขั้นตอนข้อมูลตามไฟล์และฟังก์ชัน

| ขั้น | ต้นทางข้อมูล | ไฟล์ → ฟังก์ชัน / หมวดโค้ด | รูปแบบข้อมูล (Payload / Variable) | ปลายทางถัดไป |
| :---: | :--- | :--- | :--- | :--- |
| **1** | เปิดระบบ | `start_digital_twin.bat` → `index.html` | โหลด `style.css`, Three.js, MediaPipe, `arm_cad_data.js`, `app.js` | `js/app.js` (SECTION 01–04) |
| **2** | เรขาคณิต CAD | `js/arm_cad_data.js` → `js/app.js` (`b64ToFloat32`, SECTION 04) | Base64 → `Float32Array` → `THREE.Mesh` 39 ชิ้น ใน 12 กลุ่มกลไก | `groupMap` ภายใน `cadRoot` |
| **3A** | กล้องเว็บแคม | `js/app.js` (`toggleWebcam` → `onHandResults`, SECTION 11) | ภาพ 640×480 → จุดมือ 21 จุด (`lm[0..20]`) → `(targetX, targetY, targetZ)` + `gripperOpenPct` | `solveIK()` → `state.target` |
| **3B** | UI / Demo / Play | `index.html` → `js/app.js` (SECTION 08, 09, 12) | ค่า Slider, ลำดับท่า Pick & Place / Circle IK, หรือ `recordedFrames[i]` | `state.target` |
| **4** | การกรองสัญญาณ | `js/app.js` (`animate` — EMA Filter, SECTION 12) | `current += (target - current) * smoothing` | `state.current` |
| **5A** | แสดงผล 3D & HUD | `js/app.js` (`updateArmKinematics`, `updateJointLabels`, SECTION 06, 08) | มุมหมุน 12 กลุ่ม, พิกัด `TCP (X,Y,Z)` หน่วย mm, ฟิสิกส์หยิบ-วางกล่อง | `#canvas-container` & HUD |
| **5B** | ส่งคำสั่งฮาร์ดแวร์ | `js/app.js` (`sendSerialTelemetry`, SECTION 10) | ASCII CSV `"S1,S2,S3,S4\n"` ช่วง `0..180` ทุก ~40 ms (`25 Hz`) | พอร์ต USB Serial |
| **6** | ขับมอเตอร์จริง | `arduino_servo_bridge/arduino_servo_bridge.ino` (`loop`) | `sscanf("%d,%d,%d,%d")` → `constrain(0, 180)` → `Servo.write()` | มอเตอร์ Servo 4 ตัว |
| **7** | ส่งออกเส้นทาง | `js/app.js` (`#btn-export-traj`, SECTION 09) | JSON (`model`, `timestamp`, `current_pose`, `recorded_frames`) | `robotic_arm_4dof_trajectory.json` |

### 2.3 เส้นทางข้อมูลโมเดล 3 มิติ (CAD Asset Pipeline)

| ขั้น | ต้นทาง | ไฟล์ผลลัพธ์ | โครงสร้าง / รูปแบบข้อมูล | การนำไปใช้งาน |
| :---: | :--- | :--- | :--- | :--- |
| **1** | `models/Robotic_Arm_4DOF_Assembly.sc` | `js/arm_cad_data.js` | `window.ARM_CAD_DATA = { meta, bodies[0..38] }` (Base64 `pos_b64`, `norm_b64`, `edge_b64`) | โหลดเรนเดอร์ 3D ใน `js/app.js` |
| **2** | `models/Robotic_Arm_4DOF_Assembly.sc` | `models/Robotic_Arm_4DOF_Assembly.glb` | glTF 2.0 Binary | แสดงผล 3D Viewer / ดาวน์โหลดจากหน้าเว็บ |
| **3** | `models/Robotic_Arm_4DOF_Assembly.sc` | `models/stl/*.stl` (40 ไฟล์) | Binary STL (หน่วย mm) | พิมพ์ 3 มิติ / อ้างอิงใน URDF |
| **4** | `models/Robotic_Arm_4DOF_Assembly.sc` | `models/Robotic_Arm_4DOF.urdf` | URDF XML (สเกล `0.001` เป็นเมตร) | จำลองหุ่นยนต์ใน ROS / PyBullet / Gazebo |

---

## 3. ระบบพิกัด จลนศาสตร์ 4-Bar และโปรโตคอลการสื่อสาร

### 3.1 การแปลงระบบพิกัด (CAD Z-up → Three.js Y-up)
* **ในระบบ CAD (SolidCraft):** `+X` = ด้านหน้า, `+Y` = ด้านซ้าย, `+Z` = ด้านบน (หน่วยมิลลิเมตร)
* **ใน Three.js:** หมุนกลุ่มราก `cadRoot.rotation.x = -Math.PI / 2` เพื่อแปลงให้แกน `+Z` ของ CAD ชี้ขึ้นตามแกน `+Y` ของฉาก 3 มิติโดยไม่ต้องคำนวณพิกัดชิ้นส่วนใหม่

### 3.2 พิกัดจุดหมุนและมิติหลัก (อ้างอิงจาก `Robotic_Arm_4DOF_Assembly.sc`)
* ความยาวแขนท่อนบน `L1 = 80.0 mm`, แขนท่อนล่าง `L2 = 80.0 mm`
* ระยะชดเชยหัวคีบจากข้อมือถึงปลายแขนกล (TCP Offset): `~76.6 mm`
* จุดหมุนหลัก (หน่วย mm ในพิกัด CAD Home Pose):
  * `yaw`: `[-28.0, 0.0, 29.0]` | `shoulder`: `[-28.0, 0.0, 64.0]`
  * `elbow`: `[5.809, 0.0, 136.505]` | `wrist`: `[85.371, 0.0, 128.142]`

### 3.3 ข้อกำหนดโปรโตคอล USB Serial & สูตรแปลงองศา Servo
* **การเชื่อมต่อ:** `115200 baud`, `8N1`, ความถี่ส่งสูงสุด `25 Hz` (เว้นระยะขั้นต่ำ `40 ms` ใน `sendSerialTelemetry()`)
* **รูปแบบแพ็กเก็ต:** `"S1,S2,S3,S4\n"` (ตัวเลขจำนวนเต็ม 4 ค่า คั่นด้วยจุลภาค ปิดท้ายด้วย `\n`)
* **สูตรแปลงจากค่า Digital Twin (`state.current`) เป็นองศา Servo (`0..180°`):**
  * `S1 = clamp(round(90 + j1), 0, 180)` *(ช่วงใช้งาน `j1`: `-90°` ถึง `+90°`)*
  * `S2 = clamp(round(90 + j2), 0, 180)` *(ช่วงใช้งาน `j2`: `-45°` ถึง `+65°`)*
  * `S3 = clamp(round(90 + j3), 0, 180)` *(ช่วงใช้งาน `j3`: `-55°` ถึง `+55°`)*
  * `S4 = clamp(round((j4 / 100) * 180), 0, 180)` *(ช่วงใช้งาน `j4`: `0%` ปิดสนิท ถึง `100%` อ้าสุด)*

---

## 4. มาตรฐานคอมเมนต์และการจัดโครงสร้างโค้ดที่ใช้ในโปรเจกต์

เพื่อให้ผู้ที่มาพัฒนาต่อสามารถอ่านและแก้ไขได้อย่างปลอดภัย โค้ดทุกไฟล์ใช้มาตรฐานเดียวกัน:
1. **File Header มาตรฐาน:** ระบุ `@file`, `@project (ISY49303 IoT)`, `@description`, `@dependencies` และ `@data_flow` ไว้บนสุดของทุกไฟล์
2. **สารบัญแบ่งหมวดหมู่ (Section Indexing):**
   * `index.html` และ `css/style.css` แบ่งเป็นส่วน `01`–`05` ตรงกัน
   * `js/app.js` แบ่งเป็น `SECTION 01`–`12` พร้อม **JSDoc** (`@param`, `@returns`, `@type`) ครบทุกฟังก์ชัน
   * `arduino_servo_bridge.ino` ใช้ **Doxygen** (`@brief`, `@pinout`, `@protocol`)
3. **การลดความซ้ำซ้อน (DRY):** รวมลอจิกตัดการเชื่อมต่อ WebSerial ไว้ที่ `disconnectSerial()`, แคช DOM Elements ไว้ที่ออบเจกต์ `ui`, และใช้ค่าคงที่ `HOME_POSE` ร่วมกัน

---

## 5. แนวทางการส่งมอบและส่งออกผลลัพธ์พร้อมใช้งาน

1. **อัปเดตขึ้น GitHub Repository (`main`):**
   ```powershell
   git add .
   git commit -m "docs: organize project structure, reduce redundancy, and standardize Thai comments (ISY49303)"
   git push origin main
   ```
2. **เปิดใช้งานออนไลน์ผ่าน GitHub Pages:**
   ไปที่ **Repository → Settings → Pages → Source: Deploy from a branch (`main` / `/ (root)`)** จะได้ลิงก์ HTTPS พร้อมใช้งานจริง
3. **สร้างไฟล์ `.zip` สะอาดสำหรับส่งงาน (ไม่ติด `.git/` และไฟล์ขยะ):**
   ```powershell
   git archive --format=zip --prefix=Group01_Project/ HEAD -o Group01_Project_ISY49303_Ready.zip
   ```
