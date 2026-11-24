# 4-DOF Robotic Arm Digital Twin (Hand Tracking & IoT WebSerial)

Web-based 3D Digital Twin for a 4-DOF parallel-linkage robotic arm with real-time MediaPipe Hand Tracking, 3D Inverse Kinematics, Pick-and-Place simulation, CAD part inspection (39 bodies), and USB WebSerial bridge for Arduino / ESP32.

## Features
- **3D CAD Digital Twin**: Full 39-body 4-bar parallel linkage assembly rendered in Three.js with Exploded View, X-Ray mode, CAD edges, and clickable part inspector.
- **Real-Time Hand Tracking**: Uses MediaPipe Hands via webcam (processed 100% locally in the browser; no video is uploaded) in either 3D Inverse Kinematics or Direct 4-DOF Joint mode.
- **IoT Hardware Bridge (WebSerial)**: Streams real-time servo angles (`S1,S2,S3,S4\n` at `115200` baud) directly from Chrome/Edge to Arduino or ESP32 over USB.
- **3D Model Exports**: Includes `.glb`, `.stl` (full assembly + 39 individual parts), `.urdf`, and `.sc` source files.

## Quick Start

### Option 1: Local Server (Windows)
Double-click `start_digital_twin.bat` or run:
```bash
python -m http.server 8080 --bind 127.0.0.1
```
Then open `http://localhost:8080` in Chrome or Edge.

### Option 2: GitHub Pages
Enable **GitHub Pages** on the `main` branch (root `/`). Both Webcam (`getUserMedia`) and WebSerial API work automatically over HTTPS.

## Arduino / ESP32 Wiring
Upload `arduino_servo_bridge/arduino_servo_bridge.ino` to your board:

| Servo | Joint | Arduino Pin | ESP32 Pin |
| :--- | :--- | :---: | :---: |
| Servo 1 | Base Yaw | D3 | GPIO 13 |
| Servo 2 | Shoulder Pitch | D5 | GPIO 12 |
| Servo 3 | Elbow Pitch | D6 | GPIO 14 |
| Servo 4 | Gripper Claws | D9 | GPIO 27 |

*Note: For ESP32, install the `ESP32Servo` library via Arduino Library Manager.*

## Project Structure
```text
├── index.html                           # Main Web Digital Twin UI
├── start_digital_twin.bat               # Local localhost-only HTTP server launcher
├── css/style.css                        # Dark-mode HUD styling
├── js/
│   ├── app.js                           # 3D scene, 4-bar kinematics, IK, Hand Tracking, WebSerial
│   └── arm_cad_data.js                  # Extracted 39-body CAD geometry bundle
├── arduino_servo_bridge/
│   └── arduino_servo_bridge.ino         # Arduino / ESP32 USB Serial servo receiver
└── models/
    ├── Robotic_Arm_4DOF.urdf            # ROS / PyBullet URDF description
    ├── Robotic_Arm_4DOF_Assembly.glb    # glTF 2.0 binary 3D model
    ├── Robotic_Arm_4DOF_Assembly.sc     # SolidCraft parametric CAD source
    └── stl/                             # Full assembly + 39 individual STL parts
```
