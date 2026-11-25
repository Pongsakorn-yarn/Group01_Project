/**
 * =============================================================================
 * @file        js/app.js
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description สมองประมวลผลหลักของ Digital Twin ครอบคลุมการเรนเดอร์โมเดล CAD 3 มิติ,
 *              การคำนวณ Forward Kinematics (FK) ของกลไก 4-Bar Parallel Linkage,
 *              การคำนวณ Inverse Kinematics (IK) แบบวิเคราะห์ 3 มิติ, การตรวจจับมือ
 *              ด้วย MediaPipe Hands, ฟิสิกส์หยิบ-วางวัตถุ, การบันทึก/ส่งออกท่าทาง
 *              และการส่งคำสั่งผ่าน USB WebSerial ไปยังบอร์ด Arduino / ESP32
 * -----------------------------------------------------------------------------
 * @dependencies
 *   - THREE & THREE.OrbitControls (โหลดผ่าน CDN ใน index.html)
 *   - Hands, drawConnectors, drawLandmarks (MediaPipe Hands ผ่าน CDN)
 *   - window.ARM_CAD_DATA (ชุดข้อมูล Mesh 39 ชิ้นจาก js/arm_cad_data.js)
 *
 * @data_flow
 *   1. ข้อมูลขาเข้า (INPUTS)
 *      - กล้องเว็บแคม -> MediaPipe Hands (21 จุดบนมือ) -> onHandResults() -> solveIK()
 *      - ตัวเลื่อน UI Sliders / โหมดจำลองอัตโนมัติ (Pick & Place, Circle IK) / Play
 *      -> เขียนองศาเป้าหมายลงใน `state.target {j1, j2, j3, j4, wrist}`
 *   2. การกรองสัญญาณและจลนศาสตร์ (ลูป `animate()` 60 FPS)
 *      - EMA Filter ค่อย ๆ ลู่ค่า `state.current` เข้าหา `state.target` ให้นุ่มนวล
 *      - `updateArmKinematics()` คำนวณตำแหน่งและมุมหมุนกลไก 4-Bar ทั้ง 12 กลุ่ม
 *        (39 ชิ้นส่วน), อัปเดตพิกัดปลายแขน (TCP) และตรวจจับการหยิบ-ปล่อยกล่อง
 *   3. ข้อมูลขาออก (OUTPUTS)
 *      - หน้าจอ 3D : Three.js WebGLRenderer + ตัวเลข Telemetry บน index.html
 *      - ฮาร์ดแวร์  : `sendSerialTelemetry()` ส่งข้อความ "S1,S2,S3,S4\n" ทุก ~40 ms
 *                   ผ่าน WebSerial (115200 baud) -> arduino_servo_bridge.ino
 *      - ส่งออกไฟล์ : ดาวน์โหลดเส้นทางท่าทาง JSON (`robotic_arm_4dof_trajectory.json`)
 *
 * @sections (สารบัญโค้ด — กด Ctrl+F ค้นหาเลข SECTION ได้ทันที)
 *   - SECTION 01 : ตรวจสอบข้อมูล CAD, ฟังก์ชันถอดรหัส Base64, ค่าคงที่ & State หลัก
 *   - SECTION 02 : ตั้งค่าฉาก 3 มิติ Three.js (Camera, Renderer, Lighting, พื้นโต๊ะ)
 *   - SECTION 03 : โครงสร้างต้นไม้จลนศาสตร์ (Kinematic Tree), จุดหมุน & กล่องเป้าหมาย
 *   - SECTION 04 : สร้างชิ้นส่วน Mesh 39 ชิ้นจาก CAD Data และรายการ Parts Inspector
 *   - SECTION 05 : การคลิกเลือกชิ้นส่วนบนฉาก 3 มิติ (Raycasting)
 *   - SECTION 06 : จลนศาสตร์เดินหน้า 4-Bar Parallel Linkage (Forward Kinematics)
 *   - SECTION 07 : จลนศาสตร์ย้อนกลับ 3 มิติ (Analytical 3D Inverse Kinematics)
 *   - SECTION 08 : การผูกตัวควบคุม UI Sliders, ปุ่มแสดงผล และป้ายตัวเลข HUD
 *   - SECTION 09 : โหมดจำลองอัตโนมัติ (Demo), บันทึกท่าทาง (Recorder) & ส่งออก JSON
 *   - SECTION 10 : การสื่อสารฮาร์ดแวร์ IoT ผ่าน USB WebSerial API
 *   - SECTION 11 : ระบบตรวจจับมือผ่านกล้องเว็บแคม (MediaPipe Hands)
 *   - SECTION 12 : ลูปเรนเดอร์และจำลองหลัก 60 FPS (Main Loop)
 * =============================================================================
 */
(function () {
  'use strict';

  /* =========================================================================
   * SECTION 01: ตรวจสอบข้อมูล CAD, ฟังก์ชันถอดรหัส Base64, ค่าคงที่ & State หลัก
   * ========================================================================= */

  /** @type {Object} ข้อมูลโมเดล CAD 39 ชิ้นและพิกัดจุดหมุนที่สกัดไว้ล่วงหน้า */
  const cadData = window.ARM_CAD_DATA;
  const cadDotEl = document.getElementById('cad-dot');
  const cadStatusEl = document.getElementById('cad-status-text');

  if (typeof THREE === 'undefined') {
    if (cadDotEl) cadDotEl.className = 'dot warn';
    if (cadStatusEl) cadStatusEl.textContent = '3D Engine: CDN Offline';
    alert('ไม่สามารถโหลดไลบรารี Three.js จาก CDN ได้ กรุณาตรวจสอบการเชื่อมต่ออินเทอร์เน็ตแล้วรีเฟรชหน้าเว็บ');
    return;
  }

  if (!cadData || !Array.isArray(cadData.bodies)) {
    if (cadDotEl) cadDotEl.className = 'dot warn';
    if (cadStatusEl) cadStatusEl.textContent = 'CAD Mesh: Missing';
    alert('ไม่พบข้อมูลโมเดล CAD (js/arm_cad_data.js)');
    return;
  }

  if (cadDotEl) cadDotEl.className = 'dot active';
  if (cadStatusEl) cadStatusEl.textContent = `CAD Mesh: ${cadData.bodies.length}/${cadData.bodies.length} Loaded`;

  /**
   * ถอดรหัสข้อความ Base64 เป็นอาเรย์เลขทศนิยม 32-bit (Float32Array)
   * สำหรับข้อมูลจุดยอด (Vertex), เวกเตอร์ตั้งฉาก (Normal) และเส้นขอบจาก arm_cad_data.js
   * @param {string} b64 - สตริง Base64 ของข้อมูลไบนารี float32
   * @returns {Float32Array} อาเรย์พิกัดที่พร้อมส่งเข้า Three.js BufferAttribute
   */
  function b64ToFloat32(b64) {
    const bin = atob(b64);
    const len = bin.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
      bytes[i] = bin.charCodeAt(i);
    }
    return new Float32Array(bytes.buffer);
  }

  /** ท่าเริ่มต้นมาตรฐานของแขนกล (CAD Home Pose) ใช้ร่วมกันทั้งตอนเปิดเว็บและปุ่ม Reset */
  const HOME_POSE = Object.freeze({ j1: 0, j2: 0, j3: 0, j4: 35, wrist: 0 });

  /** แคชอ้างอิง DOM Elements ที่ถูกอัปเดตซ้ำบ่อย ๆ เพื่อลดการค้นหา DOM ในลูป 60 FPS */
  const ui = {
    canvasContainer: document.getElementById('canvas-container'),
    partsList: document.getElementById('parts-list'),
    selectedPartBadge: document.getElementById('selected-part-badge'),
    tcpX: document.getElementById('tcp-x'),
    tcpY: document.getElementById('tcp-y'),
    tcpZ: document.getElementById('tcp-z'),
    valJ1: document.getElementById('val-j1'),
    valJ2: document.getElementById('val-j2'),
    valJ3: document.getElementById('val-j3'),
    valJ4: document.getElementById('val-j4'),
    valWrist: document.getElementById('val-wrist'),
    fpsBadge: document.getElementById('fps-badge'),
    pinchBar: document.getElementById('pinch-bar'),
    pinchValText: document.getElementById('pinch-val-text'),
    handModeTag: document.getElementById('hand-mode-tag'),
    handDetectTag: document.getElementById('hand-detect-tag'),
    camDot: document.getElementById('cam-dot'),
    camStatusText: document.getElementById('cam-status-text'),
    serialDot: document.getElementById('serial-dot'),
    serialStatusText: document.getElementById('serial-status-text'),
    btnSerial: document.getElementById('btn-connect-serial'),
    btnDemoPick: document.getElementById('btn-demo-pick'),
    btnDemoCircle: document.getElementById('btn-demo-circle'),
    btnRecord: document.getElementById('btn-record-pose'),
    btnPlay: document.getElementById('btn-play-pose')
  };

  /**
   * สถานะกลางทั้งหมดของระบบ Digital Twin (Single Source of Truth)
   * - j1: มุมหมุนฐาน Base Yaw (-90° ถึง +90°)
   * - j2: มุมก้ม-เงยไหล่ Shoulder Pitch (-45° ถึง +65°, ค่าบวก = โน้มไปด้านหน้า/ลง)
   * - j3: มุมศอก Elbow Pitch (-55° ถึง +55°, ค่าบวก = ปลายแขนก้มลง)
   * - j4: เปอร์เซ็นต์การอ้าหัวคีบ Gripper (0% = ปิดสนิท, 100% = อ้าสุด, 35% = ท่าเริ่มต้น)
   * - wrist: มุมชดเชยข้อมือ Wrist Trim (-45° ถึง +45°)
   */
  const state = {
    target: { ...HOME_POSE },
    current: { ...HOME_POSE },
    smoothing: 0.22,
    explodeFactor: 0,
    showEdges: true,
    showPivots: true,
    xrayMode: false,
    handMode: 'ik', // 'ik' | 'direct'
    mirrorCam: true,
    camRunning: false,
    camInitializing: false,
    demoMode: null, // 'pick' | 'circle' | null
    demoTime: 0,
    selectedBodyId: null,
    // สถานะการบันทึกและเล่นซ้ำท่าทาง
    recording: false,
    playing: false,
    recordedFrames: [],
    recordStartTime: 0,
    playIndex: 0,
    playElapsed: 0,
    // สถานะการเชื่อมต่อพอร์ต USB WebSerial
    serialPort: null,
    serialWriter: null,
    serialSending: false,
    lastSerialSend: 0
  };

  /**
   * หยุดโหมดอัตโนมัติ (Demo) และการเล่นซ้ำ (Play) พร้อมรีเซ็ตปุ่มสถานะบน UI
   */
  function stopAutomation() {
    state.demoMode = null;
    state.playing = false;
    ui.btnDemoPick.classList.remove('active');
    ui.btnDemoCircle.classList.remove('active');
    ui.btnPlay.classList.remove('active');
  }

  /* =========================================================================
   * SECTION 02: ตั้งค่าฉาก 3 มิติ Three.js (Camera, Renderer, Lighting, พื้นโต๊ะ)
   * ========================================================================= */

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x070b14);
  scene.fog = new THREE.FogExp2(0x070b14, 0.0009);

  const camera = new THREE.PerspectiveCamera(
    42,
    window.innerWidth / window.innerHeight,
    1,
    3000
  );
  camera.position.set(240, 210, 260);

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  ui.canvasContainer.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(25, 85, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.maxPolarAngle = Math.PI / 2 + 0.08;
  controls.minDistance = 80;
  controls.maxDistance = 900;

  // แสงสว่างในฉาก (Hemisphere + Directional หลักพร้อมเงา + Rim Light สีฟ้า)
  const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 0.85);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.15);
  dirLight.position.set(180, 340, 220);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 50;
  dirLight.shadow.camera.far = 800;
  const shadowBounds = 220;
  dirLight.shadow.camera.left = -shadowBounds;
  dirLight.shadow.camera.right = shadowBounds;
  dirLight.shadow.camera.top = shadowBounds;
  dirLight.shadow.camera.bottom = -shadowBounds;
  dirLight.shadow.bias = -0.0005;
  scene.add(dirLight);

  const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.55);
  rimLight.position.set(-220, 140, -180);
  scene.add(rimLight);

  // เส้นตารางโต๊ะทำงานและพื้นรับเงา
  const gridHelper = new THREE.GridHelper(500, 25, 0x0ea5e9, 0x1e293b);
  gridHelper.position.y = -0.2;
  scene.add(gridHelper);

  const groundMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(1200, 1200),
    new THREE.MeshStandardMaterial({ color: 0x090d18, roughness: 0.85, metalness: 0.1 })
  );
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.position.y = -0.5;
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);

  /* =========================================================================
   * SECTION 03: โครงสร้างต้นไม้จลนศาสตร์ (Kinematic Tree), จุดหมุน & กล่องเป้าหมาย
   * ========================================================================= */

  // แปลงระบบพิกัดจาก CAD (แกน Z ชี้ขึ้น) -> Three.js (แกน Y ชี้ขึ้น) โดยหมุนรอบแกน X -90°
  const cadRoot = new THREE.Group();
  cadRoot.rotation.x = -Math.PI / 2;
  scene.add(cadRoot);

  /**
   * สร้างคู่กลุ่ม Three.js (pivot และ offset) เพื่อให้ชิ้นส่วน Mesh ที่ถูกสร้างในพิกัด
   * ท่าเริ่มต้น (CAD Home Pose) สามารถหมุนรอบจุดหมุนใด ๆ `p0` ได้อย่างแม่นยำ
   * @param {number[]} p0 - พิกัดจุดหมุน [x, y, z] ในระบบพิกัด CAD (หน่วย mm)
   * @returns {{pivot: THREE.Group, offset: THREE.Group, p0: number[]}}
   */
  function createPivotNode(p0) {
    const pivot = new THREE.Group();
    pivot.position.set(p0[0], p0[1], p0[2]);
    const offset = new THREE.Group();
    offset.position.set(-p0[0], -p0[1], -p0[2]);
    pivot.add(offset);
    return { pivot, offset, p0: [...p0] };
  }

  /** พิกัดจุดหมุนที่ท่าเริ่มต้น CAD Home Pose (mm) อ้างอิงจาก Robotic_Arm_4DOF_Assembly.sc */
  const PIVOTS = {
    yaw: [-28.0, 0.0, 29.0],
    shoulder: [-28.0, 0.0, 64.0],
    upperParRear: [-44.0, -26.5, 72.0],
    elbow: [5.809, 0.0, 136.505],
    bellcrankTop: [5.809, -20.5, 159.505],
    bellcrankBack: [-16.191, -26.5, 126.505],
    elbowServoHorn: [-33.831, -26.5, 64.984],
    wrist: [85.371, 0.0, 128.142],
    clawLeft: [113.371, 12.0, 127.642],
    clawRight: [113.371, -12.0, 127.642],
    driveLinkRight: [111.371, -20.0, 134.642]
  };

  // สร้างลำดับชั้นกลุ่มกลไก 12 กลุ่มภายใน cadRoot
  const baseStaticGroup = new THREE.Group();
  cadRoot.add(baseStaticGroup);

  const turntableNode = createPivotNode(PIVOTS.yaw);
  cadRoot.add(turntableNode.pivot);
  // turntableWorld อยู่ในระบบพิกัด CAD Home Pose และหมุนไปพร้อมกับแกนฐาน (Base Yaw)
  const turntableWorld = turntableNode.offset;

  const upperMainNode = createPivotNode(PIVOTS.shoulder);
  turntableWorld.add(upperMainNode.pivot);

  const upperParNode = createPivotNode(PIVOTS.upperParRear);
  turntableWorld.add(upperParNode.pivot);

  const bellcrankNode = createPivotNode(PIVOTS.elbow);
  turntableWorld.add(bellcrankNode.pivot);

  const pushRodNode = createPivotNode(PIVOTS.bellcrankBack);
  turntableWorld.add(pushRodNode.pivot);

  const forearmMainNode = createPivotNode(PIVOTS.elbow);
  turntableWorld.add(forearmMainNode.pivot);

  const forearmParNode = createPivotNode(PIVOTS.bellcrankTop);
  turntableWorld.add(forearmParNode.pivot);

  const gripperMountNode = createPivotNode(PIVOTS.wrist);
  turntableWorld.add(gripperMountNode.pivot);

  // ก้ามหนีบซ้าย-ขวาและก้านขับเป็นลูกของ gripperMountNode.offset
  const clawLeftNode = createPivotNode(PIVOTS.clawLeft);
  gripperMountNode.offset.add(clawLeftNode.pivot);

  const clawRightNode = createPivotNode(PIVOTS.clawRight);
  gripperMountNode.offset.add(clawRightNode.pivot);

  const driveLinkNode = createPivotNode(PIVOTS.driveLinkRight);
  gripperMountNode.offset.add(driveLinkNode.pivot);

  /** ตารางจับคู่ชื่อกลุ่มจากไฟล์ arm_cad_data.js เข้ากับกลุ่ม Three.js จริง */
  const groupMap = {
    base_static: baseStaticGroup,
    turntable_yaw: turntableWorld,
    upper_main_arm: upperMainNode.offset,
    upper_parallel_link: upperParNode.offset,
    bellcrank: bellcrankNode.offset,
    elbow_push_rod: pushRodNode.offset,
    forearm_main: forearmMainNode.offset,
    forearm_parallel_link: forearmParNode.offset,
    gripper_mount: gripperMountNode.offset,
    gripper_claw_left: clawLeftNode.offset,
    gripper_claw_right: clawRightNode.offset,
    gripper_drive_link: driveLinkNode.offset
  };

  /**
   * สร้างสัญลักษณ์วงแหวนและแกนหมุนสำหรับแสดงตำแหน่งจุดหมุนข้อต่อ (Joint Pivot Marker)
   * @param {number} [radius=3.2] - รัศมีวงแหวน (mm)
   * @param {number} [color=0x10b981] - สีรูปแบบ Hex
   * @returns {THREE.Group} กลุ่มวัตถุสัญลักษณ์จุดหมุน
   */
  function makePivotIndicator(radius = 3.2, color = 0x10b981) {
    const g = new THREE.Group();
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(radius, 0.55, 12, 28),
      new THREE.MeshBasicMaterial({ color })
    );
    g.add(ring);

    const axisCyl = new THREE.Mesh(
      new THREE.CylinderGeometry(0.45, 0.45, radius * 4.5, 10),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.7 })
    );
    g.add(axisCyl);
    return g;
  }

  const markerYaw = makePivotIndicator(9, 0xf59e0b);
  markerYaw.position.set(...PIVOTS.yaw);
  markerYaw.children[1].rotation.x = Math.PI / 2;
  turntableWorld.add(markerYaw);

  const markerShoulder = makePivotIndicator(5.5, 0x10b981);
  markerShoulder.position.set(...PIVOTS.shoulder);
  markerShoulder.children[0].rotation.x = Math.PI / 2;
  turntableWorld.add(markerShoulder);

  const markerElbow = makePivotIndicator(5.0, 0x38bdf8);
  markerElbow.children[0].rotation.x = Math.PI / 2;
  bellcrankNode.pivot.add(markerElbow);

  const markerWrist = makePivotIndicator(4.5, 0xf43f5e);
  markerWrist.children[0].rotation.x = Math.PI / 2;
  gripperMountNode.pivot.add(markerWrist);

  const pivotMarkers = [markerYaw, markerShoulder, markerElbow, markerWrist];

  // จุดปลายแขนกล (TCP: Tool Center Point) อยู่กึ่งกลางระหว่างปากคีบ
  const tcpLocalPos = new THREE.Vector3(162.0, 0.0, 129.0);
  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(2.8, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x10b981 })
  );
  tcpMarker.position.copy(tcpLocalPos);
  gripperMountNode.offset.add(tcpMarker);

  // เส้นรอยเคลื่อนที่ของปลายแขนกล (TCP Motion Trail)
  const MAX_TRAIL = 120;
  const trailPositions = new Float32Array(MAX_TRAIL * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trailLine = new THREE.Line(
    trailGeo,
    new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.55 })
  );
  scene.add(trailLine);
  let trailCount = 0;

  // กล่องเป้าหมายสำหรับทดสอบฟิสิกส์หยิบ-วาง (Pick & Place Target Cube ขนาด 18 mm)
  const targetCubeMat = new THREE.MeshStandardMaterial({
    color: 0xf59e0b,
    roughness: 0.25,
    metalness: 0.2,
    emissive: 0x78350f,
    emissiveIntensity: 0.35
  });
  const targetCube = new THREE.Mesh(new THREE.BoxGeometry(18, 18, 18), targetCubeMat);
  targetCube.castShadow = true;
  targetCube.receiveShadow = true;
  const CUBE_HOME = new THREE.Vector3(135.0, 45.0, 9.0);
  targetCube.position.copy(CUBE_HOME);
  cadRoot.add(targetCube);
  let cubeGrabbed = false;

  /* =========================================================================
   * SECTION 04: สร้างชิ้นส่วน Mesh 39 ชิ้นจาก CAD Data และรายการ Parts Inspector
   * ========================================================================= */

  const bodyMeshes = [];
  const edgeLines = [];

  cadData.bodies.forEach((b) => {
    const posArr = b64ToFloat32(b.pos_b64);
    const normArr = b64ToFloat32(b.norm_b64);
    const edgeArr = b64ToFloat32(b.edge_b64);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
    geom.setAttribute('normal', new THREE.BufferAttribute(normArr, 3));

    // กำหนดผิววัสดุสมจริงตามประเภทชิ้นส่วน (โลหะสำหรับ Pin/Standoff, พลาสติกด้านสำหรับ Servo)
    let [r, g, bl] = b.color;
    let metalness = 0.2;
    let roughness = 0.32;
    if (b.name.startsWith('Pin_') || b.name.startsWith('Standoff_')) {
      r = 212; g = 218; bl = 228;
      metalness = 0.82;
      roughness = 0.22;
    } else if (b.name.startsWith('Servo_')) {
      r = 38; g = 44; bl = 58;
      metalness = 0.35;
      roughness = 0.55;
    }

    const colorHex = new THREE.Color(r / 255, g / 255, bl / 255);
    const mat = new THREE.MeshStandardMaterial({
      color: colorHex,
      metalness,
      roughness,
      side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geom, mat);
    mesh.name = b.name;
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // คำนวณจุดศูนย์กลางชิ้นส่วนสำหรับโหมดแยกชิ้นส่วน (Exploded View)
    const center = new THREE.Vector3(
      0.5 * (b.bbox_min[0] + b.bbox_max[0]),
      0.5 * (b.bbox_min[1] + b.bbox_max[1]),
      0.5 * (b.bbox_min[2] + b.bbox_max[2])
    );
    mesh.userData = {
      bodyId: b.id,
      name: b.name,
      group: b.group,
      triCount: b.tri_count,
      baseColor: colorHex.clone(),
      center
    };

    // เส้นขอบโครงสร้าง CAD (Feature Outline Edges)
    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute('position', new THREE.BufferAttribute(edgeArr, 3));
    const lines = new THREE.LineSegments(
      edgeGeo,
      new THREE.LineBasicMaterial({ color: 0xbae6fd, transparent: true, opacity: 0.28 })
    );
    mesh.add(lines);
    edgeLines.push(lines);

    const parentGroup = groupMap[b.group] || baseStaticGroup;
    parentGroup.add(mesh);
    bodyMeshes.push(mesh);

    // เพิ่มรายการชิ้นส่วนลงในแผง CAD Parts Inspector (ใช้ DOM API ที่ปลอดภัย)
    const item = document.createElement('div');
    item.className = 'part-item';
    item.dataset.bodyId = b.id;

    const leftWrap = document.createElement('span');
    leftWrap.style.cssText = 'display:flex;align-items:center;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';

    const swatch = document.createElement('span');
    swatch.className = 'part-swatch';
    swatch.style.background = `rgb(${r},${g},${bl})`;

    const nameSpan = document.createElement('span');
    nameSpan.textContent = `${String(b.id).padStart(2, '0')}. ${b.name}`;

    leftWrap.appendChild(swatch);
    leftWrap.appendChild(nameSpan);

    const triSpan = document.createElement('span');
    triSpan.style.cssText = 'color:#64748b;font-family:var(--font-mono);font-size:10px;';
    triSpan.textContent = `${b.tri_count}t`;

    item.appendChild(leftWrap);
    item.appendChild(triSpan);
    item.addEventListener('click', () => selectPart(b.id));
    ui.partsList.appendChild(item);
  });

  /**
   * ไฮไลต์ชิ้นส่วน CAD ที่ผู้ใช้เลือกทั้งในฉาก 3 มิติและรายการ Parts Inspector
   * หากคลิกซ้ำที่ชิ้นส่วนเดิมจะเป็นการยกเลิกการเลือก
   * @param {number} bodyId - รหัสชิ้นส่วน (0..38)
   */
  function selectPart(bodyId) {
    state.selectedBodyId = state.selectedBodyId === bodyId ? null : bodyId;
    document.querySelectorAll('.part-item').forEach((el) => {
      el.classList.toggle('selected', Number(el.dataset.bodyId) === state.selectedBodyId);
    });
    bodyMeshes.forEach((m) => {
      const isSel = m.userData.bodyId === state.selectedBodyId;
      m.material.emissive.setHex(isSel ? 0x0ea5e9 : 0x000000);
      m.material.emissiveIntensity = isSel ? 0.55 : 0.0;
    });
    if (state.selectedBodyId !== null) {
      const b = cadData.bodies[state.selectedBodyId];
      ui.selectedPartBadge.textContent = `#${b.id} ${b.name} (${b.group})`;
    } else {
      ui.selectedPartBadge.textContent = 'All 39 Parts';
    }
  }

  /* =========================================================================
   * SECTION 05: การคลิกเลือกชิ้นส่วนบนฉาก 3 มิติ (Raycasting)
   * ========================================================================= */

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let downPos = { x: 0, y: 0 };

  renderer.domElement.addEventListener('pointerdown', (e) => {
    downPos = { x: e.clientX, y: e.clientY };
  });

  renderer.domElement.addEventListener('pointerup', (e) => {
    // ข้ามหากเป็นการลากเมาส์หมุนกล้อง (> 5 px)
    if (Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y) > 5) return;
    pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
    pointer.y = -(e.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObjects(bodyMeshes, false);
    if (hits.length > 0) {
      const hitId = hits[0].object.userData.bodyId;
      selectPart(hitId);
      const itemEl = ui.partsList.querySelector(`[data-body-id="${hitId}"]`);
      if (itemEl) itemEl.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    } else if (state.selectedBodyId !== null) {
      selectPart(state.selectedBodyId);
    }
  });

  /* =========================================================================
   * SECTION 06: จลนศาสตร์เดินหน้า 4-Bar Parallel Linkage (Forward Kinematics)
   * ========================================================================= */

  /** เวกเตอร์แขนท่อนบน (Shoulder -> Elbow) ที่ท่าเริ่มต้น (mm) */
  const V1_HOME = {
    x: PIVOTS.elbow[0] - PIVOTS.shoulder[0], // 33.809 mm
    z: PIVOTS.elbow[2] - PIVOTS.shoulder[2]  // 72.505 mm
  };

  /** เวกเตอร์แขนท่อนล่าง (Elbow -> Wrist) ที่ท่าเริ่มต้น (mm) */
  const V2_HOME = {
    x: PIVOTS.wrist[0] - PIVOTS.elbow[0],    // 79.562 mm
    z: PIVOTS.wrist[2] - PIVOTS.elbow[2]     // -8.363 mm
  };

  /** มุมเริ่มต้นของก้านผลักข้อศอก (Elbow Push Rod) */
  const PUSHROD_HOME_ANGLE = Math.atan2(
    PIVOTS.bellcrankBack[2] - PIVOTS.elbowServoHorn[2],
    PIVOTS.bellcrankBack[0] - PIVOTS.elbowServoHorn[0]
  );

  // ตัวแปรชั่วคราวสำหรับคำนวณพิกัด TCP (จองหน่วยความจำครั้งเดียวเพื่อไม่ให้สร้างออบเจกต์ใหม่ทุกเฟรม)
  const tmpTcpWorld = new THREE.Vector3();
  const tmpInvCadMat = new THREE.Matrix4();

  /**
   * หมุนเวกเตอร์ 2 มิติในระนาบ XZ รอบแกน +Y (ระบบพิกัดมือขวา)
   * @param {number} vx - องค์ประกอบเวกเตอร์แกน X
   * @param {number} vz - องค์ประกอบเวกเตอร์แกน Z
   * @param {number} angleRad - มุมหมุน (เรเดียน)
   * @returns {{x: number, z: number}} เวกเตอร์ใหม่หลังหมุน
   */
  function rotateY2D(vx, vz, angleRad) {
    const c = Math.cos(angleRad);
    const s = Math.sin(angleRad);
    return {
      x: vx * c + vz * s,
      z: -vx * s + vz * c
    };
  }

  /**
   * คำนวณ Forward Kinematics (FK) สำหรับแขนกล 4-Bar Parallel Linkage
   * อ่านองศาที่ผ่านการกรองจาก `state.current` ไปอัปเดตกลุ่มชิ้นส่วนทั้ง 12 กลุ่ม
   * (39 ชิ้นส่วน CAD), คำนวณพิกัดปลายแขนกล (TCP) แสดงบน HUD พร้อมเส้นทางเคลื่อนที่
   * และตรวจสอบเงื่อนไขฟิสิกส์การหยิบ-ปล่อยกล่องเป้าหมาย
   */
  function updateArmKinematics() {
    const deg2rad = Math.PI / 180;
    const th1 = state.current.j1 * deg2rad;
    const th2 = state.current.j2 * deg2rad;
    const th3 = state.current.j3 * deg2rad;
    const wristTrim = state.current.wrist * deg2rad;
    // แปลงเปอร์เซ็นต์การอ้าหัวคีบ (0..100%) เป็นมุมหมุนก้ามหนีบ (-0.05..+0.57 rad)
    const clawAngle = -0.05 + (state.current.j4 / 100) * 0.62;

    // 1. หมุนฐาน (Base Yaw) รอบแกน Z ที่จุด [-28, 0, 29]
    turntableNode.pivot.rotation.z = th1;

    // 2. หมุนแขนท่อนบนหลักและก้านขนานรอบแกน Y ด้วยมุม th2
    upperMainNode.pivot.rotation.y = th2;
    upperParNode.pivot.rotation.y = th2;

    // 3. คำนวณตำแหน่งข้อศอกใหม่ในระบบพิกัดของแท่นหมุน
    const v1 = rotateY2D(V1_HOME.x, V1_HOME.z, th2);
    const elbowCur = {
      x: PIVOTS.shoulder[0] + v1.x,
      y: 0.0,
      z: PIVOTS.shoulder[2] + v1.z
    };

    // 4. ชิ้นส่วนสามเหลี่ยม Bellcrank เลื่อนตามข้อศอกและคงแนวตั้งตรง (0 pitch)
    bellcrankNode.pivot.position.set(elbowCur.x, elbowCur.y, elbowCur.z);
    bellcrankNode.pivot.rotation.y = 0;

    // 5. ก้านผลักศอก (Push Rod) เคลื่อนตามรูด้านหลัง Bellcrank และปรับมุมเข้าหาแตร Servo 3
    const bcBackCur = {
      x: elbowCur.x - 22.0,
      y: -26.5,
      z: elbowCur.z - 10.0
    };
    pushRodNode.pivot.position.set(bcBackCur.x, bcBackCur.y, bcBackCur.z);
    const curPrAngle = Math.atan2(
      bcBackCur.z - PIVOTS.elbowServoHorn[2],
      bcBackCur.x - PIVOTS.elbowServoHorn[0]
    );
    pushRodNode.pivot.rotation.y = PUSHROD_HOME_ANGLE - curPrAngle;

    // 6. แขนท่อนล่างหลัก (Forearm Main) เลื่อนตามข้อศอกและหมุนรอบแกน Y ด้วยมุม th3
    forearmMainNode.pivot.position.set(elbowCur.x, elbowCur.y, elbowCur.z);
    forearmMainNode.pivot.rotation.y = th3;

    // 7. ก้านขนานแขนท่อนล่าง (Forearm Parallel) เลื่อนตามจุดบน Bellcrank และหมุนด้วยมุม th3
    forearmParNode.pivot.position.set(elbowCur.x, -20.5, elbowCur.z + 23.0);
    forearmParNode.pivot.rotation.y = th3;

    // 8. คำนวณตำแหน่งข้อมือใหม่ในระบบพิกัดของแท่นหมุน
    const v2 = rotateY2D(V2_HOME.x, V2_HOME.z, th3);
    const wristCur = {
      x: elbowCur.x + v2.x,
      y: 0.0,
      z: elbowCur.z + v2.z
    };

    // 9. แท่นยึดหัวคีบ (Gripper Mount) คงระดับขนานพื้นด้วยกลไก 4-Bar + ชดเชยมุม wristTrim
    gripperMountNode.pivot.position.set(wristCur.x, wristCur.y, wristCur.z);
    gripperMountNode.pivot.rotation.y = wristTrim;

    // 10. ก้ามหนีบเฟืองคู่ซ้าย-ขวาและก้านขับหัวคีบ
    clawLeftNode.pivot.rotation.z = clawAngle;
    clawRightNode.pivot.rotation.z = -clawAngle;
    driveLinkNode.pivot.rotation.z = -clawAngle * 0.55;

    // อัปเดตพิกัดปลายแขนกล (TCP) กลับเป็นระบบพิกัด CAD (mm)
    cadRoot.updateMatrixWorld(true);
    tcpMarker.getWorldPosition(tmpTcpWorld);
    tmpInvCadMat.copy(cadRoot.matrixWorld).invert();
    const tcpCad = tmpTcpWorld.clone().applyMatrix4(tmpInvCadMat);

    ui.tcpX.textContent = tcpCad.x.toFixed(1);
    ui.tcpY.textContent = tcpCad.y.toFixed(1);
    ui.tcpZ.textContent = tcpCad.z.toFixed(1);

    // อัปเดตเส้นรอยเคลื่อนที่ (TCP Trail)
    if (trailCount < MAX_TRAIL) {
      trailPositions[trailCount * 3] = tmpTcpWorld.x;
      trailPositions[trailCount * 3 + 1] = tmpTcpWorld.y;
      trailPositions[trailCount * 3 + 2] = tmpTcpWorld.z;
      trailCount++;
    } else {
      trailPositions.copyWithin(0, 3);
      trailPositions[(MAX_TRAIL - 1) * 3] = tmpTcpWorld.x;
      trailPositions[(MAX_TRAIL - 1) * 3 + 1] = tmpTcpWorld.y;
      trailPositions[(MAX_TRAIL - 1) * 3 + 2] = tmpTcpWorld.z;
    }
    trailGeo.setDrawRange(0, trailCount);
    trailGeo.attributes.position.needsUpdate = true;

    // ฟิสิกส์การหยิบและปล่อยกล่องเป้าหมาย
    const distToCube = tcpCad.distanceTo(targetCube.position);
    if (!cubeGrabbed && distToCube < 26.0 && state.current.j4 < 28) {
      cubeGrabbed = true;
      targetCubeMat.emissive.setHex(0x065f46);
    } else if (cubeGrabbed && state.current.j4 > 34) {
      cubeGrabbed = false;
      targetCubeMat.emissive.setHex(0x78350f);
      // วางกล่องกลับลงบนพื้นโต๊ะ (z = 9 mm สำหรับกล่องสูง 18 mm)
      targetCube.position.z = 9.0;
      targetCube.rotation.set(0, 0, 0);
    }

    if (cubeGrabbed) {
      targetCube.position.copy(tcpCad);
      targetCube.rotation.z = th1;
    }
  }

  /* =========================================================================
   * SECTION 07: จลนศาสตร์ย้อนกลับ 3 มิติ (Analytical 3D Inverse Kinematics)
   * ========================================================================= */

  const L1 = 80.0; // ความยาวแขนท่อนบน (mm)
  const L2 = 80.0; // ความยาวแขนท่อนล่าง (mm)
  const HOME_ALPHA1 = Math.atan2(V1_HOME.x, V1_HOME.z);  // มุมเริ่มต้นแขนท่อนบน (+25.0° จากแกน +Z)
  const HOME_ALPHA2 = Math.atan2(-V2_HOME.z, V2_HOME.x); // มุมเริ่มต้นแขนท่อนล่าง (+6.0° ลงจากแกน +X)

  /**
   * คำนวณ Analytical 3D Inverse Kinematics (IK) แบบศอกยกขึ้น (Elbow-up)
   * แปลงพิกัดเป้าหมายของปลายแขนกล (TCP) หน่วย mm ในระบบพิกัด CAD ให้เป็นองศาข้อต่อ
   * (j1: ฐาน, j2: ไหล่, j3: ศอก) โดยใช้กฎของโคไซน์กับความยาวแขน L1 = 80 mm, L2 = 80 mm
   * @param {number} tx - พิกัดเป้าหมายแกน X ในระบบ CAD (mm)
   * @param {number} ty - พิกัดเป้าหมายแกน Y ในระบบ CAD (mm)
   * @param {number} tz - พิกัดเป้าหมายแกน Z ในระบบ CAD (mm)
   * @returns {{j1: number, j2: number, j3: number}} ค่าองศาข้อต่อที่จำกัดช่วงปลอดภัยแล้ว (องศา)
   */
  function solveIK(tx, ty, tz) {
    // 1. คำนวณมุมหมุนฐาน (Base Yaw) รอบแกน [-28, 0]
    const dxYaw = tx - PIVOTS.yaw[0];
    const dyYaw = ty - PIVOTS.yaw[1];
    const j1Rad = Math.atan2(dyYaw, dxYaw);
    const j1Deg = THREE.MathUtils.clamp((j1Rad * 180) / Math.PI, -90, 90);

    // 2. คำนวณระยะเอื้อมในแนวรัศมี หักระยะหัวคีบจากข้อมือถึงปลายแขนกล (~76.6 mm)
    const radialTotal = Math.hypot(dxYaw, dyYaw);
    const wristRadial = Math.max(25.0, radialTotal - 76.6);
    const wristZ = Math.max(15.0, tz);

    const dr = wristRadial;
    const dz = wristZ - PIVOTS.shoulder[2]; // เทียบกับความสูงแกนไหล่ (64 mm)
    const rawDist = Math.hypot(dr, dz);
    const dist = THREE.MathUtils.clamp(rawDist, 28.0, L1 + L2 - 2.0); // ระยะเอื้อมปลอดภัย 28..158 mm

    // 3. ใช้กฎของโคไซน์ (Law of Cosines) หามุมแขนท่อนบนแบบศอกยกขึ้น (Elbow-up)
    const phi = Math.atan2(dr, dz);
    const cosA = THREE.MathUtils.clamp((L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist), -1, 1);
    const alpha1 = phi - Math.acos(cosA);

    // 4. คำนวณเวกเตอร์แขนท่อนล่าง (Elbow -> Wrist) เพื่อหามุมก้มของศอก
    const elR = L1 * Math.sin(alpha1);
    const elZ = L1 * Math.cos(alpha1);
    const scale = dist / Math.max(1e-5, rawDist);
    const fwR = dr * scale - elR;
    const fwZ = dz * scale - elZ;
    const alpha2 = Math.atan2(-fwZ, fwR);

    const j2Deg = THREE.MathUtils.clamp(((alpha1 - HOME_ALPHA1) * 180) / Math.PI, -45, 65);
    const j3Deg = THREE.MathUtils.clamp(((alpha2 - HOME_ALPHA2) * 180) / Math.PI, -55, 55);

    return { j1: j1Deg, j2: j2Deg, j3: j3Deg };
  }

  /* =========================================================================
   * SECTION 08: การผูกตัวควบคุม UI Sliders, ปุ่มแสดงผล และป้ายตัวเลข HUD
   * ========================================================================= */

  const sliderJ1 = document.getElementById('slider-j1');
  const sliderJ2 = document.getElementById('slider-j2');
  const sliderJ3 = document.getElementById('slider-j3');
  const sliderJ4 = document.getElementById('slider-j4');
  const sliderWrist = document.getElementById('slider-wrist');

  /**
   * อัปเดตตำแหน่งแถบเลื่อน (Sliders) บนหน้าจอให้ตรงกับค่าเป้าหมาย `state.target`
   */
  function syncSlidersFromState() {
    sliderJ1.value = state.target.j1;
    sliderJ2.value = state.target.j2;
    sliderJ3.value = state.target.j3;
    sliderJ4.value = state.target.j4;
    sliderWrist.value = state.target.wrist;
    updateJointLabels();
  }

  /**
   * อัปเดตป้ายตัวเลของศาข้อต่อและค่า PWM (0..180°) บนแผงควบคุมด้านขวา
   */
  function updateJointLabels() {
    const pwm1 = Math.round(90 + state.current.j1);
    const pwm2 = Math.round(90 + state.current.j2);
    const pwm3 = Math.round(90 + state.current.j3);
    ui.valJ1.textContent = `${state.current.j1.toFixed(1)}° (PWM ${pwm1}°)`;
    ui.valJ2.textContent = `${state.current.j2.toFixed(1)}° (PWM ${pwm2}°)`;
    ui.valJ3.textContent = `${state.current.j3.toFixed(1)}° (PWM ${pwm3}°)`;
    ui.valJ4.textContent = `${Math.round(state.current.j4)}% Open`;
    ui.valWrist.textContent =
      Math.abs(state.current.wrist) < 0.3
        ? '0.0° (4-Bar Level)'
        : `${state.current.wrist.toFixed(1)}°`;
  }

  // ผูก Event ของตัวเลื่อนข้อต่อทั้ง 5 ช่องเข้ากับ state.target
  [
    [sliderJ1, 'j1'],
    [sliderJ2, 'j2'],
    [sliderJ3, 'j3'],
    [sliderJ4, 'j4'],
    [sliderWrist, 'wrist']
  ].forEach(([el, key]) => {
    el.addEventListener('input', () => {
      stopAutomation();
      state.target[key] = parseFloat(el.value);
    });
  });

  document.getElementById('btn-reset-home').addEventListener('click', () => {
    stopAutomation();
    state.target = { ...HOME_POSE };
    syncSlidersFromState();
  });

  // ตัวเลื่อนความนุ่มนวล (EMA Smoothing Filter)
  const sliderSmoothing = document.getElementById('slider-smoothing');
  sliderSmoothing.addEventListener('input', () => {
    state.smoothing = parseFloat(sliderSmoothing.value);
    document.getElementById('val-smoothing').textContent = state.smoothing.toFixed(2);
  });

  // ตัวเลื่อนแยกชิ้นส่วน (Exploded View)
  const sliderExplode = document.getElementById('slider-explode');
  sliderExplode.addEventListener('input', () => {
    const pct = parseFloat(sliderExplode.value);
    document.getElementById('val-explode').textContent = `${pct}%`;
    const factor = (pct / 100) * 0.65;
    bodyMeshes.forEach((m) => {
      const c = m.userData.center;
      m.position.set(c.x * factor * 0.25, c.y * factor * 1.4, (c.z - 60) * factor * 0.35);
    });
  });

  // ปุ่มสลับการแสดงผล (เส้นขอบ CAD, แกนหมุน Joint, โหมด X-Ray)
  const btnEdges = document.getElementById('btn-toggle-edges');
  btnEdges.addEventListener('click', () => {
    state.showEdges = !state.showEdges;
    btnEdges.classList.toggle('active', state.showEdges);
    edgeLines.forEach((l) => (l.visible = state.showEdges));
  });

  const btnPivots = document.getElementById('btn-toggle-pivots');
  btnPivots.addEventListener('click', () => {
    state.showPivots = !state.showPivots;
    btnPivots.classList.toggle('active', state.showPivots);
    pivotMarkers.forEach((p) => (p.visible = state.showPivots));
  });

  const btnXray = document.getElementById('btn-toggle-xray');
  btnXray.addEventListener('click', () => {
    state.xrayMode = !state.xrayMode;
    btnXray.classList.toggle('active', state.xrayMode);
    bodyMeshes.forEach((m) => {
      m.material.transparent = state.xrayMode;
      m.material.opacity = state.xrayMode ? 0.42 : 1.0;
    });
  });

  // ตัวเลือกโหมดควบคุมด้วยมือและปุ่มกลับด้านกล้อง (Mirror)
  const selectHandMode = document.getElementById('select-hand-mode');
  selectHandMode.addEventListener('change', () => {
    state.handMode = selectHandMode.value;
    ui.handModeTag.textContent = state.handMode === 'ik' ? 'MODE: IK 3D' : 'MODE: DIRECT 4-DOF';
  });

  const btnMirror = document.getElementById('btn-mirror-cam');
  btnMirror.addEventListener('click', () => {
    state.mirrorCam = !state.mirrorCam;
    btnMirror.classList.toggle('active', state.mirrorCam);
    btnMirror.textContent = state.mirrorCam ? '🪞 Mirror: ON' : '🪞 Mirror: OFF';
  });

  /* =========================================================================
   * SECTION 09: โหมดจำลองอัตโนมัติ (Demo), บันทึกท่าทาง (Recorder) & ส่งออก JSON
   * ========================================================================= */

  ui.btnDemoPick.addEventListener('click', () => {
    const nextMode = state.demoMode === 'pick' ? null : 'pick';
    stopAutomation();
    state.demoMode = nextMode;
    state.demoTime = 0;
    ui.btnDemoPick.classList.toggle('active', state.demoMode === 'pick');
  });

  ui.btnDemoCircle.addEventListener('click', () => {
    const nextMode = state.demoMode === 'circle' ? null : 'circle';
    stopAutomation();
    state.demoMode = nextMode;
    state.demoTime = 0;
    ui.btnDemoCircle.classList.toggle('active', state.demoMode === 'circle');
  });

  document.getElementById('btn-reset-cube').addEventListener('click', () => {
    cubeGrabbed = false;
    targetCube.position.copy(CUBE_HOME);
    targetCube.rotation.set(0, 0, 0);
  });

  ui.btnRecord.addEventListener('click', () => {
    state.recording = !state.recording;
    if (state.recording) {
      if (state.playing) {
        state.playing = false;
        ui.btnPlay.classList.remove('active');
      }
      state.recordedFrames = [];
      state.recordStartTime = performance.now();
      ui.btnRecord.classList.add('danger');
      ui.btnRecord.textContent = '⏹️ หยุดบันทึก (Stop)';
    } else {
      ui.btnRecord.classList.remove('danger');
      ui.btnRecord.textContent = `⏺️ บันทึก (${state.recordedFrames.length}f)`;
    }
  });

  ui.btnPlay.addEventListener('click', () => {
    if (state.recordedFrames.length === 0) {
      alert('ยังไม่มีข้อมูลท่าทางที่บันทึกไว้ กดปุ่ม Record แล้วขยับแขนกลก่อนครับ');
      return;
    }
    const nextPlay = !state.playing;
    stopAutomation();
    if (nextPlay && state.recording) {
      state.recording = false;
      ui.btnRecord.classList.remove('danger');
      ui.btnRecord.textContent = `⏺️ บันทึก (${state.recordedFrames.length}f)`;
    }
    state.playing = nextPlay;
    state.playIndex = 0;
    state.playElapsed = 0;
    ui.btnPlay.classList.toggle('active', state.playing);
  });

  document.getElementById('btn-export-traj').addEventListener('click', () => {
    const exportData = {
      model: 'Robotic_Arm_4DOF_Assembly.sc',
      timestamp: new Date().toISOString(),
      current_pose: state.current,
      recorded_frames: state.recordedFrames
    };
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'robotic_arm_4dof_trajectory.json';
    a.click();
    URL.revokeObjectURL(url);
  });

  /* =========================================================================
   * SECTION 10: การสื่อสารฮาร์ดแวร์ IoT ผ่าน USB WebSerial API
   * ========================================================================= */

  /**
   * ตัดการเชื่อมต่อพอร์ต USB WebSerial อย่างปลอดภัย ปลดล็อก Writer และรีเซ็ตสถานะปุ่มบน UI
   * (ใช้ร่วมกันทั้งกรณีผู้ใช้กดตัดการเชื่อมต่อเองและกรณีเกิด Error ระหว่างส่งข้อมูล)
   * @returns {Promise<void>}
   */
  async function disconnectSerial() {
    try {
      if (state.serialWriter) state.serialWriter.releaseLock();
    } catch (_) { /* ข้ามข้อผิดพลาดหาก Writer ถูกปล่อยไปแล้ว */ }
    try {
      if (state.serialPort) await state.serialPort.close();
    } catch (_) { /* ข้ามข้อผิดพลาดหากพอร์ตถูกถอดสายไปแล้ว */ }
    state.serialWriter = null;
    state.serialPort = null;
    state.serialSending = false;
    ui.serialDot.className = 'dot';
    ui.serialStatusText.textContent = 'IoT Serial: Disconnected';
    ui.btnSerial.textContent = '🔌 เชื่อมต่อ Arduino / ESP32 (WebSerial)';
  }

  if ('serial' in navigator) {
    navigator.serial.addEventListener('disconnect', (event) => {
      if (state.serialPort && event.target === state.serialPort) {
        console.warn('USB Serial device physically disconnected');
        disconnectSerial();
      }
    });
  }

  ui.btnSerial.addEventListener('click', async () => {
    if (!('serial' in navigator)) {
      alert('เบราว์เซอร์นี้ไม่รองรับ WebSerial API (แนะนำให้เปิดด้วย Chrome หรือ Edge บนโน้ตบุ๊ก)');
      return;
    }
    try {
      if (state.serialPort) {
        await disconnectSerial();
        return;
      }
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      state.serialPort = port;
      state.serialWriter = port.writable.getWriter();
      ui.serialDot.className = 'dot active';
      ui.serialStatusText.textContent = 'IoT Serial: 115200 Connected';
      ui.btnSerial.textContent = '🔌 ตัดการเชื่อมต่อ Serial';
    } catch (err) {
      console.warn('WebSerial connection cancelled or failed:', err);
    }
  });

  const serialEncoder = new TextEncoder();

  /**
   * ส่งค่าองศา Servo ปัจจุบันออกทางพอร์ต USB WebSerial ทุก ~40 ms (~25 Hz)
   * ในรูปแบบข้อความ ASCII CSV "S1,S2,S3,S4\n" (ช่วง 0..180 องศา)
   * เพื่อส่งต่อไปยังเฟิร์มแวร์ `arduino_servo_bridge/arduino_servo_bridge.ino`
   * @param {number} now - ค่าเวลาความละเอียดสูง (ms) จาก requestAnimationFrame
   * @returns {Promise<void>}
   */
  async function sendSerialTelemetry(now) {
    if (!state.serialWriter || state.serialSending || now - state.lastSerialSend < 40) return;
    state.lastSerialSend = now;
    state.serialSending = true;
    const s1 = Math.round(THREE.MathUtils.clamp(90 + state.current.j1, 0, 180));
    const s2 = Math.round(THREE.MathUtils.clamp(90 + state.current.j2, 0, 180));
    const s3 = Math.round(THREE.MathUtils.clamp(90 + state.current.j3, 0, 180));
    const s4 = Math.round(THREE.MathUtils.clamp((state.current.j4 / 100) * 180, 0, 180));
    const msg = `${s1},${s2},${s3},${s4}\n`;
    try {
      await state.serialWriter.write(serialEncoder.encode(msg));
    } catch (e) {
      console.warn('Serial write error, disconnecting:', e);
      await disconnectSerial();
    } finally {
      state.serialSending = false;
    }
  }

  /* =========================================================================
   * SECTION 11: ระบบตรวจจับมือผ่านกล้องเว็บแคม (MediaPipe Hands)
   * ========================================================================= */

  const videoEl = document.getElementById('webcam-video');
  const trackCanvas = document.getElementById('tracking-canvas');
  const trackCtx = trackCanvas.getContext('2d');
  const camOverlay = document.getElementById('camera-overlay');
  const btnToggleCam = document.getElementById('btn-toggle-cam');
  const btnStartCamOverlay = document.getElementById('btn-start-cam-overlay');

  let handsInstance = null;
  let mediaStream = null;
  let animCamActive = false;

  /**
   * ฟังก์ชัน Callback ของ MediaPipe Hands ที่ทำงานทุกเฟรมจากกล้องเว็บแคม
   * วาดภาพกล้องและโครงกระดูกมือ 21 จุดลงบน `#tracking-canvas`, คำนวณจุดศูนย์กลางฝ่ามือ
   * (X, Y), ขนาดมือเพื่อประมาณระยะลึก (Reach) และระยะจีบนิ้วโป้ง-นิ้วชี้ (Pinch)
   * จากนั้นอัปเดตค่าเป้าหมายลงใน `state.target` ตามโหมดที่เลือก (IK หรือ Direct)
   * @param {Object} results - ผลลัพธ์จาก MediaPipe Hands ประกอบด้วย `image` และ `multiHandLandmarks`
   */
  function onHandResults(results) {
    trackCtx.save();
    trackCtx.clearRect(0, 0, trackCanvas.width, trackCanvas.height);

    if (state.mirrorCam) {
      trackCtx.translate(trackCanvas.width, 0);
      trackCtx.scale(-1, 1);
    }

    // วาดภาพเฟรมจากกล้องเว็บแคม
    if (results.image) {
      trackCtx.drawImage(results.image, 0, 0, trackCanvas.width, trackCanvas.height);
    }

    if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
      const lm = results.multiHandLandmarks[0];
      ui.handDetectTag.textContent = 'HAND LOCKED';
      ui.handDetectTag.style.borderColor = '#10b981';
      ui.handDetectTag.style.color = '#6ee7b7';

      // วาดเส้นโครงกระดูกและจุด Landmarks 21 จุดบนมือ
      if (window.drawConnectors && window.HAND_CONNECTIONS) {
        window.drawConnectors(trackCtx, lm, window.HAND_CONNECTIONS, {
          color: '#0ea5e9',
          lineWidth: 3
        });
        window.drawLandmarks(trackCtx, lm, {
          color: '#f8fafc',
          fillColor: '#10b981',
          lineWidth: 1,
          radius: 4
        });
      }

      // 1. จุดศูนย์กลางฝ่ามือ (เฉลี่ยจากข้อมือ #0 และโคนนิ้วชี้ #5, กลาง #9, ก้อย #17)
      const rawPalmX = (lm[0].x + lm[5].x + lm[9].x + lm[17].x) / 4;
      const palmX = state.mirrorCam ? 1.0 - rawPalmX : rawPalmX;
      const palmY = (lm[0].y + lm[5].y + lm[9].y + lm[17].y) / 4;

      // 2. ขนาดฝ่ามือในภาพ 2 มิติ (ระยะ #0 ถึง #9) ใช้ประมาณระยะใกล้-ไกล (Reach)
      const handScale = Math.hypot(lm[9].x - lm[0].x, lm[9].y - lm[0].y);

      // 3. ระยะจีบนิ้วโป้ง (#4) กับนิ้วชี้ (#8) หารด้วยขนาดมือเพื่อให้คงที่ทุกระยะ
      const pinchDist = Math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y);
      const normPinch = pinchDist / Math.max(0.08, handScale);
      const gripperOpenPct = THREE.MathUtils.clamp(((normPinch - 0.22) / 0.85) * 100, 0, 100);

      // วาดเส้นแสดงระยะจีบนิ้วบน Canvas
      trackCtx.beginPath();
      trackCtx.moveTo(lm[4].x * trackCanvas.width, lm[4].y * trackCanvas.height);
      trackCtx.lineTo(lm[8].x * trackCanvas.width, lm[8].y * trackCanvas.height);
      trackCtx.strokeStyle = gripperOpenPct < 28 ? '#f43f5e' : '#10b981';
      trackCtx.lineWidth = 4;
      trackCtx.stroke();

      // อัปเดตแถบ Pinch Meter บนหน้าเว็บ
      ui.pinchBar.style.width = `${gripperOpenPct}%`;
      ui.pinchValText.textContent =
        gripperOpenPct < 28 ? `GRAB (${Math.round(gripperOpenPct)}%)` : `${Math.round(gripperOpenPct)}%`;

      // แปลงค่าจากมือเป็นองศาเป้าหมาย 4-DOF
      stopAutomation();
      if (state.handMode === 'ik') {
        const targetY = ((0.5 - palmX) / 0.38) * 115.0;
        const targetZ = THREE.MathUtils.clamp(20.0 + (0.85 - palmY) * 230.0, 14.0, 200.0);
        const targetX = THREE.MathUtils.clamp(95.0 + ((handScale - 0.14) / 0.18) * 95.0, 80.0, 195.0);

        const ikSol = solveIK(targetX, targetY, targetZ);
        state.target.j1 = ikSol.j1;
        state.target.j2 = ikSol.j2;
        state.target.j3 = ikSol.j3;
        state.target.j4 = gripperOpenPct;
      } else {
        // โหมดควบคุมข้อต่อโดยตรง (Direct Joint Mapping)
        state.target.j1 = THREE.MathUtils.clamp((0.5 - palmX) * 190, -90, 90);
        state.target.j2 = THREE.MathUtils.clamp((palmY - 0.45) * 140, -45, 65);
        // คำนวณมุมเอียงสันมือเทียบแนวราบด้วย |dx| เพื่อไม่ให้มุมกระโดด ±180° ไม่ว่าจะใช้มือซ้ายหรือขวา
        const tiltRad = Math.atan2(lm[17].y - lm[5].y, Math.abs(lm[17].x - lm[5].x) + 1e-5);
        state.target.j3 = THREE.MathUtils.clamp(((tiltRad * 180) / Math.PI) * 1.15, -55, 55);
        state.target.j4 = gripperOpenPct;
      }
      syncSlidersFromState();
    } else {
      ui.handDetectTag.textContent = 'NO HAND';
      ui.handDetectTag.style.borderColor = 'rgba(56, 189, 248, 0.35)';
      ui.handDetectTag.style.color = '#e2e8f0';
    }

    trackCtx.restore();
  }

  /**
   * ปิดสตรีมกล้องเว็บแคมและคืนสถานะปุ่ม/ไฟแสดงผลทั้งหมดอย่างปลอดภัย
   */
  function stopWebcam(statusText = 'Camera: Standby', dotClass = 'dot') {
    state.camRunning = false;
    animCamActive = false;
    if (mediaStream) {
      mediaStream.getTracks().forEach((t) => t.stop());
      mediaStream = null;
    }
    videoEl.srcObject = null;
    camOverlay.style.display = 'flex';
    ui.camDot.className = dotClass;
    ui.camStatusText.textContent = statusText;
    btnToggleCam.textContent = '📷 เปิดกล้อง Hand Tracking';
    btnToggleCam.classList.remove('danger');
  }

  /**
   * เปิดหรือปิดสตรีมกล้องเว็บแคม (`getUserMedia`) และเริ่มลูปประมวลผล MediaPipe Hands
   * โดยทำงานในเบราว์เซอร์ 100% ไม่มีการส่งภาพวิดีโอออกไปยังเซิร์ฟเวอร์ภายนอก
   * @returns {Promise<void>}
   */
  async function toggleWebcam() {
    if (state.camInitializing) return;

    if (state.camRunning) {
      stopWebcam();
      return;
    }

    if (typeof window.Hands === 'undefined') {
      ui.camDot.className = 'dot warn';
      ui.camStatusText.textContent = 'Camera: MediaPipe CDN Offline';
      alert('ไม่สามารถโหลด MediaPipe Hands จาก CDN ได้ กรุณาตรวจสอบอินเทอร์เน็ตแล้วลองใหม่');
      return;
    }

    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      ui.camDot.className = 'dot warn';
      ui.camStatusText.textContent = 'Camera: Unsupported Context';
      alert('เบราว์เซอร์ไม่อนุญาตให้เข้าถึงกล้อง กรุณาเปิดผ่าน http://localhost:8080 หรือ HTTPS');
      return;
    }

    state.camInitializing = true;
    try {
      ui.camDot.className = 'dot warn';
      ui.camStatusText.textContent = 'Camera: Initializing AI...';

      if (!handsInstance) {
        handsInstance = new window.Hands({
          locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4/${file}`
        });
        handsInstance.setOptions({
          maxNumHands: 1,
          modelComplexity: 1,
          minDetectionConfidence: 0.65,
          minTrackingConfidence: 0.65
        });
        handsInstance.onResults(onHandResults);
      }

      mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { width: 640, height: 480, facingMode: 'user' },
        audio: false
      });
      videoEl.srcObject = mediaStream;
      await videoEl.play();

      state.camRunning = true;
      animCamActive = true;
      camOverlay.style.display = 'none';
      ui.camDot.className = 'dot active';
      ui.camStatusText.textContent = 'Camera: Live Hand Tracking';
      btnToggleCam.textContent = '⏹️ ปิดกล้อง (Stop Camera)';
      btnToggleCam.classList.add('danger');

      async function processVideoFrame() {
        if (!animCamActive) return;
        try {
          if (videoEl.readyState >= 2) {
            await handsInstance.send({ image: videoEl });
          }
        } catch (err) {
          console.error('Hand tracking frame error:', err);
          stopWebcam('Camera: Tracking Error', 'dot warn');
          return;
        }
        if (animCamActive) {
          requestAnimationFrame(processVideoFrame);
        }
      }
      requestAnimationFrame(processVideoFrame);
    } catch (err) {
      console.error('Webcam error:', err);
      stopWebcam('Camera: Permission/Error', 'dot warn');
      alert(
        'ไม่สามารถเปิดกล้องเว็บแคมได้: ' +
        (err.message || err) +
        '\nตรวจสอบว่าอนุญาตสิทธิ์การใช้กล้องในเบราว์เซอร์แล้ว'
      );
    } finally {
      state.camInitializing = false;
    }
  }

  btnToggleCam.addEventListener('click', toggleWebcam);
  btnStartCamOverlay.addEventListener('click', toggleWebcam);

  /* =========================================================================
   * SECTION 12: ลูปเรนเดอร์และจำลองหลัก 60 FPS (Main Loop)
   * ========================================================================= */

  let lastTime = performance.now();
  let frameCounter = 0;
  let fpsTimer = performance.now();

  /**
   * ลูปหลักสำหรับจำลองและเรนเดอร์ฉาก 3 มิติที่ 60 FPS
   * ลำดับการทำงานในแต่ละเฟรม:
   *   1. อัปเดตตัวเลข FPS บนหน้าจอ
   *   2. คำนวณท่าทางจากโหมดอัตโนมัติ (Pick & Place, วาดวงกลม IK) หรือเล่นซ้ำท่าทางที่บันทึกไว้
   *   3. บันทึกเฟรมปัจจุบันหากกำลังเปิดโหมด Record
   *   4. กรองความนุ่มนวลด้วย Exponential Moving Average (EMA): target -> current
   *   5. อัปเดตป้ายตัวเลข HUD, คำนวณ 4-Bar FK (`updateArmKinematics`), ส่งค่า WebSerial,
   *      อัปเดต OrbitControls และเรนเดอร์ภาพด้วย Three.js WebGLRenderer
   * @param {number} now - ค่าเวลาความละเอียดสูง (ms)
   */
  function animate(now) {
    requestAnimationFrame(animate);
    const dt = Math.min(0.05, (now - lastTime) / 1000);
    lastTime = now;

    // 1. คำนวณและแสดงอัตราเฟรม (FPS) ทุก 500 ms
    frameCounter++;
    if (now - fpsTimer >= 500) {
      const fps = Math.round((frameCounter * 1000) / (now - fpsTimer));
      ui.fpsBadge.textContent = `${fps} FPS`;
      frameCounter = 0;
      fpsTimer = now;
    }

    // 2. ประมวลผลโหมดจำลองอัตโนมัติหรือเล่นซ้ำท่าทางที่บันทึกไว้
    if (state.demoMode === 'pick') {
      state.demoTime += dt;
      const t = state.demoTime % 8.0;
      if (t < 1.5) {
        const sol = solveIK(135, 45, 65);
        state.target = { ...sol, j4: 65, wrist: 0 };
      } else if (t < 2.8) {
        const sol = solveIK(135, 45, 16);
        state.target = { ...sol, j4: 60, wrist: 0 };
      } else if (t < 3.6) {
        const sol = solveIK(135, 45, 16);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 5.2) {
        const frac = (t - 3.6) / 1.6;
        const y = 45 - frac * 100;
        const z = 16 + Math.sin(frac * Math.PI) * 75;
        const sol = solveIK(135, y, z);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 6.2) {
        const sol = solveIK(135, -55, 18);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 7.0) {
        const sol = solveIK(135, -55, 22);
        state.target = { ...sol, j4: 65, wrist: 0 };
      } else {
        if (!cubeGrabbed) targetCube.position.copy(CUBE_HOME);
        const sol = solveIK(140, 0, 110);
        state.target = { ...sol, j4: 45, wrist: 0 };
      }
      syncSlidersFromState();
    } else if (state.demoMode === 'circle') {
      state.demoTime += dt * 1.8;
      const tx = 135 + 25 * Math.cos(state.demoTime);
      const ty = 55 * Math.sin(state.demoTime);
      const tz = 95 + 35 * Math.sin(state.demoTime * 2);
      const sol = solveIK(tx, ty, tz);
      state.target.j1 = sol.j1;
      state.target.j2 = sol.j2;
      state.target.j3 = sol.j3;
      state.target.j4 = 35 + 25 * Math.sin(state.demoTime * 3);
      syncSlidersFromState();
    } else if (state.playing && state.recordedFrames.length > 0) {
      const lastFrame = state.recordedFrames[state.recordedFrames.length - 1];
      if (typeof lastFrame.t === 'number' && lastFrame.t > 0) {
        state.playElapsed = (state.playElapsed + dt * 1000) % (lastFrame.t + 16.67);
        while (
          state.playIndex + 1 < state.recordedFrames.length &&
          state.recordedFrames[state.playIndex + 1].t <= state.playElapsed
        ) {
          state.playIndex++;
        }
        if (state.recordedFrames[state.playIndex].t > state.playElapsed) {
          state.playIndex = 0;
        }
      } else {
        state.playIndex = (state.playIndex + 1) % state.recordedFrames.length;
      }
      const f = state.recordedFrames[state.playIndex];
      state.target = { j1: f.j1, j2: f.j2, j3: f.j3, j4: f.j4, wrist: f.wrist };
      syncSlidersFromState();
    }

    // 3. บันทึกเฟรมปัจจุบันพร้อมเวลา (ms) หากกำลังเปิดโหมด Record
    if (state.recording) {
      state.recordedFrames.push({ ...state.current, t: Math.round(now - state.recordStartTime) });
      ui.btnRecord.textContent = `⏹️ หยุด (${state.recordedFrames.length}f)`;
    }

    // 4. กรองความนุ่มนวลด้วย Exponential Moving Average (EMA) แบบอิสระจากอัตราเฟรมจอ (60Hz/120Hz/144Hz)
    const alpha = 1 - Math.pow(1 - state.smoothing, dt * 60);
    ['j1', 'j2', 'j3', 'j4', 'wrist'].forEach((k) => {
      state.current[k] += (state.target[k] - state.current[k]) * alpha;
    });

    // 5. อัปเดต HUD, จลนศาสตร์ 3 มิติ, ส่งคำสั่ง Serial และเรนเดอร์ฉาก
    updateJointLabels();
    updateArmKinematics();
    sendSerialTelemetry(now);

    controls.update();
    renderer.render(scene, camera);
  }

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // คำนวณท่าเริ่มต้นครั้งแรกและเริ่มลูปหลัก
  updateArmKinematics();
  requestAnimationFrame(animate);
})();
