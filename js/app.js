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
  const toastContainerEl = document.getElementById('toast-container');

  /**
   * แสดงข้อความแจ้งเตือนแบบ Non-Blocking Toast (แทน alert() ที่บล็อกการทำงาน)
   * @param {string} message - ข้อความแจ้งเตือน
   * @param {'info'|'success'|'warn'|'error'} [type='info'] - ระดับสีของข้อความ
   * @param {number} [duration=4200] - ระยะเวลาแสดงผล (ms)
   */
  function showToast(message, type = 'info', duration = 4200) {
    if (!toastContainerEl) {
      console.warn(`[${type}] ${message}`);
      return;
    }
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    toastContainerEl.appendChild(toast);
    setTimeout(() => {
      toast.style.opacity = '0';
      toast.style.transition = 'opacity 0.25s ease';
      setTimeout(() => toast.remove(), 260);
    }, duration);
  }

  // ผูกปุ่มเมนู 3 ขีด (Navigation Drawer) ก่อนตรวจสอบ Three.js เพื่อให้สลับหน้าได้แม้เปิดแบบออฟไลน์
  let isHardwareActiveGuard = () => false;
  if (window.ArmFactoryBus && window.ArmFactoryBus.initNavigationDrawer) {
    window.ArmFactoryBus.initNavigationDrawer({
      windowName: 'arm4dof_control',
      isHardwareActive: () => isHardwareActiveGuard(),
      onAutoPopout: (label) => {
        showToast(`เปิด ${label} ในแท็บแยกอัตโนมัติเพื่อไม่ให้กล้องหรือ USB WebSerial หลุด`, 'info', 3500);
      }
    });
  }

  if (typeof THREE === 'undefined') {
    if (cadDotEl) cadDotEl.className = 'dot warn';
    if (cadStatusEl) cadStatusEl.textContent = '3D Engine: CDN Offline';
    showToast('ไม่สามารถโหลดไลบรารี Three.js จาก CDN ได้ กรุณาตรวจสอบอินเทอร์เน็ตแล้วรีเฟรชหน้าเว็บ', 'error', 7000);
    return;
  }

  if (!cadData || !Array.isArray(cadData.bodies)) {
    if (cadDotEl) cadDotEl.className = 'dot warn';
    if (cadStatusEl) cadStatusEl.textContent = 'CAD Mesh: Missing';
    showToast('ไม่พบข้อมูลโมเดล CAD (js/arm_cad_data.js)', 'error', 7000);
    return;
  }

  if (cadDotEl) cadDotEl.className = 'dot active';
  if (cadStatusEl) cadStatusEl.textContent = `CAD Mesh: ${cadData.bodies.length}/${cadData.bodies.length} Loaded`;

  // เชื่อมต่อระบบบัสข้อมูลสดข้ามหน้าต่าง (BroadcastChannel + localStorage Snapshot)
  const factoryBus = window.ArmFactoryBus ? window.ArmFactoryBus.createBus('station') : null;
  const stationId = `station-${Math.random().toString(36).slice(2, 8)}`;
  const stationStartMs = Date.now();
  const factoryMetrics = {
    fps: 60,
    telemetryHz: 10,
    cycleCount: 0,
    tcp: { x: 162.0, y: 0.0, z: 129.0 },
    handDetected: false,
    pinchPct: 50,
    estopMs: 0,
    lastBusPublish: 0,
    lastBusSampleMs: 0,
    busRateWindowStart: performance.now(),
    busRateCount: 0,
    prevLimitClamped: { j1: false, j2: false, j3: false, j4: false },
    serial: {
      txCount: 0,
      txHz: 0,
      txWindowStart: performance.now(),
      txWindowCount: 0,
      lastPacket: '',
      lastLimitCmd: '',
      lastHandshake: 'Waiting',
      limitsAcked: false,
      recentPackets: []
    }
  };

  function logFactoryEvent(level, category, message, detail = '') {
    if (factoryBus) {
      factoryBus.pushEvent(level, category, message, detail);
    }
  }

  function recordSerialPacket(dir, text) {
    const clean = String(text || '').trim();
    if (!clean) return;
    factoryMetrics.serial.recentPackets.unshift({
      ts: Date.now(),
      dir,
      text: clean
    });
    if (factoryMetrics.serial.recentPackets.length > 36) {
      factoryMetrics.serial.recentPackets.length = 36;
    }
  }

  // โหลดจำนวน Cycle Count เดิมจาก Snapshot ล่าสุด (ถ้ามี) เพื่อคงยอดผลิตในเซสชัน
  (function restorePreviousCycleCount() {
    if (!window.ArmFactoryBus) return;
    const prev = window.ArmFactoryBus.readSnapshot();
    if (prev && prev.production && typeof prev.production.cycleCount === 'number' && prev.production.cycleCount >= 0) {
      factoryMetrics.cycleCount = Math.floor(prev.production.cycleCount);
    }
  })();

  logFactoryEvent('success', 'SYSTEM', 'สถานีควบคุม 3D Digital Twin พร้อมทำงาน', `โหลดชิ้นส่วน CAD ${cadData.bodies.length} ชิ้น`);

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
    pinchBarTrack: document.getElementById('pinch-bar-track'),
    pinchValText: document.getElementById('pinch-val-text'),
    handModeTag: document.getElementById('hand-mode-tag'),
    handDetectTag: document.getElementById('hand-detect-tag'),
    camDot: document.getElementById('cam-dot'),
    camStatusText: document.getElementById('cam-status-text'),
    serialDot: document.getElementById('serial-dot'),
    serialStatusText: document.getElementById('serial-status-text'),
    serialHelperText: document.getElementById('serial-helper-text'),
    btnSerial: document.getElementById('btn-connect-serial'),
    btnDemoPick: document.getElementById('btn-demo-pick'),
    btnDemoCircle: document.getElementById('btn-demo-circle'),
    btnRecord: document.getElementById('btn-record-pose'),
    btnPlay: document.getElementById('btn-play-pose'),
    btnToggleHud: document.getElementById('btn-toggle-hud'),
    edgeTabLeft: document.getElementById('edge-tab-left'),
    edgeTabRight: document.getElementById('edge-tab-right'),
    btnResetCam: document.getElementById('btn-reset-cam'),
    btnEstop: document.getElementById('btn-estop'),
    btnOpenSettings: document.getElementById('btn-open-settings'),
    settingsBackdrop: document.getElementById('settings-backdrop'),
    settingsModal: document.getElementById('settings-modal'),
    btnCloseSettings: document.getElementById('btn-close-settings'),
    btnDoneSettings: document.getElementById('btn-done-settings'),
    btnResetDisplaySettings: document.getElementById('btn-reset-display-settings'),
    valSettingTheme: document.getElementById('val-setting-theme'),
    sliderSceneBrightness: document.getElementById('slider-scene-brightness'),
    valSceneBrightness: document.getElementById('val-scene-brightness'),
    sliderPanelOpacity: document.getElementById('slider-panel-opacity'),
    valPanelOpacity: document.getElementById('val-panel-opacity'),
    toggleGlassBlur: document.getElementById('toggle-glass-blur'),
    valSettingScale: document.getElementById('val-setting-scale'),
    tabDisplay: document.getElementById('tab-display'),
    tabLimits: document.getElementById('tab-limits'),
    tabpanelDisplay: document.getElementById('tabpanel-display'),
    tabpanelLimits: document.getElementById('tabpanel-limits'),
    btnResetJointLimits: document.getElementById('btn-reset-joint-limits'),
    limitValidationMsg: document.getElementById('limit-validation-msg'),
    ctrlGroups: {
      j1: document.getElementById('ctrl-group-j1'),
      j2: document.getElementById('ctrl-group-j2'),
      j3: document.getElementById('ctrl-group-j3'),
      j4: document.getElementById('ctrl-group-j4')
    },
    hudLimitPills: {
      j1: document.getElementById('hud-limit-j1'),
      j2: document.getElementById('hud-limit-j2'),
      j3: document.getElementById('hud-limit-j3'),
      j4: document.getElementById('hud-limit-j4')
    },
    modalLimitPills: {
      j1: document.getElementById('modal-limit-j1'),
      j2: document.getElementById('modal-limit-j2'),
      j3: document.getElementById('modal-limit-j3'),
      j4: document.getElementById('modal-limit-j4')
    },
    limitCards: {
      j1: document.getElementById('limit-card-j1'),
      j2: document.getElementById('limit-card-j2'),
      j3: document.getElementById('limit-card-j3'),
      j4: document.getElementById('limit-card-j4')
    },
    pwmRangeBadges: {
      j1: document.getElementById('pwm-range-j1'),
      j2: document.getElementById('pwm-range-j2'),
      j3: document.getElementById('pwm-range-j3'),
      j4: document.getElementById('pwm-range-j4')
    },
    limitInputs: {
      j1: { min: document.getElementById('limit-j1-min'), max: document.getElementById('limit-j1-max') },
      j2: { min: document.getElementById('limit-j2-min'), max: document.getElementById('limit-j2-max') },
      j3: { min: document.getElementById('limit-j3-min'), max: document.getElementById('limit-j3-max') },
      j4: { min: document.getElementById('limit-j4-min'), max: document.getElementById('limit-j4-max') }
    }
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
    // สถานะความปลอดภัยฉุกเฉิน (Emergency Stop)
    estop: false,
    // สถานะการเชื่อมต่อพอร์ต USB WebSerial
    serialPort: null,
    serialWriter: null,
    serialReader: null,
    serialReading: false,
    serialSending: false,
    boardReady: false,
    pendingLimitSync: false,
    lastSerialSend: 0
  };

  /**
   * หยุดโหมดอัตโนมัติ (Demo) และการเล่นซ้ำ (Play) พร้อมรีเซ็ตปุ่มสถานะบน UI
   */
  function stopAutomation() {
    state.demoMode = null;
    state.playing = false;
    ui.btnDemoPick.classList.remove('active');
    ui.btnDemoPick.setAttribute('aria-pressed', 'false');
    ui.btnDemoCircle.classList.remove('active');
    ui.btnDemoCircle.setAttribute('aria-pressed', 'false');
    ui.btnPlay.classList.remove('active');
    ui.btnPlay.setAttribute('aria-pressed', 'false');
  }

  /* =========================================================================
   * ระบบพับเก็บแผงควบคุม (Left/Right Panels) และการ์ดเมนูทุกใบ (WAI-ARIA Disclosure)
   * รองรับทั้ง Desktop (พับซ้าย/ขวาอิสระ + โฟกัส 3D) และ Tablet/Mobile (Drawer/Bottom Sheet)
   * ========================================================================= */
  const LAYOUT_STORAGE_KEY = 'arm4dof_ui_layout_v1';

  function loadLayoutPrefs() {
    try {
      const raw = localStorage.getItem(LAYOUT_STORAGE_KEY);
      return raw ? JSON.parse(raw) : {};
    } catch {
      return {};
    }
  }

  function saveLayoutPrefs() {
    try {
      const collapsedCards = {};
      document.querySelectorAll('.collapsible-card[data-card-id]').forEach((card) => {
        collapsedCards[card.dataset.cardId] = card.classList.contains('is-collapsed');
      });
      const prefs = {
        leftCollapsed: document.body.classList.contains('left-collapsed'),
        rightCollapsed: document.body.classList.contains('right-collapsed'),
        collapsedCards
      };
      localStorage.setItem(LAYOUT_STORAGE_KEY, JSON.stringify(prefs));
    } catch {
      // ข้ามหากเบราว์เซอร์ปิดการเข้าถึง localStorage (Private Mode)
    }
  }

  function syncDesktopPanelButtons() {
    const leftCollapsed = document.body.classList.contains('left-collapsed');
    const rightCollapsed = document.body.classList.contains('right-collapsed');
    const allHidden = leftCollapsed && rightCollapsed;
    document.body.classList.toggle('hud-hidden', allHidden);

    if (ui.btnToggleHud) {
      ui.btnToggleHud.setAttribute('aria-pressed', String(allHidden));
      const labelEl = ui.btnToggleHud.querySelector('.btn-icon-label');
      if (labelEl) labelEl.textContent = allHidden ? 'แสดงแผง' : 'ซ่อนแผง';
    }
  }

  function setDesktopPanelCollapsed(side, collapsed, persist = true) {
    const cls = side === 'left' ? 'left-collapsed' : 'right-collapsed';
    document.body.classList.toggle(cls, Boolean(collapsed));
    syncDesktopPanelButtons();
    if (persist) saveLayoutPrefs();
  }

  function toggleDesktopPanel(side) {
    if (window.innerWidth < 1200) {
      const currentlyOpen =
        (side === 'left' && document.body.classList.contains('mobile-panel-left')) ||
        (side === 'right' && document.body.classList.contains('mobile-panel-right'));
      setMobilePanel(currentlyOpen ? 'none' : side);
      return;
    }
    const cls = side === 'left' ? 'left-collapsed' : 'right-collapsed';
    setDesktopPanelCollapsed(side, !document.body.classList.contains(cls));
  }

  function setCardCollapsed(card, collapsed, persist = true) {
    if (!card) return;
    const isCollapsed = Boolean(collapsed);
    card.classList.toggle('is-collapsed', isCollapsed);
    const toggleBtn = card.querySelector('.card-toggle');
    if (toggleBtn) {
      toggleBtn.setAttribute('aria-expanded', String(!isCollapsed));
    }
    const collapseEl = card.querySelector('.card-collapse');
    if (collapseEl) {
      collapseEl.inert = isCollapsed;
    }
    const parentPanel = card.closest('.left-panel, .right-panel');
    if (parentPanel) syncPanelCardsToggleBtn(parentPanel.id);
    if (persist) saveLayoutPrefs();
  }

  function syncPanelCardsToggleBtn(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) return;
    const btn = panel.querySelector(`[data-panel-cards-toggle="${panelId}"]`);
    if (!btn) return;
    const cards = Array.from(panel.querySelectorAll('.collapsible-card'));
    const allCollapsed = cards.length > 0 && cards.every((c) => c.classList.contains('is-collapsed'));
    btn.setAttribute('aria-pressed', String(allCollapsed));
    btn.textContent = allCollapsed ? 'ขยายทั้งหมด' : 'ย่อทั้งหมด';
  }

  function setMobilePanel(panel) {
    document.body.classList.remove('mobile-panel-left', 'mobile-panel-right');
    if (panel === 'left') {
      document.body.classList.add('mobile-panel-left');
    } else if (panel === 'right') {
      document.body.classList.add('mobile-panel-right');
    }
    document.querySelectorAll('.mobile-nav-btn').forEach((btn) => {
      const isActive = btn.dataset.panel === panel;
      btn.classList.toggle('active', isActive);
      btn.setAttribute('aria-pressed', String(isActive));
    });
  }

  // ผูกอีเวนต์ปุ่มสลับแผงบนแถบนำทางล่าง (Tablet / Mobile)
  document.querySelectorAll('.mobile-nav-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      const target = btn.dataset.panel || 'none';
      const alreadyOpen =
        (target === 'left' && document.body.classList.contains('mobile-panel-left')) ||
        (target === 'right' && document.body.classList.contains('mobile-panel-right'));
      setMobilePanel(alreadyOpen ? 'none' : target);
    });
  });

  // ผูกอีเวนต์แถบจับด้านบนของ Bottom Sheet (รองรับคลิกและคีย์บอร์ด)
  document.querySelectorAll('.drawer-handle').forEach((handle) => {
    handle.addEventListener('click', () => setMobilePanel('none'));
    handle.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' || e.key === ' ') {
        e.preventDefault();
        setMobilePanel('none');
      }
    });
  });

  // ผูกอีเวนต์ปุ่มพับ/เปิดแผงซ้าย-ขวา (Panel Toolbar & Edge Tabs)
  if (ui.edgeTabLeft) {
    ui.edgeTabLeft.addEventListener('click', () => setDesktopPanelCollapsed('left', false));
  }
  if (ui.edgeTabRight) {
    ui.edgeTabRight.addEventListener('click', () => setDesktopPanelCollapsed('right', false));
  }
  document.querySelectorAll('[data-panel-collapse]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const side = btn.dataset.panelCollapse;
      if (window.innerWidth < 1200) {
        setMobilePanel('none');
      } else {
        setDesktopPanelCollapsed(side, true);
      }
    });
  });

  // ผูกอีเวนต์ปุ่มซ่อน/แสดงแผงควบคุมทั้งหมดพร้อมกัน
  if (ui.btnToggleHud) {
    ui.btnToggleHud.addEventListener('click', () => {
      const allHidden = document.body.classList.contains('left-collapsed') &&
        document.body.classList.contains('right-collapsed');
      setDesktopPanelCollapsed('left', !allHidden, false);
      setDesktopPanelCollapsed('right', !allHidden, true);
    });
  }

  // ผูกอีเวนต์ปุ่มย่อ/ขยายการ์ดเมนูแต่ละใบ
  document.querySelectorAll('.collapsible-card').forEach((card) => {
    const toggleBtn = card.querySelector('.card-toggle');
    if (toggleBtn) {
      toggleBtn.addEventListener('click', () => {
        setCardCollapsed(card, !card.classList.contains('is-collapsed'));
      });
    }
  });

  // ผูกอีเวนต์ปุ่มย่อ/ขยายการ์ดทั้งหมดภายในแผง
  document.querySelectorAll('[data-panel-cards-toggle]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const panelId = btn.dataset.panelCardsToggle;
      const panel = document.getElementById(panelId);
      if (!panel) return;
      const cards = Array.from(panel.querySelectorAll('.collapsible-card'));
      const shouldCollapse = !cards.every((c) => c.classList.contains('is-collapsed'));
      cards.forEach((c) => setCardCollapsed(c, shouldCollapse, false));
      saveLayoutPrefs();
    });
  });

  // คืนค่าสถานะการพับแผงและการ์ดจากครั้งล่าสุดของผู้ใช้
  (function restoreLayoutPrefs() {
    const prefs = loadLayoutPrefs();
    if (prefs.leftCollapsed) document.body.classList.add('left-collapsed');
    if (prefs.rightCollapsed) document.body.classList.add('right-collapsed');
    if (prefs.collapsedCards && typeof prefs.collapsedCards === 'object') {
      document.querySelectorAll('.collapsible-card[data-card-id]').forEach((card) => {
        const saved = prefs.collapsedCards[card.dataset.cardId];
        if (typeof saved === 'boolean') {
          setCardCollapsed(card, saved, false);
        }
      });
    }
    syncDesktopPanelButtons();
    syncPanelCardsToggleBtn('left-panel');
    syncPanelCardsToggleBtn('right-panel');
  })();

  /* =========================================================================
   * SECTION 02: ตั้งค่าฉาก 3 มิติ Three.js (Camera, Renderer, Lighting, พื้นโต๊ะ)
   * ========================================================================= */

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x070b14);
  scene.fog = new THREE.FogExp2(0x070b14, 0.00115);

  const initWidth = ui.canvasContainer.clientWidth || window.innerWidth;
  const initHeight = ui.canvasContainer.clientHeight || window.innerHeight;

  const DEFAULT_CAM_POS = Object.freeze({ x: 275, y: 225, z: 315 });
  const DEFAULT_CAM_TARGET = Object.freeze({ x: 45, y: 82, z: 0 });

  const camera = new THREE.PerspectiveCamera(
    42,
    initWidth / initHeight,
    1,
    6000
  );
  camera.position.set(DEFAULT_CAM_POS.x, DEFAULT_CAM_POS.y, DEFAULT_CAM_POS.z);

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(initWidth, initHeight, false);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  ui.canvasContainer.appendChild(renderer.domElement);

  renderer.domElement.addEventListener('webglcontextlost', (event) => {
    event.preventDefault();
    showToast('WebGL Context ถูกขัดจังหวะชั่วคราว กำลังพยายามกู้คืนฉาก 3 มิติ...', 'warn', 5000);
  });

  renderer.domElement.addEventListener('webglcontextrestored', () => {
    showToast('กู้คืนฉากแสดงผล 3 มิติสำเร็จแล้ว', 'success', 3000);
  });

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(DEFAULT_CAM_TARGET.x, DEFAULT_CAM_TARGET.y, DEFAULT_CAM_TARGET.z);

  function resetCameraView() {
    camera.position.set(DEFAULT_CAM_POS.x, DEFAULT_CAM_POS.y, DEFAULT_CAM_POS.z);
    controls.target.set(DEFAULT_CAM_TARGET.x, DEFAULT_CAM_TARGET.y, DEFAULT_CAM_TARGET.z);
    controls.update();
  }

  if (ui.btnResetCam) {
    ui.btnResetCam.addEventListener('click', () => {
      resetCameraView();
      showToast('รีเซ็ตมุมกล้อง 3D เรียบร้อยแล้ว', 'info', 2000);
    });
  }
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.maxPolarAngle = Math.PI / 2 - 0.03;
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

  // เส้นตารางโต๊ะทำงานและพื้นรับเงา (ใช้ let เพื่อให้เปลี่ยนสีตารางตามธีม Dark/Light ได้ทันที)
  let gridHelper = new THREE.GridHelper(500, 25, 0x0ea5e9, 0x1e293b);
  gridHelper.position.y = -0.2;
  scene.add(gridHelper);

  // พื้นรับเงาแบบวงกลมขนาดใหญ่กลืนเข้ากับขอบฟ้า (Infinite Studio Horizon) ไม่เห็นขอบตัดเมื่อซูมออก
  const groundMesh = new THREE.Mesh(
    new THREE.CircleGeometry(5000, 64),
    new THREE.MeshStandardMaterial({ color: 0x0b1220, roughness: 0.92, metalness: 0.05 })
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

    // เพิ่มรายการชิ้นส่วนลงในแผง CAD Parts Inspector (ใช้ DOM API ที่ปลอดภัยและรองรับคีย์บอร์ด)
    const item = document.createElement('button');
    item.type = 'button';
    item.className = 'part-item';
    item.setAttribute('role', 'option');
    item.setAttribute('aria-selected', 'false');
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
    triSpan.className = 'part-tri-count';
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
      const isSelected = Number(el.dataset.bodyId) === state.selectedBodyId;
      el.classList.toggle('selected', isSelected);
      el.setAttribute('aria-selected', String(isSelected));
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
   * ระบบตั้งค่าการแสดงผลและธีมสี (Display & Theme Settings — CSS + Three.js Sync)
   * บันทึกอัตโนมัติลง localStorage และเปิดเป็น Modal โดยไม่กระทบกล้อง/WebSerial
   * ========================================================================= */
  const DISPLAY_STORAGE_KEY = 'arm4dof_display_settings_v1';
  const DISPLAY_DEFAULTS = Object.freeze({
    theme: 'dark',      // 'dark' | 'light' | 'auto'
    brightness: 100,    // 40 .. 200 (%)
    panelOpacity: 88,   // 45 .. 100 (%)
    glassBlur: true,    // boolean
    uiScale: 'normal'   // 'compact' | 'normal' | 'large'
  });

  const THEME_LABELS = Object.freeze({
    dark: 'Dark',
    light: 'Light',
    auto: 'Auto (ระบบ)'
  });

  const SCALE_LABELS = Object.freeze({
    compact: 'กะทัดรัด',
    normal: 'ปกติ',
    large: 'ใหญ่'
  });

  const systemColorSchemeMedia = window.matchMedia
    ? window.matchMedia('(prefers-color-scheme: light)')
    : null;

  function loadDisplaySettings() {
    try {
      const raw = localStorage.getItem(DISPLAY_STORAGE_KEY);
      if (!raw) return { ...DISPLAY_DEFAULTS };
      const parsed = JSON.parse(raw);
      return {
        theme: ['dark', 'light', 'auto'].includes(parsed.theme) ? parsed.theme : DISPLAY_DEFAULTS.theme,
        brightness: typeof parsed.brightness === 'number'
          ? THREE.MathUtils.clamp(Math.round(parsed.brightness), 40, 200)
          : DISPLAY_DEFAULTS.brightness,
        panelOpacity: typeof parsed.panelOpacity === 'number'
          ? THREE.MathUtils.clamp(Math.round(parsed.panelOpacity), 45, 100)
          : DISPLAY_DEFAULTS.panelOpacity,
        glassBlur: typeof parsed.glassBlur === 'boolean' ? parsed.glassBlur : DISPLAY_DEFAULTS.glassBlur,
        uiScale: ['compact', 'normal', 'large'].includes(parsed.uiScale)
          ? parsed.uiScale
          : DISPLAY_DEFAULTS.uiScale
      };
    } catch {
      return { ...DISPLAY_DEFAULTS };
    }
  }

  function saveDisplaySettings(settings) {
    try {
      localStorage.setItem(DISPLAY_STORAGE_KEY, JSON.stringify(settings));
    } catch {
      // ข้ามหากเบราว์เซอร์ปิดการเข้าถึง localStorage
    }
    if (factoryBus) factoryBus.notifyThemeChanged();
  }

  let displaySettings = loadDisplaySettings();
  let lastAppliedSceneTheme = null;
  let lastFocusedBeforeSettings = null;

  function resolveEffectiveTheme(themePref) {
    if (themePref === 'auto') {
      return (systemColorSchemeMedia && systemColorSchemeMedia.matches) ? 'light' : 'dark';
    }
    return themePref === 'light' ? 'light' : 'dark';
  }

  /**
   * อัปเดตพื้นหลัง 3D, หมอกขอบฟ้า, สีพื้นโต๊ะ, สีเส้นตาราง Grid, แสงสว่าง และเส้นขอบ CAD
   * ให้กลมกลืนกับธีมสีและระดับความสว่างที่ผู้ใช้เลือก
   */
  function syncThreeSceneAppearance(resolvedTheme, brightnessPct) {
    const isLight = resolvedTheme === 'light';
    const bFactor = THREE.MathUtils.clamp(brightnessPct / 100, 0.4, 2.0);

    if (lastAppliedSceneTheme !== resolvedTheme) {
      lastAppliedSceneTheme = resolvedTheme;
      const bgHex = isLight ? 0xe8eef5 : 0x070b14;
      const groundHex = isLight ? 0xdce5f0 : 0x0b1220;
      const gridCenterHex = isLight ? 0x0284c7 : 0x0ea5e9;
      const gridLineHex = isLight ? 0x94a3b8 : 0x1e293b;
      const edgeHex = isLight ? 0x1e293b : 0xbae6fd;
      const edgeOpacity = isLight ? 0.3 : 0.28;

      scene.background.setHex(bgHex);
      if (scene.fog) scene.fog.color.setHex(bgHex);
      groundMesh.material.color.setHex(groundHex);

      // สร้าง GridHelper ใหม่ให้สีเส้นตารางตรงธีมและคืนหน่วยความจำของเดิม
      if (gridHelper) {
        scene.remove(gridHelper);
        if (gridHelper.geometry) gridHelper.geometry.dispose();
        if (Array.isArray(gridHelper.material)) {
          gridHelper.material.forEach((m) => m.dispose());
        } else if (gridHelper.material) {
          gridHelper.material.dispose();
        }
      }
      gridHelper = new THREE.GridHelper(500, 25, gridCenterHex, gridLineHex);
      gridHelper.position.y = -0.2;
      scene.add(gridHelper);

      hemiLight.color.setHex(isLight ? 0xffffff : 0xdbeafe);
      hemiLight.groundColor.setHex(isLight ? 0x94a3b8 : 0x0f172a);
      rimLight.color.setHex(isLight ? 0x0284c7 : 0x38bdf8);
      trailLine.material.color.setHex(isLight ? 0x0284c7 : 0x38bdf8);

      edgeLines.forEach((l) => {
        l.material.color.setHex(edgeHex);
        l.material.opacity = edgeOpacity;
      });
    }

    // ปรับความเข้มแสงส่องโมเดลแขนกลและ Exposure โดยตรง
    const baseHemi = isLight ? 0.98 : 0.85;
    hemiLight.intensity = baseHemi * bFactor;
    dirLight.intensity = 1.15 * bFactor;
    rimLight.intensity = (isLight ? 0.42 : 0.55) * bFactor;
    renderer.toneMappingExposure = 0.85 + 0.3 * bFactor;
  }

  function syncSettingsModalControls() {
    const resolvedTheme = resolveEffectiveTheme(displaySettings.theme);
    if (ui.valSettingTheme) {
      ui.valSettingTheme.textContent = displaySettings.theme === 'auto'
        ? `Auto (${resolvedTheme === 'light' ? 'Light' : 'Dark'})`
        : THEME_LABELS[displaySettings.theme];
    }
    document.querySelectorAll('[data-theme-value]').forEach((btn) => {
      const isSelected = btn.dataset.themeValue === displaySettings.theme;
      btn.classList.toggle('active', isSelected);
      btn.setAttribute('aria-checked', String(isSelected));
    });

    if (ui.sliderSceneBrightness) ui.sliderSceneBrightness.value = String(displaySettings.brightness);
    if (ui.valSceneBrightness) ui.valSceneBrightness.textContent = `${displaySettings.brightness}%`;

    if (ui.sliderPanelOpacity) ui.sliderPanelOpacity.value = String(displaySettings.panelOpacity);
    if (ui.valPanelOpacity) ui.valPanelOpacity.textContent = `${displaySettings.panelOpacity}%`;

    if (ui.toggleGlassBlur) ui.toggleGlassBlur.checked = displaySettings.glassBlur;

    if (ui.valSettingScale) ui.valSettingScale.textContent = SCALE_LABELS[displaySettings.uiScale] || 'ปกติ';
    document.querySelectorAll('[data-scale-value]').forEach((btn) => {
      const isSelected = btn.dataset.scaleValue === displaySettings.uiScale;
      btn.classList.toggle('active', isSelected);
      btn.setAttribute('aria-checked', String(isSelected));
    });
  }

  function applyDisplaySettings(nextSettings, persist = true) {
    displaySettings = { ...displaySettings, ...nextSettings };
    const resolvedTheme = resolveEffectiveTheme(displaySettings.theme);
    const root = document.documentElement;

    root.dataset.theme = resolvedTheme;
    root.dataset.uiScale = displaySettings.uiScale;

    const alpha = +(displaySettings.panelOpacity / 100).toFixed(2);
    const alphaStrong = +Math.min(1, alpha + 0.06).toFixed(2);
    root.style.setProperty('--panel-alpha', String(alpha));
    root.style.setProperty('--panel-alpha-strong', String(alphaStrong));
    root.classList.toggle('no-glass-blur', !displaySettings.glassBlur);

    const metaTheme = document.querySelector('meta[name="theme-color"]');
    if (metaTheme) {
      metaTheme.setAttribute('content', resolvedTheme === 'light' ? '#e8eef5' : '#090d16');
    }

    syncThreeSceneAppearance(resolvedTheme, displaySettings.brightness);
    syncSettingsModalControls();

    if (persist) saveDisplaySettings(displaySettings);

    // คำนวณระยะปลอดภัยของ Top Bar และ Bottom Nav ใหม่หลังเปลี่ยนขนาด UI
    requestAnimationFrame(() => {
      if (typeof syncLayoutOffsets === 'function') syncLayoutOffsets();
    });
  }

  function isSettingsModalOpen() {
    return Boolean(ui.settingsBackdrop && !ui.settingsBackdrop.hidden);
  }

  function openSettingsModal() {
    if (!ui.settingsBackdrop || !ui.settingsModal) return;
    lastFocusedBeforeSettings = document.activeElement;
    syncSettingsModalControls();
    ui.settingsBackdrop.hidden = false;
    if (ui.btnOpenSettings) ui.btnOpenSettings.setAttribute('aria-expanded', 'true');
    requestAnimationFrame(() => {
      if (ui.btnCloseSettings) {
        ui.btnCloseSettings.focus();
      } else {
        ui.settingsModal.focus();
      }
    });
  }

  function closeSettingsModal() {
    if (!ui.settingsBackdrop || ui.settingsBackdrop.hidden) return;
    ui.settingsBackdrop.hidden = true;
    if (ui.btnOpenSettings) ui.btnOpenSettings.setAttribute('aria-expanded', 'false');
    if (lastFocusedBeforeSettings && typeof lastFocusedBeforeSettings.focus === 'function') {
      lastFocusedBeforeSettings.focus();
    } else if (ui.btnOpenSettings) {
      ui.btnOpenSettings.focus();
    }
  }

  if (ui.btnOpenSettings) {
    ui.btnOpenSettings.addEventListener('click', () => {
      if (isSettingsModalOpen()) {
        closeSettingsModal();
      } else {
        openSettingsModal();
      }
    });
  }
  if (ui.btnCloseSettings) ui.btnCloseSettings.addEventListener('click', closeSettingsModal);
  if (ui.btnDoneSettings) ui.btnDoneSettings.addEventListener('click', closeSettingsModal);
  if (ui.settingsBackdrop) {
    ui.settingsBackdrop.addEventListener('click', (e) => {
      if (e.target === ui.settingsBackdrop) closeSettingsModal();
    });
  }

  // Focus Trap ภายใน Modal ตามมาตรฐาน WAI-ARIA Dialog
  if (ui.settingsModal) {
    ui.settingsModal.addEventListener('keydown', (e) => {
      if (e.key !== 'Tab') return;
      const focusables = ui.settingsModal.querySelectorAll(
        'button:not([disabled]), input:not([disabled]), select:not([disabled]), [tabindex]:not([tabindex="-1"])'
      );
      if (focusables.length === 0) return;
      const first = focusables[0];
      const last = focusables[focusables.length - 1];
      if (e.shiftKey && document.activeElement === first) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && document.activeElement === last) {
        e.preventDefault();
        first.focus();
      }
    });
  }

  document.querySelectorAll('[data-theme-value]').forEach((btn) => {
    btn.addEventListener('click', () => {
      applyDisplaySettings({ theme: btn.dataset.themeValue }, true);
    });
  });

  if (ui.sliderSceneBrightness) {
    ui.sliderSceneBrightness.addEventListener('input', () => {
      applyDisplaySettings({ brightness: parseInt(ui.sliderSceneBrightness.value, 10) }, true);
    });
  }

  if (ui.sliderPanelOpacity) {
    ui.sliderPanelOpacity.addEventListener('input', () => {
      applyDisplaySettings({ panelOpacity: parseInt(ui.sliderPanelOpacity.value, 10) }, true);
    });
  }

  if (ui.toggleGlassBlur) {
    ui.toggleGlassBlur.addEventListener('change', () => {
      applyDisplaySettings({ glassBlur: Boolean(ui.toggleGlassBlur.checked) }, true);
    });
  }

  document.querySelectorAll('[data-scale-value]').forEach((btn) => {
    btn.addEventListener('click', () => {
      applyDisplaySettings({ uiScale: btn.dataset.scaleValue }, true);
    });
  });

  if (ui.btnResetDisplaySettings) {
    ui.btnResetDisplaySettings.addEventListener('click', () => {
      applyDisplaySettings({ ...DISPLAY_DEFAULTS }, true);
      showToast('คืนค่าการแสดงผลและธีมสีเป็นค่าเริ่มต้นเรียบร้อยแล้ว', 'success', 2600);
    });
  }

  // ติดตามการเปลี่ยนธีมของระบบปฏิบัติการแบบ Real-Time เมื่อผู้ใช้เลือก Auto
  if (systemColorSchemeMedia) {
    const onSystemThemeChange = () => {
      if (displaySettings.theme === 'auto') {
        applyDisplaySettings({}, false);
      }
    };
    if (typeof systemColorSchemeMedia.addEventListener === 'function') {
      systemColorSchemeMedia.addEventListener('change', onSystemThemeChange);
    } else if (typeof systemColorSchemeMedia.addListener === 'function') {
      systemColorSchemeMedia.addListener(onSystemThemeChange);
    }
  }

  // นำค่าตั้งค่าการแสดงผลที่บันทึกไว้มาใช้งานทันที
  applyDisplaySettings(displaySettings, false);

  if (factoryBus) {
    factoryBus.on('theme', () => {
      applyDisplaySettings(loadDisplaySettings(), false);
    });
  }

  /* =========================================================================
   * ระบบตั้งค่าขอบเขตความปลอดภัยมอเตอร์ Servo (Joint Software Limits: Min / Max)
   * Single Source of Truth สำหรับ Manual Slider, Hand Tracking, IK, Demo, Home & WebSerial
   * ========================================================================= */
  const JOINT_LIMITS_STORAGE_KEY = 'arm4dof_joint_limits_v1';
  const JOINT_AXES = Object.freeze(['j1', 'j2', 'j3', 'j4']);
  const JOINT_HARD_LIMITS = Object.freeze({
    j1: Object.freeze({ min: -90, max: 90, minSpan: 5, unit: '°', label: 'DOF 1 (Base Yaw)' }),
    j2: Object.freeze({ min: -45, max: 65, minSpan: 5, unit: '°', label: 'DOF 2 (Shoulder)' }),
    j3: Object.freeze({ min: -55, max: 55, minSpan: 5, unit: '°', label: 'DOF 3 (Elbow)' }),
    j4: Object.freeze({ min: 0, max: 100, minSpan: 5, unit: '%', label: 'DOF 4 (Gripper)' })
  });

  function createDefaultJointLimits() {
    return {
      j1: { min: JOINT_HARD_LIMITS.j1.min, max: JOINT_HARD_LIMITS.j1.max },
      j2: { min: JOINT_HARD_LIMITS.j2.min, max: JOINT_HARD_LIMITS.j2.max },
      j3: { min: JOINT_HARD_LIMITS.j3.min, max: JOINT_HARD_LIMITS.j3.max },
      j4: { min: JOINT_HARD_LIMITS.j4.min, max: JOINT_HARD_LIMITS.j4.max }
    };
  }

  function sanitizeAxisLimit(axis, rawMin, rawMax, changedBound = null) {
    const hard = JOINT_HARD_LIMITS[axis];
    let minVal = Number.isFinite(rawMin) ? Math.round(rawMin) : hard.min;
    let maxVal = Number.isFinite(rawMax) ? Math.round(rawMax) : hard.max;
    let notice = '';

    const clampedMin = THREE.MathUtils.clamp(minVal, hard.min, hard.max - hard.minSpan);
    const clampedMax = THREE.MathUtils.clamp(maxVal, hard.min + hard.minSpan, hard.max);
    if (clampedMin !== minVal || clampedMax !== maxVal) {
      notice = `${hard.label} ถูกปรับให้อยู่ใน Hard Limit (${hard.min}${hard.unit} ถึง ${hard.max}${hard.unit})`;
    }
    minVal = clampedMin;
    maxVal = clampedMax;

    if (maxVal - minVal < hard.minSpan) {
      if (changedBound === 'min') {
        maxVal = Math.min(hard.max, minVal + hard.minSpan);
        if (maxVal - minVal < hard.minSpan) {
          minVal = maxVal - hard.minSpan;
        }
      } else {
        minVal = Math.max(hard.min, maxVal - hard.minSpan);
        if (maxVal - minVal < hard.minSpan) {
          maxVal = minVal + hard.minSpan;
        }
      }
      notice = `${hard.label} ต้องมีระยะห่าง Min–Max อย่างน้อย ${hard.minSpan}${hard.unit} (ปรับเป็น ${minVal}${hard.unit}..${maxVal}${hard.unit})`;
    }

    return { min: minVal, max: maxVal, notice };
  }

  function loadJointLimits() {
    const defaults = createDefaultJointLimits();
    try {
      const raw = localStorage.getItem(JOINT_LIMITS_STORAGE_KEY);
      if (!raw) return defaults;
      const parsed = JSON.parse(raw);
      JOINT_AXES.forEach((axis) => {
        if (parsed && parsed[axis]) {
          const res = sanitizeAxisLimit(axis, Number(parsed[axis].min), Number(parsed[axis].max));
          defaults[axis] = { min: res.min, max: res.max };
        }
      });
      return defaults;
    } catch {
      return defaults;
    }
  }

  function saveJointLimits(limits) {
    try {
      localStorage.setItem(JOINT_LIMITS_STORAGE_KEY, JSON.stringify(limits));
    } catch {
      // ข้ามหากเบราว์เซอร์ปิดการเข้าถึง localStorage
    }
  }

  let jointLimits = loadJointLimits();
  const limitHitState = { j1: false, j2: false, j3: false, j4: false };

  function showLimitValidationNotice(message) {
    if (!ui.limitValidationMsg) return;
    if (message) {
      ui.limitValidationMsg.textContent = message;
      ui.limitValidationMsg.hidden = false;
    } else {
      ui.limitValidationMsg.textContent = '';
      ui.limitValidationMsg.hidden = true;
    }
  }

  /**
   * ฟังก์ชันกลาง (Single Source of Truth) สำหรับจำกัดองศาเป้าหมายทั้ง 4 แกนให้อยู่ใน Joint Software Limits
   * และบันทึกสถานะการชนขอบเขต (limitHitState) เพื่อแสดงป้ายเตือนสีอำพัน LIMIT บนหน้าจอ
   */
  function clampJointTargets(pose, trackHit = true) {
    const out = { ...pose };
    const eps = 0.05;
    JOINT_AXES.forEach((axis) => {
      if (typeof out[axis] !== 'number' || Number.isNaN(out[axis])) return;
      const lim = jointLimits[axis];
      const hard = JOINT_HARD_LIMITS[axis];
      const raw = out[axis];
      const clamped = THREE.MathUtils.clamp(raw, lim.min, lim.max);
      out[axis] = clamped;
      if (trackHit) {
        const isTighterThanHard = lim.min > hard.min || lim.max < hard.max;
        const exceeded = raw < lim.min - eps || raw > lim.max + eps;
        const atCustomBoundary = isTighterThanHard &&
          (Math.abs(clamped - lim.min) <= 0.25 || Math.abs(clamped - lim.max) <= 0.25);
        limitHitState[axis] = exceeded || atCustomBoundary;
      }
    });
    if (typeof out.wrist === 'number' && !Number.isNaN(out.wrist)) {
      out.wrist = THREE.MathUtils.clamp(out.wrist, -45, 45);
    }
    return out;
  }

  function updateLimitIndicatorsUI() {
    JOINT_AXES.forEach((axis) => {
      const lim = jointLimits[axis];
      const hard = JOINT_HARD_LIMITS[axis];
      const isTighterThanHard = lim.min > hard.min || lim.max < hard.max;
      const nearBoundary = isTighterThanHard &&
        (state.target[axis] <= lim.min + 0.25 || state.target[axis] >= lim.max - 0.25);
      const active = Boolean(limitHitState[axis] || nearBoundary);

      const hudPill = ui.hudLimitPills[axis];
      if (hudPill) hudPill.hidden = !active;
      const modalPill = ui.modalLimitPills[axis];
      if (modalPill) modalPill.hidden = !active;
      const ctrlGroup = ui.ctrlGroups[axis];
      if (ctrlGroup) ctrlGroup.classList.toggle('is-limited', active);
      const limitCard = ui.limitCards[axis];
      if (limitCard) limitCard.classList.toggle('is-limited', active);

      if (active && !factoryMetrics.prevLimitClamped[axis]) {
        const hard = JOINT_HARD_LIMITS[axis];
        logFactoryEvent(
          'warn',
          'SAFETY',
          `ชนขอบเขตความปลอดภัย ${hard.label}`,
          `ช่วงอนุญาต ${lim.min}${hard.unit}..${lim.max}${hard.unit}`
        );
      }
      factoryMetrics.prevLimitClamped[axis] = active;
    });
  }

  function axisValueToPwm(axis, val) {
    if (axis === 'j4') {
      return Math.round(THREE.MathUtils.clamp((val / 100) * 180, 0, 180));
    }
    return Math.round(THREE.MathUtils.clamp(90 + val, 0, 180));
  }

  function buildLimitCommandString() {
    const mn1 = axisValueToPwm('j1', jointLimits.j1.min);
    const mx1 = axisValueToPwm('j1', jointLimits.j1.max);
    const mn2 = axisValueToPwm('j2', jointLimits.j2.min);
    const mx2 = axisValueToPwm('j2', jointLimits.j2.max);
    const mn3 = axisValueToPwm('j3', jointLimits.j3.min);
    const mx3 = axisValueToPwm('j3', jointLimits.j3.max);
    const mn4 = axisValueToPwm('j4', jointLimits.j4.min);
    const mx4 = axisValueToPwm('j4', jointLimits.j4.max);
    return `LIMIT:${mn1},${mx1},${mn2},${mx2},${mn3},${mx3},${mn4},${mx4}\n`;
  }

  function syncJointLimitInputsUI() {
    JOINT_AXES.forEach((axis) => {
      const lim = jointLimits[axis];
      const inputs = ui.limitInputs[axis];
      if (inputs && inputs.min && inputs.max) {
        inputs.min.value = String(lim.min);
        inputs.max.value = String(lim.max);
      }
      const badge = ui.pwmRangeBadges[axis];
      if (badge) {
        const pwmMin = axisValueToPwm(axis, lim.min);
        const pwmMax = axisValueToPwm(axis, lim.max);
        badge.textContent = axis === 'j4'
          ? `PWM ${pwmMin}°–${pwmMax}° (${lim.min}%–${lim.max}%)`
          : `PWM ${pwmMin}°–${pwmMax}°`;
      }
    });
  }

  function syncSliderLimitAttributes() {
    const map = {
      j1: document.getElementById('slider-j1'),
      j2: document.getElementById('slider-j2'),
      j3: document.getElementById('slider-j3'),
      j4: document.getElementById('slider-j4')
    };
    JOINT_AXES.forEach((axis) => {
      const el = map[axis];
      if (!el) return;
      el.min = String(jointLimits[axis].min);
      el.max = String(jointLimits[axis].max);
    });
  }

  function applyJointLimits(nextLimits, persist = true, syncSerial = true) {
    jointLimits = nextLimits;
    syncSliderLimitAttributes();
    syncJointLimitInputsUI();

    // ปรับเฉพาะ state.target ให้อยู่ในช่วงปลอดภัย เพื่อให้ EMA Filter ค่อย ๆ ลู่แขนกลเข้าช่วงอย่างนุ่มนวล (ไม่กระชาก)
    state.target = clampJointTargets(state.target, true);
    if (typeof syncSlidersFromState === 'function') {
      syncSlidersFromState();
    }
    updateLimitIndicatorsUI();

    if (persist) saveJointLimits(jointLimits);
    if (syncSerial && state.serialWriter) {
      state.pendingLimitSync = true;
    }
  }

  // สลับแท็บภายใน Settings Modal (WAI-ARIA Tabs)
  function setActiveSettingsTab(tabKey) {
    const isLimits = tabKey === 'limits';
    if (ui.tabDisplay) {
      ui.tabDisplay.classList.toggle('active', !isLimits);
      ui.tabDisplay.setAttribute('aria-selected', String(!isLimits));
    }
    if (ui.tabLimits) {
      ui.tabLimits.classList.toggle('active', isLimits);
      ui.tabLimits.setAttribute('aria-selected', String(isLimits));
    }
    if (ui.tabpanelDisplay) ui.tabpanelDisplay.hidden = isLimits;
    if (ui.tabpanelLimits) ui.tabpanelLimits.hidden = !isLimits;
    if (ui.btnResetDisplaySettings) ui.btnResetDisplaySettings.hidden = isLimits;
    if (ui.btnResetJointLimits) ui.btnResetJointLimits.hidden = !isLimits;
    if (isLimits) {
      syncJointLimitInputsUI();
      updateLimitIndicatorsUI();
    }
  }

  document.querySelectorAll('[data-settings-tab]').forEach((tabBtn) => {
    tabBtn.addEventListener('click', () => {
      setActiveSettingsTab(tabBtn.dataset.settingsTab);
    });
    tabBtn.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') {
        e.preventDefault();
        const nextTab = tabBtn.dataset.settingsTab === 'display' ? 'limits' : 'display';
        setActiveSettingsTab(nextTab);
        const targetEl = nextTab === 'limits' ? ui.tabLimits : ui.tabDisplay;
        if (targetEl) targetEl.focus();
      }
    });
  });

  // ผูก Event ช่องกรอก Min / Max ของทั้ง 4 แกน
  document.querySelectorAll('.limit-num-input[data-limit-axis]').forEach((inputEl) => {
    const commitLimitChange = () => {
      const axis = inputEl.dataset.limitAxis;
      const bound = inputEl.dataset.limitBound;
      const pair = ui.limitInputs[axis];
      if (!pair || !pair.min || !pair.max) return;
      const rawMin = parseFloat(pair.min.value);
      const rawMax = parseFloat(pair.max.value);
      const res = sanitizeAxisLimit(axis, rawMin, rawMax, bound);
      showLimitValidationNotice(res.notice);
      const updated = {
        ...jointLimits,
        [axis]: { min: res.min, max: res.max }
      };
      applyJointLimits(updated, true, true);
    };

    inputEl.addEventListener('change', commitLimitChange);
    inputEl.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        commitLimitChange();
      }
    });
  });

  if (ui.btnResetJointLimits) {
    ui.btnResetJointLimits.addEventListener('click', () => {
      showLimitValidationNotice('');
      JOINT_AXES.forEach((a) => { limitHitState[a] = false; });
      applyJointLimits(createDefaultJointLimits(), true, true);
      showToast('คืนค่าขอบเขตความปลอดภัย Servo ทั้ง 4 แกนเป็นค่าเริ่มต้นแล้ว', 'success', 2800);
    });
  }

  // บังคับใช้ขอบเขตความปลอดภัยตั้งแต่ตอนเริ่มต้นระบบ (ให้ท่าเริ่มต้นอยู่ในช่วงปลอดภัยเสมอ)
  state.target = clampJointTargets(state.target, false);
  state.current = { ...state.target };
  syncSliderLimitAttributes();
  syncJointLimitInputsUI();

  /* =========================================================================
   * SECTION 05: การคลิกเลือกชิ้นส่วน CAD และลากย้ายกล่องเป้าหมายบนโต๊ะ 3 มิติ (Raycasting)
   * ========================================================================= */

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const cubeDragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -9.0); // ระนาบโต๊ะทำงานที่ความสูงศูนย์กลางกล่อง (world y = 9 / cad z = 9)
  const tmpDragWorld = new THREE.Vector3();
  const tmpDragCad = new THREE.Vector3();
  const cubeDragOffset = new THREE.Vector2(0, 0);

  let downPos = { x: 0, y: 0 };
  let isDraggingCube = false;
  let isHoveringCube = false;
  let activeDragPointerId = null;

  /**
   * จำกัดตำแหน่งกล่องลูกบาศก์ให้อยู่บนพื้นที่โต๊ะทำงาน (รัศมี ≤ 220 mm)
   * และไม่ทับพื้นที่ฐานแขนกล (กันออกนอกรัศมี 72 mm จากจุดศูนย์กลางฐาน [-22, 0])
   */
  function clampCubeTablePos(x, y) {
    let cx = x;
    let cy = y;

    // 1. ป้องกันกล่องซ้อนทับฐานแขนกล
    const baseCx = -22.0;
    const baseCy = 0.0;
    const minBaseDist = 72.0;
    const dxBase = cx - baseCx;
    const dyBase = cy - baseCy;
    const distBase = Math.hypot(dxBase, dyBase);
    if (distBase < minBaseDist) {
      if (distBase < 1e-4) {
        cx = baseCx + minBaseDist;
        cy = baseCy;
      } else {
        const push = minBaseDist / distBase;
        cx = baseCx + dxBase * push;
        cy = baseCy + dyBase * push;
      }
    }

    // 2. จำกัดให้อยู่ในรัศมีโต๊ะทำงานไม่เกิน 220 mm จากจุดศูนย์กลางฉาก
    const maxRadius = 220.0;
    const distOrigin = Math.hypot(cx, cy);
    if (distOrigin > maxRadius) {
      const scale = maxRadius / distOrigin;
      cx *= scale;
      cy *= scale;
    }

    return { x: cx, y: cy };
  }

  /**
   * อัปเดตเคอร์เซอร์เมาส์ (grab / grabbing) และแสงไฮไลต์ emissive ของกล่องเป้าหมาย
   */
  function updateCubeVisualState() {
    if (isDraggingCube) {
      renderer.domElement.classList.add('cube-grabbing');
      renderer.domElement.classList.remove('cube-grab');
      targetCubeMat.emissive.setHex(0xb45309);
      targetCubeMat.emissiveIntensity = 0.65;
    } else if (isHoveringCube) {
      renderer.domElement.classList.add('cube-grab');
      renderer.domElement.classList.remove('cube-grabbing');
      targetCubeMat.emissive.setHex(cubeGrabbed ? 0x065f46 : 0x92400e);
      targetCubeMat.emissiveIntensity = 0.52;
    } else {
      renderer.domElement.classList.remove('cube-grab', 'cube-grabbing');
      targetCubeMat.emissive.setHex(cubeGrabbed ? 0x065f46 : 0x78350f);
      targetCubeMat.emissiveIntensity = 0.35;
    }
  }

  function setPointerFromEvent(e) {
    const rect = renderer.domElement.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return false;
    pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    return true;
  }

  function intersectCubeTablePlane() {
    if (!raycaster.ray.intersectPlane(cubeDragPlane, tmpDragWorld)) return false;
    tmpDragCad.copy(tmpDragWorld);
    cadRoot.worldToLocal(tmpDragCad);
    return true;
  }

  function finishCubeDrag(pointerId) {
    if (!isDraggingCube) return;
    isDraggingCube = false;
    activeDragPointerId = null;
    controls.enabled = true;
    if (typeof pointerId === 'number' && renderer.domElement.hasPointerCapture &&
        renderer.domElement.hasPointerCapture(pointerId)) {
      try {
        renderer.domElement.releasePointerCapture(pointerId);
      } catch (_) { /* ignore */ }
    }
    updateCubeVisualState();
  }

  renderer.domElement.addEventListener('pointerdown', (e) => {
    downPos = { x: e.clientX, y: e.clientY };

    // รองรับเฉพาะปุ่มซ้ายของเมาส์หรือการแตะหน้าจอสัมผัส
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    if (!setPointerFromEvent(e)) return;

    raycaster.setFromCamera(pointer, camera);
    const cubeHits = raycaster.intersectObject(targetCube, false);
    if (cubeHits.length > 0) {
      isDraggingCube = true;
      isHoveringCube = true;
      activeDragPointerId = e.pointerId;
      controls.enabled = false;

      // หากกล่องกำลังถูกแขนกลคีบอยู่ ให้ปล่อยออกและวางตั้งตรงบนโต๊ะอย่างปลอดภัย
      if (cubeGrabbed) {
        cubeGrabbed = false;
        targetCube.rotation.set(0, 0, 0);
        targetCube.position.z = 9.0;
      }

      if (intersectCubeTablePlane()) {
        cubeDragOffset.set(
          targetCube.position.x - tmpDragCad.x,
          targetCube.position.y - tmpDragCad.y
        );
      } else {
        cubeDragOffset.set(0, 0);
      }

      if (renderer.domElement.setPointerCapture) {
        try {
          renderer.domElement.setPointerCapture(e.pointerId);
        } catch (_) { /* ignore */ }
      }
      updateCubeVisualState();
    }
  });

  renderer.domElement.addEventListener('pointermove', (e) => {
    if (!setPointerFromEvent(e)) return;
    raycaster.setFromCamera(pointer, camera);

    if (isDraggingCube) {
      if (e.pointerId !== activeDragPointerId) return;
      if (intersectCubeTablePlane()) {
        const clamped = clampCubeTablePos(
          tmpDragCad.x + cubeDragOffset.x,
          tmpDragCad.y + cubeDragOffset.y
        );
        targetCube.position.set(clamped.x, clamped.y, 9.0);
        targetCube.rotation.set(0, 0, 0);
      }
      return;
    }

    if (e.pointerType === 'mouse') {
      const hovered = raycaster.intersectObject(targetCube, false).length > 0;
      if (hovered !== isHoveringCube) {
        isHoveringCube = hovered;
        updateCubeVisualState();
      }
    }
  });

  renderer.domElement.addEventListener('pointerup', (e) => {
    // หากเป็นการปล่อยจากการลาก targetCube ให้คืนการควบคุมกล้องและไม่รบกวนการเลือกชิ้นส่วน CAD
    if (isDraggingCube && e.pointerId === activeDragPointerId) {
      finishCubeDrag(e.pointerId);
      return;
    }

    // ข้ามหากเป็นการลากเมาส์หรือนิ้วหมุนกล้อง (> 6 px)
    if (Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y) > 6) return;
    if (!setPointerFromEvent(e)) return;
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

  renderer.domElement.addEventListener('pointercancel', (e) => {
    if (isDraggingCube) {
      finishCubeDrag(e.pointerId);
    }
  });

  renderer.domElement.addEventListener('pointerleave', () => {
    if (!isDraggingCube && isHoveringCube) {
      isHoveringCube = false;
      updateCubeVisualState();
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
  const tmpTcpCad = new THREE.Vector3();
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
    const tcpCad = tmpTcpCad.copy(tmpTcpWorld).applyMatrix4(tmpInvCadMat);

    ui.tcpX.textContent = tcpCad.x.toFixed(1);
    ui.tcpY.textContent = tcpCad.y.toFixed(1);
    ui.tcpZ.textContent = tcpCad.z.toFixed(1);
    factoryMetrics.tcp.x = +tcpCad.x.toFixed(1);
    factoryMetrics.tcp.y = +tcpCad.y.toFixed(1);
    factoryMetrics.tcp.z = +tcpCad.z.toFixed(1);

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

    // ฟิสิกส์การหยิบและปล่อยกล่องเป้าหมาย (ไม่ดึงกล่องกลับขณะผู้ใช้กำลังลากด้วยเมาส์/นิ้ว)
    if (!isDraggingCube) {
      const distToCube = tcpCad.distanceTo(targetCube.position);
      if (!cubeGrabbed && distToCube < 26.0 && state.current.j4 < 28) {
        cubeGrabbed = true;
        updateCubeVisualState();
        logFactoryEvent('info', 'PRODUCTION', 'หัวคีบจับชิ้นงาน (Cube Grabbed)', `TCP (${tcpCad.x.toFixed(0)}, ${tcpCad.y.toFixed(0)}, ${tcpCad.z.toFixed(0)}) mm`);
      } else if (cubeGrabbed && state.current.j4 > 34) {
        cubeGrabbed = false;
        factoryMetrics.cycleCount += 1;
        updateCubeVisualState();
        // วางกล่องกลับลงบนพื้นโต๊ะ (z = 9 mm สำหรับกล่องสูง 18 mm)
        targetCube.position.z = 9.0;
        targetCube.rotation.set(0, 0, 0);
        logFactoryEvent(
          'success',
          'PRODUCTION',
          `หยิบ-วางชิ้นงานสำเร็จ (Cycle #${factoryMetrics.cycleCount})`,
          `ตำแหน่งวาง (${targetCube.position.x.toFixed(0)}, ${targetCube.position.y.toFixed(0)}) mm`
        );
      }
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
    const j1Raw = (j1Rad * 180) / Math.PI;
    const j1Deg = THREE.MathUtils.clamp(j1Raw, jointLimits.j1.min, jointLimits.j1.max);

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

    const j2Raw = ((alpha1 - HOME_ALPHA1) * 180) / Math.PI;
    const j3Raw = ((alpha2 - HOME_ALPHA2) * 180) / Math.PI;
    const j2Deg = THREE.MathUtils.clamp(j2Raw, jointLimits.j2.min, jointLimits.j2.max);
    const j3Deg = THREE.MathUtils.clamp(j3Raw, jointLimits.j3.min, jointLimits.j3.max);

    limitHitState.j1 = j1Raw < jointLimits.j1.min - 0.05 || j1Raw > jointLimits.j1.max + 0.05;
    limitHitState.j2 = j2Raw < jointLimits.j2.min - 0.05 || j2Raw > jointLimits.j2.max + 0.05;
    limitHitState.j3 = j3Raw < jointLimits.j3.min - 0.05 || j3Raw > jointLimits.j3.max + 0.05;

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

  // ผูก Event ของตัวเลื่อนข้อต่อทั้ง 5 ช่องเข้ากับ state.target (ผ่าน clampJointTargets เสมอ)
  [
    [sliderJ1, 'j1'],
    [sliderJ2, 'j2'],
    [sliderJ3, 'j3'],
    [sliderJ4, 'j4'],
    [sliderWrist, 'wrist']
  ].forEach(([el, key]) => {
    el.addEventListener('input', () => {
      stopAutomation();
      state.target = clampJointTargets({ ...state.target, [key]: parseFloat(el.value) }, true);
      updateLimitIndicatorsUI();
    });
  });

  document.getElementById('btn-reset-home').addEventListener('click', () => {
    stopAutomation();
    state.target = clampJointTargets(HOME_POSE, true);
    trailCount = 0;
    trailGeo.setDrawRange(0, 0);
    syncSlidersFromState();
    updateLimitIndicatorsUI();
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
    btnEdges.setAttribute('aria-pressed', String(state.showEdges));
    edgeLines.forEach((l) => (l.visible = state.showEdges));
  });

  const btnPivots = document.getElementById('btn-toggle-pivots');
  btnPivots.addEventListener('click', () => {
    state.showPivots = !state.showPivots;
    btnPivots.classList.toggle('active', state.showPivots);
    btnPivots.setAttribute('aria-pressed', String(state.showPivots));
    pivotMarkers.forEach((p) => (p.visible = state.showPivots));
  });

  const btnXray = document.getElementById('btn-toggle-xray');
  btnXray.addEventListener('click', () => {
    state.xrayMode = !state.xrayMode;
    btnXray.classList.toggle('active', state.xrayMode);
    btnXray.setAttribute('aria-pressed', String(state.xrayMode));
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
    btnMirror.setAttribute('aria-pressed', String(state.mirrorCam));
    btnMirror.textContent = state.mirrorCam ? '🪞 Mirror: ON' : '🪞 Mirror: OFF';
  });

  /* =========================================================================
   * SECTION 09: โหมดจำลองอัตโนมัติ (Demo), บันทึกท่าทาง (Recorder) & ส่งออก JSON
   * ========================================================================= */

  // ตำแหน่งเป้าหมายของรอบ Pick & Place ปัจจุบัน (คำนวณอัตโนมัติจากตำแหน่งกล่องที่ผู้ใช้ลากวาง)
  const pickPlan = {
    pickX: CUBE_HOME.x,
    pickY: CUBE_HOME.y,
    placeX: 135.0,
    placeY: -55.0,
    cycleIndex: -1
  };

  function refreshPickAndPlacePlan(cycleIdx) {
    pickPlan.cycleIndex = cycleIdx;
    const cx = targetCube.position.x;
    const cy = targetCube.position.y;
    const dxYaw = cx - PIVOTS.yaw[0];
    const dyYaw = cy - PIVOTS.yaw[1];
    const radialDist = Math.hypot(dxYaw, dyYaw);
    // ตรวจสอบว่ากล่องอยู่ในระยะเอื้อมด้านหน้าของแขนกลหรือไม่ (หากไกลเกินให้ใช้จุดใกล้สุดในระยะเอื้อม)
    if (radialDist > 1e-4) {
      const clampedR = THREE.MathUtils.clamp(radialDist, 102.0, 215.0);
      const yawAngle = THREE.MathUtils.clamp(Math.atan2(dyYaw, dxYaw), -Math.PI / 2, Math.PI / 2);
      pickPlan.pickX = PIVOTS.yaw[0] + clampedR * Math.cos(yawAngle);
      pickPlan.pickY = PIVOTS.yaw[1] + clampedR * Math.sin(yawAngle);
    } else {
      pickPlan.pickX = CUBE_HOME.x;
      pickPlan.pickY = CUBE_HOME.y;
    }
    // วางกล่องไปยังฝั่งตรงข้ามของโต๊ะอย่างสมดุล
    pickPlan.placeX = THREE.MathUtils.clamp(pickPlan.pickX, 115.0, 155.0);
    pickPlan.placeY = pickPlan.pickY >= 0 ? -55.0 : 55.0;
  }

  ui.btnDemoPick.addEventListener('click', () => {
    const nextMode = state.demoMode === 'pick' ? null : 'pick';
    stopAutomation();
    state.demoMode = nextMode;
    state.demoTime = 0;
    pickPlan.cycleIndex = -1;
    const isActive = state.demoMode === 'pick';
    if (isActive) refreshPickAndPlacePlan(0);
    ui.btnDemoPick.classList.toggle('active', isActive);
    ui.btnDemoPick.setAttribute('aria-pressed', String(isActive));
  });

  ui.btnDemoCircle.addEventListener('click', () => {
    const nextMode = state.demoMode === 'circle' ? null : 'circle';
    stopAutomation();
    state.demoMode = nextMode;
    state.demoTime = 0;
    const isActive = state.demoMode === 'circle';
    ui.btnDemoCircle.classList.toggle('active', isActive);
    ui.btnDemoCircle.setAttribute('aria-pressed', String(isActive));
  });

  document.getElementById('btn-reset-cube').addEventListener('click', () => {
    cubeGrabbed = false;
    targetCube.position.copy(CUBE_HOME);
    targetCube.rotation.set(0, 0, 0);
    pickPlan.cycleIndex = -1;
    updateCubeVisualState();
  });

  ui.btnRecord.addEventListener('click', () => {
    state.recording = !state.recording;
    ui.btnRecord.setAttribute('aria-pressed', String(state.recording));
    if (state.recording) {
      if (state.playing) {
        state.playing = false;
        ui.btnPlay.classList.remove('active');
        ui.btnPlay.setAttribute('aria-pressed', 'false');
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
      showToast('ยังไม่มีข้อมูลท่าทางที่บันทึกไว้ กดปุ่ม Record แล้วขยับแขนกลก่อนครับ', 'warn');
      return;
    }
    const nextPlay = !state.playing;
    stopAutomation();
    if (nextPlay && state.recording) {
      state.recording = false;
      ui.btnRecord.classList.remove('danger');
      ui.btnRecord.setAttribute('aria-pressed', 'false');
      ui.btnRecord.textContent = `⏺️ บันทึก (${state.recordedFrames.length}f)`;
    }
    state.playing = nextPlay;
    state.playIndex = 0;
    state.playElapsed = 0;
    ui.btnPlay.classList.toggle('active', state.playing);
    ui.btnPlay.setAttribute('aria-pressed', String(state.playing));
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
    state.serialReading = false;
    try {
      if (state.serialReader) {
        await state.serialReader.cancel();
        state.serialReader.releaseLock();
      }
    } catch (_) { /* ข้ามข้อผิดพลาดหาก Reader ถูกปล่อยไปแล้ว */ }
    try {
      if (state.serialWriter) state.serialWriter.releaseLock();
    } catch (_) { /* ข้ามข้อผิดพลาดหาก Writer ถูกปล่อยไปแล้ว */ }
    try {
      if (state.serialPort) await state.serialPort.close();
    } catch (_) { /* ข้ามข้อผิดพลาดหากพอร์ตถูกถอดสายไปแล้ว */ }
    state.serialReader = null;
    state.serialWriter = null;
    state.serialPort = null;
    state.serialSending = false;
    state.boardReady = false;
    factoryMetrics.serial.txHz = 0;
    factoryMetrics.serial.lastHandshake = 'Disconnected';
    factoryMetrics.serial.limitsAcked = false;
    ui.serialDot.className = 'dot';
    ui.serialStatusText.textContent = 'IoT Serial: Disconnected';
    ui.btnSerial.textContent = '🔌 เชื่อมต่อ Arduino / ESP32 (WebSerial)';
    logFactoryEvent('info', 'SERIAL', 'ตัดการเชื่อมต่อ USB WebSerial แล้ว');
  }

  /**
   * อ่านข้อมูลตอบกลับจากไมโครคอนโทรลเลอร์ (2-Way Handshake) เช่น "READY:4DOF_BRIDGE"
   * @param {SerialPort} port - พอร์ต WebSerial ที่เปิดเชื่อมต่อแล้ว
   */
  async function readSerialLoop(port) {
    if (!port || !port.readable) return;
    const decoder = new TextDecoder();
    let rxLine = '';
    state.serialReading = true;
    try {
      state.serialReader = port.readable.getReader();
      while (state.serialReading) {
        const { value, done } = await state.serialReader.read();
        if (done) break;
        if (!value) continue;
        rxLine += decoder.decode(value, { stream: true });
        const lines = rxLine.split(/\r?\n/);
        rxLine = lines.pop() || '';
        lines.forEach((line) => {
          const trimmed = line.trim();
          if (trimmed.includes('READY:4DOF_BRIDGE')) {
            state.boardReady = true;
            state.pendingLimitSync = true;
            factoryMetrics.serial.lastHandshake = 'READY:4DOF_BRIDGE';
            recordSerialPacket('RX', trimmed);
            ui.serialDot.className = 'dot active';
            ui.serialStatusText.textContent = 'IoT Serial: 4DOF Ready';
            showToast('บอร์ดตอบกลับ READY:4DOF_BRIDGE พร้อมซิงก์ขอบเขต Servo และรับคำสั่งแล้ว', 'success', 3500);
            logFactoryEvent('success', 'SERIAL', 'บอร์ดตอบกลับ Handshake READY:4DOF_BRIDGE');
          } else if (trimmed.includes('OK:LIMITS')) {
            factoryMetrics.serial.limitsAcked = true;
            factoryMetrics.serial.lastHandshake = 'OK:LIMITS';
            recordSerialPacket('RX', trimmed);
            console.info('Board confirmed software joint limits (OK:LIMITS)');
            logFactoryEvent('success', 'SERIAL', 'บอร์ดยืนยันขอบเขต Servo (OK:LIMITS)');
          }
        });
      }
    } catch (err) {
      if (state.serialReading) {
        console.warn('Serial read loop ended:', err);
      }
    } finally {
      try {
        if (state.serialReader) state.serialReader.releaseLock();
      } catch (_) { /* ignore */ }
      state.serialReader = null;
      state.serialReading = false;
    }
  }

  const serialSupported = 'serial' in navigator;

  if (serialSupported) {
    navigator.serial.addEventListener('disconnect', (event) => {
      if (state.serialPort && event.target === state.serialPort) {
        console.warn('USB Serial device physically disconnected');
        disconnectSerial();
        showToast('สาย USB Serial ถูกถอดออก ระบบตัดการเชื่อมต่อแล้ว', 'warn');
      }
    });
  } else {
    ui.serialStatusText.textContent = 'IoT Serial: Desktop Only';
    ui.btnSerial.textContent = '🔌 WebSerial (รองรับเฉพาะ Chrome/Edge บน PC)';
    if (ui.serialHelperText) {
      ui.serialHelperText.textContent =
        'อุปกรณ์หรือเบราว์เซอร์นี้ไม่รองรับ USB WebSerial (สามารถควบคุม 3D และ Hand Tracking ได้ตามปกติ หากต้องการต่อบอร์ดจริงกรุณาเปิดด้วย Chrome/Edge บน PC)';
    }
  }

  ui.btnSerial.addEventListener('click', async () => {
    if (!serialSupported) {
      showToast('เบราว์เซอร์นี้ไม่รองรับ WebSerial API (รองรับเฉพาะ Google Chrome หรือ Edge บนคอมพิวเตอร์)', 'warn', 5000);
      return;
    }
    try {
      if (state.serialPort) {
        await disconnectSerial();
        showToast('ตัดการเชื่อมต่อ USB Serial เรียบร้อยแล้ว', 'info', 2500);
        return;
      }
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      state.serialPort = port;
      state.serialWriter = port.writable.getWriter();
      state.pendingLimitSync = true;
      ui.serialDot.className = 'dot active';
      ui.serialStatusText.textContent = 'IoT Serial: 115200 Connected';
      ui.btnSerial.textContent = '🔌 ตัดการเชื่อมต่อ Serial';
      factoryMetrics.serial.lastHandshake = 'Connected (Waiting READY)';
      factoryMetrics.serial.limitsAcked = false;
      showToast('เชื่อมต่อบอร์ด Arduino / ESP32 (115200 baud) สำเร็จ', 'success', 3000);
      logFactoryEvent('success', 'SERIAL', 'เชื่อมต่อ USB WebSerial (115200 baud) สำเร็จ');
      readSerialLoop(port);
    } catch (err) {
      console.warn('WebSerial connection cancelled or failed:', err);
      if (err && err.name !== 'NotFoundError') {
        showToast('ไม่สามารถเปิดพอร์ต Serial ได้ (ตรวจสอบว่าปิด Serial Monitor ใน Arduino IDE แล้ว)', 'error', 5000);
      }
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
    if (!state.serialWriter || state.serialSending) return;

    // ซิงก์คำสั่งขอบเขตความปลอดภัย LIMIT:... ไปยังบอร์ดแบบไม่ชนกับแพ็กเก็ตองศา (รองรับบอร์ดรุ่นเก่าโดยไม่ Error)
    if (state.pendingLimitSync) {
      state.pendingLimitSync = false;
      state.serialSending = true;
      const limitCmd = buildLimitCommandString();
      try {
        await state.serialWriter.write(serialEncoder.encode(limitCmd));
        factoryMetrics.serial.lastLimitCmd = limitCmd.trim();
        factoryMetrics.serial.txCount += 1;
        recordSerialPacket('LIM', limitCmd.trim());
        logFactoryEvent('info', 'SERIAL', 'ส่งคำสั่งซิงก์ขอบเขต Servo ไปยังบอร์ด', limitCmd.trim());
      } catch (e) {
        console.warn('Serial limit sync error, disconnecting:', e);
        await disconnectSerial();
      } finally {
        state.serialSending = false;
      }
      return;
    }

    if (state.estop) {
      factoryMetrics.serial.txHz = 0;
      factoryMetrics.serial.txWindowCount = 0;
      return;
    }
    if (now - state.lastSerialSend > 1200) {
      factoryMetrics.serial.txHz = 0;
    }
    if (now - state.lastSerialSend < 40) return;
    state.lastSerialSend = now;
    state.serialSending = true;

    // Defense-in-Depth: ด่านสุดท้ายก่อนส่งออก USB WebSerial ต้อง Clamp ซ้ำตาม Joint Software Limits และ Hard Limits เสมอ
    const safePose = clampJointTargets(state.current, false);
    const s1 = axisValueToPwm('j1', safePose.j1);
    const s2 = axisValueToPwm('j2', safePose.j2);
    const s3 = axisValueToPwm('j3', safePose.j3);
    const s4 = axisValueToPwm('j4', safePose.j4);
    const msg = `${s1},${s2},${s3},${s4}\n`;
    try {
      await state.serialWriter.write(serialEncoder.encode(msg));
      factoryMetrics.serial.txCount += 1;
      factoryMetrics.serial.txWindowCount += 1;
      factoryMetrics.serial.lastPacket = msg.trim();
      if (now - factoryMetrics.serial.txWindowStart >= 1000) {
        factoryMetrics.serial.txHz = +(
          (factoryMetrics.serial.txWindowCount * 1000) /
          Math.max(1, now - factoryMetrics.serial.txWindowStart)
        ).toFixed(1);
        factoryMetrics.serial.txWindowCount = 0;
        factoryMetrics.serial.txWindowStart = now;
      }
      if (factoryMetrics.serial.txCount % 3 === 1) {
        recordSerialPacket('TX', msg.trim());
      }
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
      factoryMetrics.handDetected = true;
      factoryMetrics.pinchPct = Math.round(gripperOpenPct);
      ui.pinchBar.style.width = `${gripperOpenPct}%`;
      if (ui.pinchBarTrack) {
        ui.pinchBarTrack.setAttribute('aria-valuenow', String(Math.round(gripperOpenPct)));
      }
      ui.pinchValText.textContent =
        gripperOpenPct < 28 ? `GRAB (${Math.round(gripperOpenPct)}%)` : `${Math.round(gripperOpenPct)}%`;

      // แปลงค่าจากมือเป็นองศาเป้าหมาย 4-DOF (บังคับผ่าน clampJointTargets เสมอ)
      stopAutomation();
      if (state.handMode === 'ik') {
        const targetY = ((0.5 - palmX) / 0.38) * 115.0;
        const targetZ = THREE.MathUtils.clamp(20.0 + (0.85 - palmY) * 230.0, 14.0, 200.0);
        const targetX = THREE.MathUtils.clamp(95.0 + ((handScale - 0.14) / 0.18) * 95.0, 80.0, 195.0);

        const ikSol = solveIK(targetX, targetY, targetZ);
        const clamped = clampJointTargets({ ...state.target, j1: ikSol.j1, j2: ikSol.j2, j3: ikSol.j3, j4: gripperOpenPct }, false);
        limitHitState.j4 = gripperOpenPct < jointLimits.j4.min - 0.05 || gripperOpenPct > jointLimits.j4.max + 0.05;
        state.target = clamped;
      } else {
        // โหมดควบคุมข้อต่อโดยตรง (Direct Joint Mapping)
        const rawJ1 = (0.5 - palmX) * 190;
        const rawJ2 = (palmY - 0.45) * 140;
        // คำนวณมุมเอียงสันมือเทียบแนวราบด้วย |dx| เพื่อไม่ให้มุมกระโดด ±180° ไม่ว่าจะใช้มือซ้ายหรือขวา
        const tiltRad = Math.atan2(lm[17].y - lm[5].y, Math.abs(lm[17].x - lm[5].x) + 1e-5);
        const rawJ3 = ((tiltRad * 180) / Math.PI) * 1.15;
        state.target = clampJointTargets({ ...state.target, j1: rawJ1, j2: rawJ2, j3: rawJ3, j4: gripperOpenPct }, true);
      }
      syncSlidersFromState();
      updateLimitIndicatorsUI();
    } else {
      factoryMetrics.handDetected = false;
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
    const wasRunning = state.camRunning;
    state.camRunning = false;
    animCamActive = false;
    factoryMetrics.handDetected = false;
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
    if (wasRunning) {
      logFactoryEvent(dotClass === 'dot warn' ? 'warn' : 'info', 'CAMERA', 'ปิดกล้อง Hand Tracking', statusText);
    }
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
      showToast('ไม่สามารถโหลด MediaPipe Hands จาก CDN ได้ กรุณาตรวจสอบอินเทอร์เน็ตแล้วลองใหม่', 'error', 5500);
      return;
    }

    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      ui.camDot.className = 'dot warn';
      ui.camStatusText.textContent = 'Camera: Unsupported Context';
      showToast('เบราว์เซอร์ไม่อนุญาตให้เข้าถึงกล้อง กรุณาเปิดผ่าน http://localhost:8080 หรือ HTTPS', 'warn', 5500);
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
        video: { width: { ideal: 640 }, height: { ideal: 480 }, facingMode: 'user' },
        audio: false
      });
      videoEl.srcObject = mediaStream;
      await videoEl.play();

      // ปรับสัดส่วน Canvas ให้ตรงกับความละเอียดจริงของกล้อง (รองรับทั้งแนวนอน PC และแนวตั้งมือถือ)
      if (videoEl.videoWidth > 0 && videoEl.videoHeight > 0) {
        trackCanvas.width = videoEl.videoWidth;
        trackCanvas.height = videoEl.videoHeight;
      }

      state.camRunning = true;
      animCamActive = true;
      camOverlay.style.display = 'none';
      ui.camDot.className = 'dot active';
      ui.camStatusText.textContent = 'Camera: Live Hand Tracking';
      btnToggleCam.textContent = '⏹️ ปิดกล้อง (Stop Camera)';
      btnToggleCam.classList.add('danger');
      logFactoryEvent('success', 'CAMERA', 'เปิดกล้อง MediaPipe Hand Tracking สำเร็จ', `โหมด ${state.handMode.toUpperCase()}`);

      async function processVideoFrame() {
        if (!animCamActive) return;
        try {
          if (videoEl.readyState >= 2) {
            await handsInstance.send({ image: videoEl });
          }
        } catch (err) {
          console.error('Hand tracking frame error:', err);
          stopWebcam('Camera: Tracking Error', 'dot warn');
          showToast('เกิดข้อผิดพลาดขณะประมวลผลภาพกล้อง กรุณาลองเปิดกล้องใหม่อีกครั้ง', 'error');
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
      showToast(
        'ไม่สามารถเปิดกล้องได้: ' + (err.message || err) + ' (ตรวจสอบการอนุญาตสิทธิ์กล้องในเบราว์เซอร์)',
        'error',
        6000
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
      factoryMetrics.fps = fps;
      ui.fpsBadge.textContent = `${fps} FPS`;
      frameCounter = 0;
      fpsTimer = now;
    }

    if (state.estop) {
      factoryMetrics.estopMs += dt * 1000;
    }

    // 2. ประมวลผลโหมดจำลองอัตโนมัติหรือเล่นซ้ำท่าทางที่บันทึกไว้ (หยุดเมื่อเปิด E-Stop)
    if (state.estop) {
      state.target = { ...state.current };
    } else if (state.demoMode === 'pick') {
      state.demoTime += dt;
      const cycleIdx = Math.floor(state.demoTime / 8.0);
      const t = state.demoTime % 8.0;
      if (cycleIdx !== pickPlan.cycleIndex && !cubeGrabbed && !isDraggingCube) {
        refreshPickAndPlacePlan(cycleIdx);
      }
      // หากผู้ใช้ลากกล่องเปลี่ยนจุดระหว่างช่วงกำลังเข้าหา ให้ตามตำแหน่งล่าสุดทันที
      if (t < 2.6 && !cubeGrabbed && !isDraggingCube) {
        refreshPickAndPlacePlan(cycleIdx);
      }

      if (t < 1.5) {
        const sol = solveIK(pickPlan.pickX, pickPlan.pickY, 65);
        state.target = clampJointTargets({ ...sol, j4: 65, wrist: 0 }, false);
        limitHitState.j4 = 65 < jointLimits.j4.min || 65 > jointLimits.j4.max;
      } else if (t < 2.8) {
        const sol = solveIK(pickPlan.pickX, pickPlan.pickY, 16);
        state.target = clampJointTargets({ ...sol, j4: 60, wrist: 0 }, false);
        limitHitState.j4 = 60 < jointLimits.j4.min || 60 > jointLimits.j4.max;
      } else if (t < 3.6) {
        const sol = solveIK(pickPlan.pickX, pickPlan.pickY, 16);
        state.target = clampJointTargets({ ...sol, j4: 12, wrist: 0 }, false);
        limitHitState.j4 = 12 < jointLimits.j4.min || 12 > jointLimits.j4.max;
      } else if (t < 5.2) {
        const frac = (t - 3.6) / 1.6;
        const x = pickPlan.pickX + (pickPlan.placeX - pickPlan.pickX) * frac;
        const y = pickPlan.pickY + (pickPlan.placeY - pickPlan.pickY) * frac;
        const z = 16 + Math.sin(frac * Math.PI) * 75;
        const sol = solveIK(x, y, z);
        state.target = clampJointTargets({ ...sol, j4: 12, wrist: 0 }, false);
        limitHitState.j4 = 12 < jointLimits.j4.min || 12 > jointLimits.j4.max;
      } else if (t < 6.2) {
        const sol = solveIK(pickPlan.placeX, pickPlan.placeY, 18);
        state.target = clampJointTargets({ ...sol, j4: 12, wrist: 0 }, false);
        limitHitState.j4 = 12 < jointLimits.j4.min || 12 > jointLimits.j4.max;
      } else if (t < 7.0) {
        const sol = solveIK(pickPlan.placeX, pickPlan.placeY, 22);
        state.target = clampJointTargets({ ...sol, j4: 65, wrist: 0 }, false);
        limitHitState.j4 = 65 < jointLimits.j4.min || 65 > jointLimits.j4.max;
      } else {
        const sol = solveIK(140, 0, 110);
        state.target = clampJointTargets({ ...sol, j4: 45, wrist: 0 }, false);
        limitHitState.j4 = 45 < jointLimits.j4.min || 45 > jointLimits.j4.max;
      }
      syncSlidersFromState();
    } else if (state.demoMode === 'circle') {
      state.demoTime += dt * 1.8;
      const tx = 135 + 25 * Math.cos(state.demoTime);
      const ty = 55 * Math.sin(state.demoTime);
      const tz = 95 + 35 * Math.sin(state.demoTime * 2);
      const sol = solveIK(tx, ty, tz);
      const rawJ4 = 35 + 25 * Math.sin(state.demoTime * 3);
      state.target = clampJointTargets({ ...state.target, j1: sol.j1, j2: sol.j2, j3: sol.j3, j4: rawJ4 }, false);
      limitHitState.j4 = rawJ4 < jointLimits.j4.min || rawJ4 > jointLimits.j4.max;
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
      state.target = clampJointTargets({ j1: f.j1, j2: f.j2, j3: f.j3, j4: f.j4, wrist: f.wrist }, true);
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

    // 5. ตรวจสอบขนาด Canvas เทียบกับ Viewport จริงทุกเฟรม (มาตรฐาน Three.js resizeRendererToDisplaySize)
    syncRendererToDisplaySize();

    // 6. อัปเดต HUD, ป้าย LIMIT, จลนศาสตร์ 3 มิติ, ส่งคำสั่ง Serial, ส่ง Telemetry ข้ามหน้าต่าง และเรนเดอร์ฉาก
    updateJointLabels();
    updateLimitIndicatorsUI();
    updateArmKinematics();
    sendSerialTelemetry(now);
    publishFactoryTelemetry(now, false);

    controls.update();
    renderer.render(scene, camera);
  }

  function getActiveControlModeLabel() {
    if (state.estop) return 'E-STOP';
    if (state.camRunning) return state.handMode === 'ik' ? 'Hand Tracking (3D IK)' : 'Hand Tracking (Direct)';
    if (state.demoMode === 'pick') return 'Auto Pick & Place';
    if (state.demoMode === 'circle') return 'Auto Circle IK';
    if (state.playing) return 'Trajectory Playback';
    return 'Manual / Standby';
  }

  function buildFactorySnapshot(liveFlag = true) {
    const safePose = clampJointTargets(state.current, false);
    const pwm = {
      s1: axisValueToPwm('j1', safePose.j1),
      s2: axisValueToPwm('j2', safePose.j2),
      s3: axisValueToPwm('j3', safePose.j3),
      s4: axisValueToPwm('j4', safePose.j4)
    };
    const limitClamped = {
      j1: Boolean(factoryMetrics.prevLimitClamped.j1),
      j2: Boolean(factoryMetrics.prevLimitClamped.j2),
      j3: Boolean(factoryMetrics.prevLimitClamped.j3),
      j4: Boolean(factoryMetrics.prevLimitClamped.j4)
    };
    const anyLimitClamped = limitClamped.j1 || limitClamped.j2 || limitClamped.j3 || limitClamped.j4;
    const uptimeMs = Math.max(1000, Date.now() - stationStartMs);
    const availabilityPct = +Math.max(0, Math.min(100, ((uptimeMs - factoryMetrics.estopMs) / uptimeMs) * 100)).toFixed(1);
    const performancePct = +Math.max(0, Math.min(100, (factoryMetrics.fps / 60) * 100)).toFixed(1);
    const qualityPct = anyLimitClamped ? 96.5 : 100.0;
    const oeePct = +((availabilityPct / 100) * (performancePct / 100) * (qualityPct / 100) * 100).toFixed(1);

    let machineState = 'STANDBY';
    if (state.estop) {
      machineState = 'E-STOP';
    } else if (anyLimitClamped) {
      machineState = 'LIMIT WARNING';
    } else if (state.camRunning || state.demoMode || state.playing || state.serialPort) {
      machineState = 'RUNNING';
    }

    return {
      version: 1,
      stationId,
      live: Boolean(liveFlag),
      timestamp: Date.now(),
      machineState,
      joints: {
        j1: +state.current.j1.toFixed(1),
        j2: +state.current.j2.toFixed(1),
        j3: +state.current.j3.toFixed(1),
        j4: +state.current.j4.toFixed(1),
        wrist: +state.current.wrist.toFixed(1)
      },
      pwm,
      tcp: { ...factoryMetrics.tcp },
      camera: {
        running: Boolean(state.camRunning),
        statusText: ui.camStatusText ? ui.camStatusText.textContent : 'Camera: Standby',
        handDetected: Boolean(factoryMetrics.handDetected),
        handMode: state.handMode,
        pinchPct: factoryMetrics.pinchPct
      },
      serial: {
        supported: Boolean(serialSupported),
        connected: Boolean(state.serialPort),
        boardReady: Boolean(state.boardReady),
        statusText: ui.serialStatusText ? ui.serialStatusText.textContent : 'IoT Serial: Disconnected',
        baudRate: 115200,
        txHz: state.serialPort ? factoryMetrics.serial.txHz : 0,
        txCount: factoryMetrics.serial.txCount,
        lastPacket: factoryMetrics.serial.lastPacket || `${pwm.s1},${pwm.s2},${pwm.s3},${pwm.s4}`,
        lastLimitCmd: factoryMetrics.serial.lastLimitCmd || buildLimitCommandString().trim(),
        lastHandshake: factoryMetrics.serial.lastHandshake,
        limitsAcked: Boolean(factoryMetrics.serial.limitsAcked),
        recentPackets: factoryMetrics.serial.recentPackets.slice(0, 24)
      },
      safety: {
        estop: Boolean(state.estop),
        limitClamped,
        anyLimitClamped
      },
      production: {
        cycleCount: factoryMetrics.cycleCount,
        cubeGrabbed: Boolean(cubeGrabbed),
        activeMode: getActiveControlModeLabel(),
        fps: factoryMetrics.fps,
        telemetryHz: factoryMetrics.telemetryHz,
        uptimeSec: Math.floor(uptimeMs / 1000),
        availabilityPct,
        performancePct,
        qualityPct,
        oeePct
      },
      jointLimits: {
        j1: { ...jointLimits.j1 },
        j2: { ...jointLimits.j2 },
        j3: { ...jointLimits.j3 },
        j4: { ...jointLimits.j4 }
      }
    };
  }

  function publishFactoryTelemetry(now, force = false) {
    if (!factoryBus) return;
    if (!force && now - factoryMetrics.lastBusPublish < 100) return;
    factoryMetrics.lastBusPublish = now;
    factoryMetrics.busRateCount += 1;
    if (now - factoryMetrics.busRateWindowStart >= 1000) {
      factoryMetrics.telemetryHz = +(
        (factoryMetrics.busRateCount * 1000) /
        Math.max(1, now - factoryMetrics.busRateWindowStart)
      ).toFixed(1);
      factoryMetrics.busRateCount = 0;
      factoryMetrics.busRateWindowStart = now;
    }
    factoryBus.publishTelemetry(buildFactorySnapshot(true), force);
  }

  if (factoryBus) {
    factoryBus.on('requestSnapshot', () => {
      publishFactoryTelemetry(performance.now(), true);
    });
  }

  const topBarEl = document.getElementById('top-bar');
  const mobileNavEl = document.getElementById('mobile-nav');
  const bottomHudEl = document.getElementById('bottom-hud');

  /**
   * คำนวณระยะปลอดภัยด้านบน (--topbar-offset) และด้านล่าง (--bottom-safe-offset)
   * จากขนาดจริงของ DOM เพื่อให้แผงควบคุม Toast และ Bottom Sheet ไม่ทับซ้อนกันในทุกการซูม
   */
  function syncLayoutOffsets() {
    if (topBarEl) {
      const rect = topBarEl.getBoundingClientRect();
      const topOffset = Math.max(56, Math.ceil(rect.bottom + 10));
      document.documentElement.style.setProperty('--topbar-offset', `${topOffset}px`);
    }

    let bottomOffset = 14;
    if (mobileNavEl && getComputedStyle(mobileNavEl).display !== 'none') {
      const navRect = mobileNavEl.getBoundingClientRect();
      bottomOffset = Math.max(bottomOffset, Math.ceil((window.innerHeight - navRect.top) + 10));
    } else if (bottomHudEl && getComputedStyle(bottomHudEl).display !== 'none') {
      const hudRect = bottomHudEl.getBoundingClientRect();
      bottomOffset = Math.max(bottomOffset, Math.ceil((window.innerHeight - hudRect.top) + 10));
    }
    document.documentElement.style.setProperty('--bottom-safe-offset', `${bottomOffset}px`);
  }

  let lastCanvasW = 0;
  let lastCanvasH = 0;
  let lastDpr = 0;

  /**
   * ซิงก์ขนาด WebGL Drawing Buffer ให้ตรงกับขนาดจริงของ #canvas-container เสมอ
   * รองรับการซูมเบราว์เซอร์ (25%–400%), ย้ายจอต่าง DPR และการหมุนหน้าจอมือถือโดยไม่เกิดแถบสีดำ
   */
  function syncRendererToDisplaySize() {
    const w = Math.floor(ui.canvasContainer.clientWidth || window.innerWidth);
    const h = Math.floor(ui.canvasContainer.clientHeight || window.innerHeight);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (w <= 0 || h <= 0) return;
    if (w !== lastCanvasW || h !== lastCanvasH || dpr !== lastDpr) {
      lastCanvasW = w;
      lastCanvasH = h;
      lastDpr = dpr;
      renderer.setPixelRatio(dpr);
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      syncLayoutOffsets();
      if (w >= 1200) {
        setMobilePanel('none');
      }
    }
  }

  function handleViewportResize() {
    syncRendererToDisplaySize();
    syncLayoutOffsets();
  }

  syncLayoutOffsets();
  syncRendererToDisplaySize();

  if (document.fonts && document.fonts.ready) {
    document.fonts.ready.then(() => syncLayoutOffsets());
  }

  if (typeof ResizeObserver !== 'undefined') {
    const ro = new ResizeObserver(() => handleViewportResize());
    ro.observe(ui.canvasContainer);
    if (topBarEl) ro.observe(topBarEl);
    if (mobileNavEl) ro.observe(mobileNavEl);
    if (bottomHudEl) ro.observe(bottomHudEl);
  }

  if (window.visualViewport) {
    window.visualViewport.addEventListener('resize', handleViewportResize);
  }
  window.addEventListener('resize', handleViewportResize);
  window.addEventListener('orientationchange', () => setTimeout(handleViewportResize, 120));
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) handleViewportResize();
  });

  // ปิดกล้องและปลดพอร์ต Serial อัตโนมัติเมื่อผู้ใช้ปิดแท็บหรือรีเฟรชหน้าเว็บ
  window.addEventListener('pagehide', () => {
    if (state.camRunning) stopWebcam();
    if (state.serialPort) disconnectSerial();
    if (factoryBus) {
      factoryBus.publishTelemetry(buildFactorySnapshot(false), true);
      factoryBus.close();
    }
  });

  // ระบบหยุดฉุกเฉิน (Emergency Stop / E-Stop) และคีย์ลัดมาตรฐาน
  function toggleEstop(forceState) {
    const next = typeof forceState === 'boolean' ? forceState : !state.estop;
    state.estop = next;
    if (ui.btnEstop) {
      ui.btnEstop.setAttribute('aria-pressed', String(state.estop));
      const labelEl = ui.btnEstop.querySelector('.btn-estop-label');
      if (labelEl) labelEl.textContent = state.estop ? 'RELEASE' : 'E-STOP';
    }
    if (state.estop) {
      stopAutomation();
      if (state.recording) {
        state.recording = false;
        ui.btnRecord.classList.remove('danger');
        ui.btnRecord.setAttribute('aria-pressed', 'false');
        ui.btnRecord.textContent = `⏺️ บันทึก (${state.recordedFrames.length}f)`;
      }
      if (state.camRunning) stopWebcam('Camera: Stopped (E-Stop)', 'dot warn');
      factoryMetrics.serial.txHz = 0;
      factoryMetrics.serial.txWindowCount = 0;
      state.target = { ...state.current };
      syncSlidersFromState();
      showToast('🛑 E-STOP ทำงาน: หยุดการเคลื่อนที่และหยุดส่งคำสั่ง Serial ชั่วคราว (กด Space หรือปุ่ม RELEASE เพื่อปลดล็อก)', 'error', 5500);
      logFactoryEvent('error', 'SAFETY', 'เปิดใช้งานหยุดฉุกเฉิน E-STOP', 'หยุดการเคลื่อนที่และระงับคำสั่ง USB Serial ชั่วคราว');
      publishFactoryTelemetry(performance.now(), true);
    } else {
      showToast('ปลดล็อก E-STOP แล้ว สามารถควบคุมแขนกลต่อได้ตามปกติ', 'success', 3000);
      logFactoryEvent('success', 'SAFETY', 'ปลดล็อกหยุดฉุกเฉิน E-STOP แล้ว');
      publishFactoryTelemetry(performance.now(), true);
    }
  }

  if (ui.btnEstop) {
    ui.btnEstop.addEventListener('click', () => toggleEstop());
  }

  window.addEventListener('keydown', (e) => {
    // ปิดหน้าต่างตั้งค่าด้วยปุ่ม Esc เสมอ แม้โฟกัสอยู่ใน Slider หรือปุ่มภายใน Modal
    if (e.code === 'Escape' && isSettingsModalOpen()) {
      e.preventDefault();
      closeSettingsModal();
      return;
    }

    const tag = (e.target && e.target.tagName) ? e.target.tagName.toUpperCase() : '';
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (isSettingsModalOpen()) return;
    const navBackdropEl = document.getElementById('nav-backdrop');
    if (navBackdropEl && !navBackdropEl.hidden) return;

    if (e.code === 'Space') {
      e.preventDefault();
      toggleEstop();
    } else if (e.code === 'KeyR') {
      resetCameraView();
      showToast('รีเซ็ตมุมกล้อง 3D (R)', 'info', 1800);
    } else if (e.code === 'KeyH') {
      if (state.estop) {
        showToast('กรุณาปลดล็อก E-STOP ก่อนกลับท่า Home Pose', 'warn', 2500);
        return;
      }
      stopAutomation();
      state.target = clampJointTargets(HOME_POSE, true);
      trailCount = 0;
      trailGeo.setDrawRange(0, 0);
      syncSlidersFromState();
      updateLimitIndicatorsUI();
      showToast('กลับสู่ท่าเริ่มต้น Home Pose (H)', 'info', 1800);
    } else if (e.code === 'BracketLeft') {
      toggleDesktopPanel('left');
    } else if (e.code === 'BracketRight') {
      toggleDesktopPanel('right');
    } else if (e.code === 'KeyF') {
      if (window.innerWidth >= 1200 && ui.btnToggleHud) {
        ui.btnToggleHud.click();
      } else {
        setMobilePanel('none');
      }
    } else if (e.code === 'Escape') {
      setMobilePanel('none');
      if (state.selectedBodyId !== null) {
        selectPart(state.selectedBodyId);
      }
    }
  });

  // ลงทะเบียน PWA Service Worker (เฉพาะบน HTTP/HTTPS)
  if ('serviceWorker' in navigator && (location.protocol === 'https:' || location.protocol === 'http:')) {
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('./sw.js').catch((err) => {
        console.warn('Service Worker registration skipped:', err);
      });
    });
  }

  // ซิงก์ตำแหน่ง Slider ให้ตรงกับท่าเริ่มต้นที่ผ่านการ Clamp ขอบเขตปลอดภัยแล้ว และเริ่มลูปหลัก
  isHardwareActiveGuard = () => Boolean(state.camRunning || state.serialPort);
  syncSlidersFromState();
  updateLimitIndicatorsUI();
  updateArmKinematics();
  requestAnimationFrame(animate);
})();
