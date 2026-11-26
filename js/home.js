/**
 * =============================================================================
 * @file        js/home.js
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description ลอจิกหน้า Home / Overview (home.html)
 *              ซิงก์ธีมสี Dark/Light/Auto ร่วมกับทุกหน้า, ติดตามสถานะสดของสถานี
 *              3D Control ผ่าน ArmFactoryBus และสลับแผนภาพสถาปัตยกรรมระบบ
 * =============================================================================
 */
(function () {
  'use strict';

  const Bus = window.ArmFactoryBus;
  if (!Bus) return;

  Bus.applySharedDisplaySettings();
  Bus.registerServiceWorker();
  Bus.initNavigationDrawer({ windowName: 'arm4dof_home' });

  const bus = Bus.createBus('home');

  const ui = {
    liveDot: document.getElementById('home-live-dot'),
    liveText: document.getElementById('home-live-text'),
    stationMode: document.getElementById('home-station-mode'),
    serialState: document.getElementById('home-serial-state'),
    cycleCount: document.getElementById('home-cycle-count'),
    btnCycleTheme: document.getElementById('btn-cycle-theme'),
    themeIcon: document.getElementById('theme-icon'),
    themeLabel: document.getElementById('theme-label'),
    btnDiagramArch: document.getElementById('btn-diagram-arch'),
    btnDiagramFlow: document.getElementById('btn-diagram-flow'),
    diagramImg: document.getElementById('hero-diagram-img'),
    diagramOpenLink: document.getElementById('diagram-open-link'),
    diagramCaption: document.getElementById('diagram-caption')
  };

  const DIAGRAMS = {
    arch: {
      src: 'docs/images/06_system_architecture_slide.svg',
      alt: 'แผนภาพสถาปัตยกรรมระบบ 4-DOF Robotic Arm Digital Twin แสดงการทำงานร่วมกันของ CAD 39 ชิ้น, MediaPipe Hands AI, Web Digital Twin และบอร์ด GOOUUU ESP32-S3-CAM'
    },
    flow: {
      src: 'docs/images/01_system_data_flow.svg',
      alt: 'แผนภาพการไหลของข้อมูลระบบ (System Data Flow) จากกล้องและผู้ใช้ผ่านเบราว์เซอร์ไปยัง USB WebSerial และเซอร์โวฮาร์ดแวร์จริง'
    }
  };

  let lastLiveReceivedAt = 0;
  let lastSnapshot = null;

  function syncThemeButton() {
    const cfg = Bus.readDisplaySettings();
    const map = {
      dark: { icon: '🌙', label: 'Dark' },
      light: { icon: '☀️', label: 'Light' },
      auto: { icon: '💻', label: 'Auto' }
    };
    const info = map[cfg.theme] || map.dark;
    if (ui.themeIcon) ui.themeIcon.textContent = info.icon;
    if (ui.themeLabel) ui.themeLabel.textContent = info.label;
  }

  if (ui.btnCycleTheme) {
    ui.btnCycleTheme.addEventListener('click', () => {
      const order = ['dark', 'light', 'auto'];
      const cur = Bus.readDisplaySettings().theme;
      const next = order[(order.indexOf(cur) + 1) % order.length];
      Bus.updateThemePreference(next);
      bus.notifyThemeChanged();
      syncThemeButton();
    });
  }

  bus.on('theme', syncThemeButton);

  function selectDiagram(key) {
    const item = DIAGRAMS[key] || DIAGRAMS.arch;
    if (ui.diagramImg) {
      ui.diagramImg.src = item.src;
      ui.diagramImg.alt = item.alt;
    }
    if (ui.diagramOpenLink) {
      ui.diagramOpenLink.href = item.src;
    }
    if (ui.diagramCaption) {
      ui.diagramCaption.textContent = item.src;
    }
    const isArch = key !== 'flow';
    if (ui.btnDiagramArch) {
      ui.btnDiagramArch.classList.toggle('active', isArch);
      ui.btnDiagramArch.setAttribute('aria-pressed', String(isArch));
    }
    if (ui.btnDiagramFlow) {
      ui.btnDiagramFlow.classList.toggle('active', !isArch);
      ui.btnDiagramFlow.setAttribute('aria-pressed', String(!isArch));
    }
  }

  if (ui.btnDiagramArch) {
    ui.btnDiagramArch.addEventListener('click', () => selectDiagram('arch'));
  }
  if (ui.btnDiagramFlow) {
    ui.btnDiagramFlow.addEventListener('click', () => selectDiagram('flow'));
  }

  function updateLiveStatus() {
    const isLive = Boolean(
      lastSnapshot &&
      lastSnapshot.live &&
      (Date.now() - lastLiveReceivedAt <= Bus.LIVE_STALE_MS)
    );

    if (ui.liveDot) {
      ui.liveDot.className = isLive ? 'dot active' : 'dot warn';
    }
    if (ui.liveText) {
      ui.liveText.textContent = isLive ? '3D Control: Live' : '3D Control: Standby';
    }
    if (ui.stationMode) {
      if (isLive) {
        const modeLabel = (lastSnapshot && lastSnapshot.modeLabel) ? String(lastSnapshot.modeLabel) : 'Live';
        ui.stationMode.textContent = modeLabel;
      } else {
        ui.stationMode.textContent = 'Standby';
      }
    }
    if (ui.serialState) {
      const ser = (lastSnapshot && lastSnapshot.serial) ? lastSnapshot.serial : {};
      ui.serialState.textContent = (isLive && ser.connected)
        ? (ser.boardReady ? 'Connected (S3-CAM)' : 'Connected')
        : 'Disconnected';
    }
    if (ui.cycleCount) {
      const prod = (lastSnapshot && lastSnapshot.production) ? lastSnapshot.production : {};
      const cycles = Number.isFinite(Number(prod.cycleCount)) ? Number(prod.cycleCount) : 0;
      ui.cycleCount.textContent = String(cycles);
    }
  }

  bus.on('telemetry', (snap) => {
    if (!snap) return;
    lastSnapshot = snap;
    if (snap.live) {
      lastLiveReceivedAt = Date.now();
    }
    updateLiveStatus();
  });

  setInterval(updateLiveStatus, 1000);
  window.addEventListener('pagehide', () => bus.close());

  syncThemeButton();
  const savedSnap = Bus.readSnapshot();
  if (savedSnap) {
    lastSnapshot = savedSnap;
  }
  updateLiveStatus();
  bus.requestSnapshot();
})();
