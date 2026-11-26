/**
 * =============================================================================
 * @file        js/factory_bus.js
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description ระบบบัสสื่อสารข้อมูลสดระหว่างหน้าต่าง (Multi-Page Real-Time Bus)
 *              ใช้ BroadcastChannel('arm4dof_factory_bus') ร่วมกับ localStorage
 *              Snapshot เพื่อซิงก์ Telemetry, Event/Alarm Log, ธีมสี และ Joint Limits
 *              ระหว่าง index.html, dashboard.html และ diagnostics.html โดยไม่ตัดกล้อง/USB
 * =============================================================================
 */
(function (global) {
  'use strict';

  const CHANNEL_NAME = 'arm4dof_factory_bus';
  const SNAPSHOT_KEY = 'arm4dof_factory_snapshot_v1';
  const EVENTS_KEY = 'arm4dof_factory_events_v1';
  const DISPLAY_STORAGE_KEY = 'arm4dof_display_settings_v1';
  const JOINT_LIMITS_STORAGE_KEY = 'arm4dof_joint_limits_v1';
  const MAX_EVENTS = 200;
  const LIVE_STALE_MS = 2600;

  const HARD_LIMITS = Object.freeze({
    j1: Object.freeze({ min: -90, max: 90, pwmMin: 0, pwmMax: 180, unit: '°', label: 'DOF 1: Base Yaw (MG996R)', shortLabel: 'S1 Base (MG996R)', isHardware: true, hwTag: 'HW · GPIO 1' }),
    j2: Object.freeze({ min: -45, max: 65, pwmMin: 45, pwmMax: 155, unit: '°', label: 'DOF 2: Shoulder Pitch (Sim 3D)', shortLabel: 'Shoulder (Sim)', isHardware: false, hwTag: 'SIM ONLY' }),
    j3: Object.freeze({ min: -55, max: 55, pwmMin: 35, pwmMax: 145, unit: '°', label: 'DOF 3: Elbow Pitch (Sim 3D)', shortLabel: 'Elbow (Sim)', isHardware: false, hwTag: 'SIM ONLY' }),
    j4: Object.freeze({ min: 0, max: 100, pwmMin: 0, pwmMax: 180, hwMin: 20, hwMax: 120, unit: '%', label: 'DOF 4: Gripper L/R (2×MG90S)', shortLabel: 'S2/S3 Grip L/R', isHardware: true, hwTag: 'HW · GPIO 2/41' })
  });

  const PINOUT_SPEC = Object.freeze([
    {
      axis: 'j1',
      servo: 'Servo 1 (MG996R)',
      name: 'Base Yaw — หมุนฐาน',
      arduinoPin: 'D3',
      esp32Pin: 'GPIO 1',
      homeCmd: '90° (0.0°)',
      hardPwm: '0°–180°',
      note: 'GOOUUU ESP32-S3-CAM (ADC1/IO ว่างสะอาด เลี่ยงขากล้องและ PSRAM)'
    },
    {
      axis: 'j4L',
      servo: 'Servo 2 (MG90S #1)',
      name: 'Gripper Left — ก้ามหนีบซ้าย',
      arduinoPin: 'D5',
      esp32Pin: 'GPIO 2',
      homeCmd: '63° (35%)',
      hardPwm: '0°–180° → HW 20°–120°',
      note: 'ควบคุมมือหนีบข้างซ้าย (รองรับทั้งโหมด Sync และแยกนิ้วอิสระ)'
    },
    {
      axis: 'j4R',
      servo: 'Servo 3 (MG90S #2)',
      name: 'Gripper Right — ก้ามหนีบขวา',
      arduinoPin: 'D6',
      esp32Pin: 'GPIO 41',
      homeCmd: '63° (35%)',
      hardPwm: '0°–180° → HW 20°–120°',
      note: 'ควบคุมมือหนีบข้างขวา (Invert สมมาตรในเฟิร์มแวร์ • ขาสำรอง GPIO 14)'
    }
  ]);

  const DISPLAY_DEFAULTS = Object.freeze({
    theme: 'dark',
    brightness: 100,
    panelOpacity: 88,
    glassBlur: true,
    uiScale: 'normal'
  });

  function clampNum(v, lo, hi) {
    return Math.min(hi, Math.max(lo, v));
  }

  function axisValueToPwm(axis, val) {
    const n = Number(val);
    if (!Number.isFinite(n)) return 90;
    if (axis === 'j4') {
      return Math.round(clampNum((n / 100) * 180, 0, 180));
    }
    return Math.round(clampNum(90 + n, 0, 180));
  }

  function createDefaultJointLimits() {
    return {
      j1: { min: HARD_LIMITS.j1.min, max: HARD_LIMITS.j1.max },
      j2: { min: HARD_LIMITS.j2.min, max: HARD_LIMITS.j2.max },
      j3: { min: HARD_LIMITS.j3.min, max: HARD_LIMITS.j3.max },
      j4: { min: HARD_LIMITS.j4.min, max: HARD_LIMITS.j4.max }
    };
  }

  function readJointLimits() {
    const out = createDefaultJointLimits();
    try {
      const raw = localStorage.getItem(JOINT_LIMITS_STORAGE_KEY);
      if (!raw) return out;
      const parsed = JSON.parse(raw);
      ['j1', 'j2', 'j3', 'j4'].forEach((axis) => {
        if (parsed && parsed[axis]) {
          const hard = HARD_LIMITS[axis];
          const mn = Number(parsed[axis].min);
          const mx = Number(parsed[axis].max);
          if (Number.isFinite(mn) && Number.isFinite(mx)) {
            let safeMin = clampNum(Math.round(mn), hard.min, hard.max - 5);
            let safeMax = clampNum(Math.round(mx), hard.min + 5, hard.max);
            if (safeMax - safeMin < 5) {
              safeMax = clampNum(safeMin + 5, hard.min + 5, hard.max);
              if (safeMax - safeMin < 5) {
                safeMin = clampNum(safeMax - 5, hard.min, hard.max - 5);
              }
            }
            out[axis] = { min: safeMin, max: safeMax };
          }
        }
      });
    } catch (_) { /* ignore */ }
    return out;
  }

  function buildLimitCommandFromLimits(limits) {
    const lim = limits || readJointLimits();
    const mnB = axisValueToPwm('j1', lim.j1.min);
    const mxB = axisValueToPwm('j1', lim.j1.max);
    const mnG = axisValueToPwm('j4', lim.j4.min);
    const mxG = axisValueToPwm('j4', lim.j4.max);
    return `LIMIT:${mnB},${mxB},${mnG},${mxG},${mnG},${mxG}`;
  }

  function readDisplaySettings() {
    try {
      const raw = localStorage.getItem(DISPLAY_STORAGE_KEY);
      if (!raw) return { ...DISPLAY_DEFAULTS };
      const cfg = JSON.parse(raw);
      return {
        theme: ['dark', 'light', 'auto'].includes(cfg.theme) ? cfg.theme : DISPLAY_DEFAULTS.theme,
        brightness: typeof cfg.brightness === 'number' ? clampNum(cfg.brightness, 40, 200) : DISPLAY_DEFAULTS.brightness,
        panelOpacity: typeof cfg.panelOpacity === 'number' ? clampNum(cfg.panelOpacity, 45, 100) : DISPLAY_DEFAULTS.panelOpacity,
        glassBlur: typeof cfg.glassBlur === 'boolean' ? cfg.glassBlur : DISPLAY_DEFAULTS.glassBlur,
        uiScale: ['compact', 'normal', 'large'].includes(cfg.uiScale) ? cfg.uiScale : DISPLAY_DEFAULTS.uiScale
      };
    } catch (_) {
      return { ...DISPLAY_DEFAULTS };
    }
  }

  function applySharedDisplaySettings(cfgInput) {
    const cfg = cfgInput || readDisplaySettings();
    const prefersLight = Boolean(window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches);
    const resolvedTheme = cfg.theme === 'auto' ? (prefersLight ? 'light' : 'dark') : cfg.theme;
    const root = document.documentElement;
    root.dataset.theme = resolvedTheme;
    root.dataset.uiScale = cfg.uiScale;
    const alpha = +(cfg.panelOpacity / 100).toFixed(2);
    const alphaStrong = +Math.min(1, alpha + 0.06).toFixed(2);
    root.style.setProperty('--panel-alpha', String(alpha));
    root.style.setProperty('--panel-alpha-strong', String(alphaStrong));
    root.classList.toggle('no-glass-blur', cfg.glassBlur === false);
    const metaTheme = document.querySelector('meta[name="theme-color"]');
    if (metaTheme) {
      metaTheme.setAttribute('content', resolvedTheme === 'light' ? '#e8eef5' : '#090d16');
    }
    return { ...cfg, resolvedTheme };
  }

  function updateThemePreference(nextTheme) {
    const cfg = readDisplaySettings();
    if (!['dark', 'light', 'auto'].includes(nextTheme)) return cfg;
    cfg.theme = nextTheme;
    try {
      localStorage.setItem(DISPLAY_STORAGE_KEY, JSON.stringify(cfg));
    } catch (_) { /* ignore */ }
    return applySharedDisplaySettings(cfg);
  }

  function readSnapshot() {
    try {
      const raw = localStorage.getItem(SNAPSHOT_KEY);
      if (!raw) return null;
      return JSON.parse(raw);
    } catch (_) {
      return null;
    }
  }

  function writeSnapshot(snapshot) {
    try {
      localStorage.setItem(SNAPSHOT_KEY, JSON.stringify(snapshot));
    } catch (_) { /* ignore */ }
  }

  function readEvents() {
    try {
      const raw = localStorage.getItem(EVENTS_KEY);
      if (!raw) return [];
      const arr = JSON.parse(raw);
      return Array.isArray(arr) ? arr : [];
    } catch (_) {
      return [];
    }
  }

  function writeEvents(events) {
    try {
      localStorage.setItem(EVENTS_KEY, JSON.stringify(events.slice(0, MAX_EVENTS)));
    } catch (_) { /* ignore */ }
  }

  function formatTime(ts) {
    try {
      const d = new Date(ts);
      return d.toLocaleTimeString('th-TH', { hour12: false });
    } catch (_) {
      return '--:--:--';
    }
  }

  function formatDateTime(ts) {
    try {
      const d = new Date(ts);
      return d.toLocaleString('th-TH', { hour12: false });
    } catch (_) {
      return '-';
    }
  }

  function createBus(role = 'listener') {
    let channel = null;
    try {
      if (typeof BroadcastChannel !== 'undefined') {
        channel = new BroadcastChannel(CHANNEL_NAME);
      }
    } catch (_) {
      channel = null;
    }

    const listeners = {
      telemetry: new Set(),
      event: new Set(),
      eventsCleared: new Set(),
      theme: new Set(),
      requestSnapshot: new Set()
    };

    function emit(kind, payload) {
      if (!listeners[kind]) return;
      listeners[kind].forEach((fn) => {
        try {
          fn(payload);
        } catch (err) {
          console.warn('FactoryBus listener error:', err);
        }
      });
    }

    if (channel) {
      channel.addEventListener('message', (e) => {
        const msg = e && e.data;
        if (!msg || typeof msg !== 'object') return;
        if (msg.type === 'telemetry' && msg.payload) {
          emit('telemetry', msg.payload);
        } else if (msg.type === 'event' && msg.payload) {
          emit('event', msg.payload);
        } else if (msg.type === 'events_cleared') {
          emit('eventsCleared', null);
        } else if (msg.type === 'theme_changed') {
          const applied = applySharedDisplaySettings();
          emit('theme', applied);
        } else if (msg.type === 'request_snapshot') {
          emit('requestSnapshot', null);
        }
      });
    }

    // Fallback & cross-tab storage synchronization
    window.addEventListener('storage', (e) => {
      if (!e) return;
      if (e.key === DISPLAY_STORAGE_KEY) {
        const applied = applySharedDisplaySettings();
        emit('theme', applied);
      } else if (e.key === SNAPSHOT_KEY && e.newValue && role !== 'station') {
        try {
          emit('telemetry', JSON.parse(e.newValue));
        } catch (_) { /* ignore */ }
      } else if (e.key === EVENTS_KEY && role !== 'station') {
        if (!e.newValue) {
          emit('eventsCleared', null);
        } else {
          try {
            const arr = JSON.parse(e.newValue);
            if (Array.isArray(arr) && arr.length > 0) {
              emit('event', arr[0]);
            }
          } catch (_) { /* ignore */ }
        }
      }
    });

    // Follow OS color scheme when theme === 'auto'
    if (window.matchMedia) {
      const mq = window.matchMedia('(prefers-color-scheme: light)');
      const onMq = () => {
        const cfg = readDisplaySettings();
        if (cfg.theme === 'auto') {
          const applied = applySharedDisplaySettings(cfg);
          emit('theme', applied);
        }
      };
      if (typeof mq.addEventListener === 'function') {
        mq.addEventListener('change', onMq);
      } else if (typeof mq.addListener === 'function') {
        mq.addListener(onMq);
      }
    }

    let lastPersistAt = 0;

    return {
      channelSupported: Boolean(channel),
      on(kind, fn) {
        if (listeners[kind]) listeners[kind].add(fn);
        return () => {
          if (listeners[kind]) listeners[kind].delete(fn);
        };
      },
      publishTelemetry(snapshot, forcePersist = false) {
        if (channel) {
          try {
            channel.postMessage({ type: 'telemetry', payload: snapshot });
          } catch (_) { /* ignore */ }
        }
        const now = Date.now();
        if (forcePersist || now - lastPersistAt >= 450) {
          lastPersistAt = now;
          writeSnapshot(snapshot);
        }
      },
      pushEvent(level, category, message, detail = '') {
        const ev = {
          id: `${Date.now()}-${Math.random().toString(36).slice(2, 7)}`,
          timestamp: Date.now(),
          level: ['info', 'success', 'warn', 'error'].includes(level) ? level : 'info',
          category: String(category || 'SYSTEM'),
          message: String(message || ''),
          detail: String(detail || '')
        };
        const list = readEvents();
        list.unshift(ev);
        if (list.length > MAX_EVENTS) list.length = MAX_EVENTS;
        writeEvents(list);
        if (channel) {
          try {
            channel.postMessage({ type: 'event', payload: ev });
          } catch (_) { /* ignore */ }
        }
        emit('event', ev);
        return ev;
      },
      clearEvents() {
        writeEvents([]);
        if (channel) {
          try {
            channel.postMessage({ type: 'events_cleared' });
          } catch (_) { /* ignore */ }
        }
        emit('eventsCleared', null);
      },
      notifyThemeChanged() {
        if (channel) {
          try {
            channel.postMessage({ type: 'theme_changed' });
          } catch (_) { /* ignore */ }
        }
      },
      requestSnapshot() {
        if (channel) {
          try {
            channel.postMessage({ type: 'request_snapshot' });
          } catch (_) { /* ignore */ }
        }
      },
      close() {
        if (channel) {
          try {
            channel.close();
          } catch (_) { /* ignore */ }
        }
      }
    };
  }

  function registerServiceWorker() {
    if ('serviceWorker' in navigator && (location.protocol === 'https:' || location.protocol === 'http:')) {
      window.addEventListener('load', () => {
        navigator.serviceWorker.register('./sw.js').catch((err) => {
          console.warn('Service Worker registration skipped:', err);
        });
      });
    }
  }

  /**
   * เปิดหรือสลับโฟกัสไปยังแท็บ/หน้าต่างแบบระบุชื่อ (Named Window Target)
   * ไม่ใส่ noopener เพื่อให้เบราว์เซอร์สลับไปแท็บเดิมแทนการสร้างแท็บซ้ำซ้อน
   */
  function openNamedWindow(url, windowName) {
    const win = window.open(url, windowName);
    if (win) {
      try {
        win.focus();
      } catch (_) { /* ignore */ }
    }
    return win;
  }

  /**
   * ผูกการทำงานของปุ่มเมนู 3 ขีด (Hamburger) และ Navigation Drawer มาตรฐาน WAI-ARIA
   * รองรับ Single-Tab Switching + Auto Named Target เมื่อกล้อง/USB กำลังทำงานบนหน้าหลัก
   * และปุ่มไอคอนย่อย ↗ สำหรับเปิดแยกจอ Multi-Monitor โดยไม่สร้างแท็บซ้ำ
   */
  function initNavigationDrawer(options = {}) {
    const pageWindowName = options.windowName || '';
    if (pageWindowName) {
      try {
        window.name = pageWindowName;
      } catch (_) { /* ignore */ }
    }

    const btnOpen = document.getElementById('btn-nav-menu');
    const backdrop = document.getElementById('nav-backdrop');
    const drawer = document.getElementById('nav-drawer');
    const btnClose = document.getElementById('btn-close-nav');
    const isHardwareActive = typeof options.isHardwareActive === 'function' ? options.isHardwareActive : () => false;
    const onAutoPopout = typeof options.onAutoPopout === 'function' ? options.onAutoPopout : null;

    if (!btnOpen || !backdrop || !drawer) return;

    function isOpen() {
      return !backdrop.hidden;
    }

    function openDrawer() {
      backdrop.hidden = false;
      btnOpen.setAttribute('aria-expanded', 'true');
      requestAnimationFrame(() => {
        const firstFocus = drawer.querySelector('.nav-drawer-link[aria-current="page"], .nav-drawer-link, #btn-close-nav');
        if (firstFocus) firstFocus.focus();
      });
    }

    function closeDrawer(restoreFocus = true) {
      if (backdrop.hidden) return;
      backdrop.hidden = true;
      btnOpen.setAttribute('aria-expanded', 'false');
      if (restoreFocus) btnOpen.focus();
    }

    btnOpen.addEventListener('click', () => {
      if (isOpen()) closeDrawer(true);
      else openDrawer();
    });

    if (btnClose) {
      btnClose.addEventListener('click', () => closeDrawer(true));
    }

    backdrop.addEventListener('click', (e) => {
      if (e.target === backdrop) closeDrawer(true);
    });

    window.addEventListener('keydown', (e) => {
      if (!isOpen()) return;
      if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        closeDrawer(true);
        return;
      }
      if (e.key === 'Tab') {
        const focusables = drawer.querySelectorAll(
          'a[href], button:not([disabled]), [tabindex]:not([tabindex="-1"])'
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
      }
    }, true);

    // 1. คลิกชื่อหน้าหลักในเมนู: สลับในแท็บเดิม ยกเว้นกล้อง/USB ทำงานอยู่บนหน้าหลัก -> ใช้ Named Window
    drawer.querySelectorAll('.nav-drawer-link').forEach((link) => {
      link.addEventListener('click', (e) => {
        if (link.getAttribute('aria-current') === 'page') {
          e.preventDefault();
          closeDrawer(true);
          return;
        }
        const targetName = link.dataset.windowName;
        const href = link.getAttribute('href');
        if (isHardwareActive() && targetName && href) {
          e.preventDefault();
          closeDrawer(false);
          openNamedWindow(href, targetName);
          if (onAutoPopout) onAutoPopout(link.dataset.pageLabel || href);
        }
      });
    });

    // 2. คลิกปุ่มไอคอน ↗ (เปิดแยกจอ Multi-Monitor): ใช้ Named Window + .focus() ไม่สร้างแท็บซ้ำ
    document.querySelectorAll('[data-popout-url][data-popout-name]').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.preventDefault();
        const url = btn.dataset.popoutUrl;
        const winName = btn.dataset.popoutName;
        if (!url || !winName) return;
        closeDrawer(false);
        openNamedWindow(url, winName);
      });
    });
  }

  global.ArmFactoryBus = Object.freeze({
    CHANNEL_NAME,
    SNAPSHOT_KEY,
    EVENTS_KEY,
    DISPLAY_STORAGE_KEY,
    JOINT_LIMITS_STORAGE_KEY,
    LIVE_STALE_MS,
    HARD_LIMITS,
    PINOUT_SPEC,
    axisValueToPwm,
    createDefaultJointLimits,
    readJointLimits,
    buildLimitCommandFromLimits,
    readDisplaySettings,
    applySharedDisplaySettings,
    updateThemePreference,
    readSnapshot,
    writeSnapshot,
    readEvents,
    writeEvents,
    formatTime,
    formatDateTime,
    createBus,
    registerServiceWorker,
    openNamedWindow,
    initNavigationDrawer
  });
})(window);
