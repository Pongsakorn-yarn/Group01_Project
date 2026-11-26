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
  const CLOUD_CONFIG_KEY = 'arm4dof_cloud_mqtt_cfg_v1';
  const CLOUD_PWD_SESSION_KEY = 'arm4dof_cloud_mqtt_pwd_v1';
  const MAX_EVENTS = 200;
  const LIVE_STALE_MS = 2600;

  const CLOUD_DEFAULTS = Object.freeze({
    host: 'h0ace9f7.ala.asia-southeast1.emqxsl.com',
    port: 8084,
    path: '/mqtt',
    topicPrefix: 'kmutt/isy49303/group01/arm01',
    username: '',
    password: '',
    rememberPassword: false,
    autoConnect: false
  });

  const HARD_LIMITS = Object.freeze({
    j1: Object.freeze({ min: -90, max: 90, pwmMin: 0, pwmMax: 180, unit: '°', label: 'DOF 1: Base Yaw (MG996R)', shortLabel: 'S1 Base (MG996R)', isHardware: true, hwTag: 'HW · GPIO 1' }),
    j2: Object.freeze({ min: -45, max: 65, pwmMin: 45, pwmMax: 155, unit: '°', label: 'DOF 2: Shoulder Pitch (Sim 3D)', shortLabel: 'Shoulder (Sim)', isHardware: false, hwTag: 'SIM ONLY' }),
    j3: Object.freeze({ min: -55, max: 55, pwmMin: 35, pwmMax: 145, unit: '°', label: 'DOF 3: Elbow Pitch (Sim 3D)', shortLabel: 'Elbow (Sim)', isHardware: false, hwTag: 'SIM ONLY' }),
    j4: Object.freeze({ min: 0, max: 100, pwmMin: 0, pwmMax: 180, hwMin: 20, hwMax: 120, unit: '%', label: 'DOF 4: Gripper L/R (2×MG90S)', shortLabel: 'S2/S3 Grip L/R', isHardware: true, hwTag: 'HW · GPIO 2/14' })
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
      esp32Pin: 'GPIO 14',
      homeCmd: '63° (35%)',
      hardPwm: '0°–180° → HW 20°–120°',
      note: 'ควบคุมมือหนีบข้างขวา (Invert สมมาตร • Native LEDC CH2 เลี่ยงขา JTAG 41)'
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

  function readCloudConfig() {
    const out = { ...CLOUD_DEFAULTS };
    try {
      const raw = localStorage.getItem(CLOUD_CONFIG_KEY);
      if (raw) {
        const parsed = JSON.parse(raw);
        if (parsed && typeof parsed === 'object') {
          if (typeof parsed.host === 'string' && parsed.host.trim()) out.host = parsed.host.trim();
          const p = Number(parsed.port);
          if (Number.isFinite(p) && p > 0 && p <= 65535) out.port = Math.round(p);
          if (typeof parsed.path === 'string' && parsed.path.trim()) out.path = parsed.path.trim();
          if (typeof parsed.topicPrefix === 'string' && parsed.topicPrefix.trim()) {
            out.topicPrefix = parsed.topicPrefix.trim().replace(/\/+$/, '');
          }
          if (typeof parsed.username === 'string') out.username = parsed.username.trim();
          out.rememberPassword = Boolean(parsed.rememberPassword);
          out.autoConnect = Boolean(parsed.autoConnect);
          if (out.rememberPassword && typeof parsed.password === 'string') {
            out.password = parsed.password;
          }
        }
      }
      if (!out.password) {
        const sessionPwd = sessionStorage.getItem(CLOUD_PWD_SESSION_KEY);
        if (typeof sessionPwd === 'string' && sessionPwd.length > 0) {
          out.password = sessionPwd;
        }
      }
    } catch (_) { /* ignore */ }
    return out;
  }

  function saveCloudConfig(cfgInput) {
    const cur = readCloudConfig();
    const next = {
      host: String(cfgInput.host || cur.host || CLOUD_DEFAULTS.host).trim(),
      port: Number(cfgInput.port) || cur.port || CLOUD_DEFAULTS.port,
      path: String(cfgInput.path || cur.path || CLOUD_DEFAULTS.path).trim() || '/mqtt',
      topicPrefix: String(cfgInput.topicPrefix || cur.topicPrefix || CLOUD_DEFAULTS.topicPrefix).trim().replace(/\/+$/, ''),
      username: String(cfgInput.username ?? cur.username ?? '').trim(),
      password: String(cfgInput.password ?? cur.password ?? ''),
      rememberPassword: Boolean(cfgInput.rememberPassword),
      autoConnect: Boolean(cfgInput.autoConnect)
    };
    try {
      const toLocal = {
        host: next.host,
        port: next.port,
        path: next.path,
        topicPrefix: next.topicPrefix,
        username: next.username,
        rememberPassword: next.rememberPassword,
        autoConnect: next.autoConnect
      };
      if (next.rememberPassword) {
        toLocal.password = next.password;
      }
      localStorage.setItem(CLOUD_CONFIG_KEY, JSON.stringify(toLocal));
      if (next.password) {
        sessionStorage.setItem(CLOUD_PWD_SESSION_KEY, next.password);
      } else {
        sessionStorage.removeItem(CLOUD_PWD_SESSION_KEY);
      }
    } catch (_) { /* ignore */ }
    return next;
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
      requestSnapshot: new Set(),
      cloudStatus: new Set(),
      boardState: new Set(),
      remoteEstop: new Set()
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

    // -------------------------------------------------------------------------
    // EMQX Cloud Serverless MQTT over WSS (:8084) Transport Layer
    // จำกัดอัตราการส่งให้เหมาะกับ EMQX Serverless (<= 10 msg/s per client)
    // -------------------------------------------------------------------------
    let mqttClient = null;
    let activeCloudCfg = readCloudConfig();
    let lastCloudTelemetryAt = 0;
    let lastCloudCmdAt = 0;
    let lastCloudCmdPayload = '';
    let cloudTxWindowStart = performance.now();
    let cloudTxWindowCount = 0;

    const cloudState = {
      status: 'disconnected', // 'disconnected' | 'connecting' | 'connected' | 'error'
      error: '',
      host: activeCloudCfg.host,
      port: activeCloudCfg.port,
      username: activeCloudCfg.username,
      topicPrefix: activeCloudCfg.topicPrefix,
      boardOnline: false,
      boardStatusText: 'OFFLINE',
      boardState: null,
      txCount: 0,
      txHz: 0,
      lastCmd: '',
      lastLimitCmd: ''
    };

    function getCloudSnapshot() {
      return {
        ...cloudState,
        boardState: cloudState.boardState ? { ...cloudState.boardState } : null
      };
    }

    function notifyCloudStatus() {
      emit('cloudStatus', getCloudSnapshot());
    }

    function disconnectCloud(reason = '') {
      if (mqttClient) {
        try {
          mqttClient.end(true);
        } catch (_) { /* ignore */ }
        mqttClient = null;
      }
      cloudState.status = 'disconnected';
      cloudState.error = reason || '';
      cloudState.txHz = 0;
      cloudTxWindowCount = 0;
      notifyCloudStatus();
    }

    function connectCloud(cfgInput) {
      if (typeof global.mqtt === 'undefined' || typeof global.mqtt.connect !== 'function') {
        cloudState.status = 'error';
        cloudState.error = 'ไม่พบไลบรารี MQTT.js (กรุณาตรวจสอบการเชื่อมต่ออินเทอร์เน็ต)';
        notifyCloudStatus();
        return false;
      }

      const cfg = cfgInput ? saveCloudConfig(cfgInput) : readCloudConfig();
      activeCloudCfg = cfg;

      if (!cfg.host || !cfg.username || !cfg.password) {
        cloudState.status = 'error';
        cloudState.error = 'กรุณากรอก Host, Username และ Password ของ EMQX Cloud ให้ครบ';
        notifyCloudStatus();
        return false;
      }

      if (mqttClient) {
        try {
          mqttClient.end(true);
        } catch (_) { /* ignore */ }
        mqttClient = null;
      }

      const cleanPath = cfg.path.startsWith('/') ? cfg.path : `/${cfg.path}`;
      const brokerUrl = `wss://${cfg.host}:${cfg.port}${cleanPath}`;
      const clientId = `group01-${role}-${Math.random().toString(16).slice(2, 8)}`;
      const prefix = cfg.topicPrefix;

      cloudState.status = 'connecting';
      cloudState.error = '';
      cloudState.host = cfg.host;
      cloudState.port = cfg.port;
      cloudState.username = cfg.username;
      cloudState.topicPrefix = prefix;
      notifyCloudStatus();

      try {
        mqttClient = global.mqtt.connect(brokerUrl, {
          clientId,
          username: cfg.username,
          password: cfg.password,
          clean: true,
          connectTimeout: 10000,
          reconnectPeriod: 4000,
          keepalive: 30
        });
      } catch (err) {
        cloudState.status = 'error';
        cloudState.error = err && err.message ? err.message : 'ไม่สามารถสร้างการเชื่อมต่อ WSS ได้';
        notifyCloudStatus();
        return false;
      }

      mqttClient.on('connect', () => {
        cloudState.status = 'connected';
        cloudState.error = '';
        notifyCloudStatus();

        const subs = [`${prefix}/status`, `${prefix}/state`];
        if (role !== 'station') {
          subs.push(`${prefix}/telemetry`, `${prefix}/events`);
        } else {
          subs.push(`${prefix}/estop`);
        }
        mqttClient.subscribe(subs, { qos: 0 }, (err) => {
          if (err) {
            console.warn('EMQX Cloud subscribe warning:', err);
          }
        });
      });

      mqttClient.on('reconnect', () => {
        cloudState.status = 'connecting';
        notifyCloudStatus();
      });

      mqttClient.on('close', () => {
        if (mqttClient) {
          cloudState.status = 'disconnected';
          cloudState.txHz = 0;
          notifyCloudStatus();
        }
      });

      mqttClient.on('error', (err) => {
        cloudState.status = 'error';
        cloudState.error = err && err.message ? err.message : 'การเชื่อมต่อ EMQX Cloud ล้มเหลว (ตรวจสอบ Username/Password หรือ ACL)';
        notifyCloudStatus();
      });

      mqttClient.on('message', (topic, payloadBuf) => {
        const text = payloadBuf ? payloadBuf.toString() : '';
        if (topic === `${prefix}/status`) {
          const upper = text.trim().toUpperCase();
          cloudState.boardOnline = upper === 'ONLINE';
          cloudState.boardStatusText = upper || 'OFFLINE';
          notifyCloudStatus();
          return;
        }
        if (topic === `${prefix}/state`) {
          try {
            const parsed = JSON.parse(text);
            cloudState.boardState = { ...parsed, receivedAt: Date.now() };
            cloudState.boardOnline = true;
            cloudState.boardStatusText = 'ONLINE';
            emit('boardState', cloudState.boardState);
            notifyCloudStatus();
          } catch (_) { /* ignore malformed */ }
          return;
        }
        if (topic === `${prefix}/estop` && role === 'station') {
          try {
            const parsed = JSON.parse(text);
            if (typeof parsed.estop === 'boolean') {
              emit('remoteEstop', parsed.estop);
            }
          } catch (_) {
            const isStop = text.trim() === '1' || text.trim().toLowerCase() === 'true';
            emit('remoteEstop', isStop);
          }
          return;
        }
        if (topic === `${prefix}/telemetry` && role !== 'station') {
          try {
            const snap = JSON.parse(text);
            if (snap && typeof snap === 'object') {
              snap._transport = 'cloud_mqtt';
              emit('telemetry', snap);
            }
          } catch (_) { /* ignore */ }
          return;
        }
        if (topic === `${prefix}/events` && role !== 'station') {
          try {
            const ev = JSON.parse(text);
            if (ev && typeof ev === 'object') {
              emit('event', ev);
            }
          } catch (_) { /* ignore */ }
        }
      });

      return true;
    }

    function publishCloudArmCmd(base, gripL, gripR, forceNow = false) {
      if (!mqttClient || !mqttClient.connected) return false;
      const now = performance.now();
      const payload = `${base},${gripL},${gripR}`;
      const unchanged = payload === lastCloudCmdPayload;
      // จำกัดสูงสุด ~6.6 Hz (>=150 ms) + Deadband เพื่อไม่ให้ชนลิมิต 10 msg/s ของ EMQX Serverless
      if (!forceNow) {
        if (now - lastCloudCmdAt < 150) return false;
        if (unchanged && now - lastCloudCmdAt < 1800) return false;
      }
      lastCloudCmdAt = now;
      lastCloudCmdPayload = payload;
      try {
        mqttClient.publish(`${activeCloudCfg.topicPrefix}/cmd`, payload, { qos: 0, retain: false });
        cloudState.txCount += 1;
        cloudState.lastCmd = payload;
        cloudTxWindowCount += 1;
        if (now - cloudTxWindowStart >= 1000) {
          cloudState.txHz = +((cloudTxWindowCount * 1000) / Math.max(1, now - cloudTxWindowStart)).toFixed(1);
          cloudTxWindowCount = 0;
          cloudTxWindowStart = now;
        }
        return true;
      } catch (_) {
        return false;
      }
    }

    function publishCloudLimit(limitCmd) {
      if (!mqttClient || !mqttClient.connected) return false;
      const clean = String(limitCmd || '').trim();
      if (!clean) return false;
      try {
        mqttClient.publish(`${activeCloudCfg.topicPrefix}/limit`, clean, { qos: 1, retain: true });
        cloudState.lastLimitCmd = clean;
        cloudState.txCount += 1;
        return true;
      } catch (_) {
        return false;
      }
    }

    function publishCloudEstop(isEstop) {
      if (!mqttClient || !mqttClient.connected) return false;
      try {
        const payload = JSON.stringify({ estop: Boolean(isEstop), ts: Date.now() });
        mqttClient.publish(`${activeCloudCfg.topicPrefix}/estop`, payload, { qos: 1, retain: false });
        cloudState.txCount += 1;
        return true;
      } catch (_) {
        return false;
      }
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
        // ส่ง Telemetry ขึ้น EMQX Cloud (~2.2 Hz) เพื่อประหยัดโควตาและไม่ชนลิมิต 10 msg/s ของ EMQX Serverless
        if (role === 'station' && mqttClient && mqttClient.connected) {
          const minGap = forcePersist ? 180 : 450;
          if (now - lastCloudTelemetryAt >= minGap) {
            lastCloudTelemetryAt = now;
            try {
              const slim = {
                ...snapshot,
                cloud: getCloudSnapshot(),
                serial: snapshot.serial
                  ? {
                      ...snapshot.serial,
                      recentPackets: Array.isArray(snapshot.serial.recentPackets)
                        ? snapshot.serial.recentPackets.slice(0, 10)
                        : []
                    }
                  : undefined
              };
              mqttClient.publish(`${activeCloudCfg.topicPrefix}/telemetry`, JSON.stringify(slim), {
                qos: 0,
                retain: false
              });
            } catch (_) { /* ignore */ }
          }
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
        if (role === 'station' && mqttClient && mqttClient.connected) {
          try {
            mqttClient.publish(`${activeCloudCfg.topicPrefix}/events`, JSON.stringify(ev), {
              qos: 0,
              retain: false
            });
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
      connectCloud,
      disconnectCloud,
      getCloudState: getCloudSnapshot,
      isCloudConnected() {
        return Boolean(mqttClient && mqttClient.connected);
      },
      publishCloudArmCmd,
      publishCloudLimit,
      publishCloudEstop,
      close() {
        disconnectCloud();
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

  /**
   * สร้างและผูกหน้าต่างตั้งค่าการเชื่อมต่อ EMQX Cloud Serverless (MQTT over WSS :8084)
   * ใช้ร่วมกันได้ทุกหน้า (index.html, dashboard.html, diagnostics.html) โดยไม่ฝังรหัสผ่านลงโค้ด
   */
  function initCloudModal(bus, options = {}) {
    if (!bus) return null;
    const roleHint = options.roleHint || 'dash_viewer';
    const triggerIds = Array.isArray(options.triggerIds) ? options.triggerIds : ['btn-cloud-mqtt'];
    const dotEl = document.getElementById(options.dotId || 'cloud-dot');
    const textEl = document.getElementById(options.textId || 'cloud-status-text');
    const onStatusChange = typeof options.onStatusChange === 'function' ? options.onStatusChange : null;

    let backdrop = document.getElementById('cloud-mqtt-backdrop');
    if (!backdrop) {
      backdrop = document.createElement('div');
      backdrop.className = 'settings-backdrop';
      backdrop.id = 'cloud-mqtt-backdrop';
      backdrop.hidden = true;
      backdrop.innerHTML = `
        <div class="settings-dialog" id="cloud-mqtt-modal" role="dialog" aria-modal="true"
          aria-labelledby="cloud-mqtt-title" tabindex="-1" style="max-width:480px;">
          <div class="settings-header">
            <h2 class="settings-title" id="cloud-mqtt-title">☁️ EMQX Cloud Serverless (MQTT WSS)</h2>
            <button type="button" class="btn-modal-close" id="btn-close-cloud-modal" aria-label="ปิดหน้าต่าง" title="ปิด (Esc)">✕</button>
          </div>
          <div class="settings-body">
            <div class="setting-group" style="margin-bottom:10px;">
              <div style="display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap;">
                <span class="setting-label" style="margin:0;">สถานะการเชื่อมต่อ Cloud</span>
                <span class="setting-badge" id="cloud-modal-badge">Disconnected</span>
              </div>
              <p class="helper-text" id="cloud-modal-board-info" style="margin:6px 0 0;">
                บอร์ด GOOUUU ESP32-S3-CAM: <strong>OFFLINE</strong>
              </p>
              <p class="helper-text" id="cloud-modal-error" style="margin:4px 0 0;color:#fb7185;" hidden></p>
            </div>

            <div class="setting-group">
              <label class="setting-label" for="cloud-input-host">EMQX Cloud Broker Host (WSS)</label>
              <input type="text" class="limit-num-input" id="cloud-input-host" style="width:100%;text-align:left;"
                placeholder="h0ace9f7.ala.asia-southeast1.emqxsl.com" autocomplete="off" />
            </div>

            <div class="limit-input-pair" style="margin-bottom:10px;">
              <div class="limit-field">
                <label class="limit-field-label" for="cloud-input-port">WSS Port</label>
                <input type="number" class="limit-num-input" id="cloud-input-port" min="1" max="65535" value="8084" />
              </div>
              <div class="limit-field">
                <label class="limit-field-label" for="cloud-input-path">WS Path</label>
                <input type="text" class="limit-num-input" id="cloud-input-path" style="text-align:left;" value="/mqtt" />
              </div>
            </div>

            <div class="setting-group">
              <label class="setting-label" for="cloud-input-prefix">Topic Prefix (ISA-95 / UNS)</label>
              <input type="text" class="limit-num-input" id="cloud-input-prefix" style="width:100%;text-align:left;"
                placeholder="kmutt/isy49303/group01/arm01" autocomplete="off" />
            </div>

            <div class="limit-input-pair" style="margin-bottom:10px;">
              <div class="limit-field">
                <label class="limit-field-label" for="cloud-input-user">Username (แนะนำ: ${roleHint})</label>
                <input type="text" class="limit-num-input" id="cloud-input-user" style="text-align:left;"
                  placeholder="${roleHint}" autocomplete="username" />
              </div>
              <div class="limit-field">
                <label class="limit-field-label" for="cloud-input-pass">Password</label>
                <input type="password" class="limit-num-input" id="cloud-input-pass" style="text-align:left;"
                  placeholder="••••••••" autocomplete="current-password" />
              </div>
            </div>

            <div class="reset-options-box" style="margin-top:8px;">
              <label class="reset-check-row">
                <input type="checkbox" id="cloud-chk-remember" />
                <span>จำรหัสผ่านบนเบราว์เซอร์เครื่องนี้ (ไม่ส่งขึ้น GitHub)</span>
              </label>
              <label class="reset-check-row">
                <input type="checkbox" id="cloud-chk-autoconnect" />
                <span>เชื่อมต่อ EMQX Cloud อัตโนมัติเมื่อเปิดหน้าเว็บ</span>
              </label>
            </div>
          </div>
          <div class="settings-footer">
            <button type="button" class="btn danger" id="btn-cloud-disconnect" hidden>ตัดการเชื่อมต่อ</button>
            <button type="button" class="btn primary" id="btn-cloud-connect">☁️ เชื่อมต่อ EMQX Cloud</button>
          </div>
        </div>
      `;
      document.body.appendChild(backdrop);
    }

    const modalEl = document.getElementById('cloud-mqtt-modal');
    const btnClose = document.getElementById('btn-close-cloud-modal');
    const btnConnect = document.getElementById('btn-cloud-connect');
    const btnDisconnect = document.getElementById('btn-cloud-disconnect');
    const badgeEl = document.getElementById('cloud-modal-badge');
    const boardInfoEl = document.getElementById('cloud-modal-board-info');
    const errEl = document.getElementById('cloud-modal-error');
    const inpHost = document.getElementById('cloud-input-host');
    const inpPort = document.getElementById('cloud-input-port');
    const inpPath = document.getElementById('cloud-input-path');
    const inpPrefix = document.getElementById('cloud-input-prefix');
    const inpUser = document.getElementById('cloud-input-user');
    const inpPass = document.getElementById('cloud-input-pass');
    const chkRemember = document.getElementById('cloud-chk-remember');
    const chkAuto = document.getElementById('cloud-chk-autoconnect');

    function populateForm() {
      const cfg = readCloudConfig();
      if (inpHost) inpHost.value = cfg.host;
      if (inpPort) inpPort.value = String(cfg.port);
      if (inpPath) inpPath.value = cfg.path;
      if (inpPrefix) inpPrefix.value = cfg.topicPrefix;
      if (inpUser) inpUser.value = cfg.username;
      if (inpPass) inpPass.value = cfg.password;
      if (chkRemember) chkRemember.checked = cfg.rememberPassword;
      if (chkAuto) chkAuto.checked = cfg.autoConnect;
    }

    function openModal() {
      populateForm();
      backdrop.hidden = false;
      requestAnimationFrame(() => {
        if (inpUser && !inpUser.value) inpUser.focus();
        else if (inpPass && !inpPass.value) inpPass.focus();
        else if (modalEl) modalEl.focus();
      });
    }

    function closeModal() {
      backdrop.hidden = true;
    }

    function renderCloudStatus(st) {
      if (!st) return;
      const connected = st.status === 'connected';
      const connecting = st.status === 'connecting';

      if (dotEl) {
        if (connected && st.boardOnline) dotEl.className = 'dot active';
        else if (connected || connecting) dotEl.className = 'dot warn';
        else dotEl.className = 'dot';
      }
      if (textEl) {
        if (connected && st.boardOnline) {
          const rssiTxt = st.boardState && typeof st.boardState.rssi === 'number' ? ` (${st.boardState.rssi} dBm)` : '';
          textEl.textContent = `Cloud: S3 Online${rssiTxt}`;
        } else if (connected) {
          textEl.textContent = 'Cloud: Connected (Board Offline)';
        } else if (connecting) {
          textEl.textContent = 'Cloud: Connecting...';
        } else if (st.status === 'error') {
          textEl.textContent = 'Cloud: Error';
        } else {
          textEl.textContent = 'Cloud: Offline';
        }
      }

      if (badgeEl) {
        badgeEl.textContent = connected ? 'Connected (WSS TLS)' : connecting ? 'Connecting...' : st.status === 'error' ? 'Error' : 'Disconnected';
      }
      if (boardInfoEl) {
        if (st.boardOnline && st.boardState) {
          boardInfoEl.innerHTML =
            `บอร์ด GOOUUU ESP32-S3-CAM: <strong style="color:#10b981;">ONLINE</strong> • ` +
            `HW: Base <code>${st.boardState.b}°</code> / L <code>${st.boardState.gl}°</code> / R <code>${st.boardState.gr}°</code> • ` +
            `Wi-Fi <code>${st.boardState.rssi} dBm</code>`;
        } else if (st.boardOnline) {
          boardInfoEl.innerHTML = 'บอร์ด GOOUUU ESP32-S3-CAM: <strong style="color:#10b981;">ONLINE</strong>';
        } else {
          boardInfoEl.innerHTML = 'บอร์ด GOOUUU ESP32-S3-CAM: <strong>OFFLINE</strong>';
        }
      }
      if (errEl) {
        if (st.error) {
          errEl.textContent = `⚠️ ${st.error}`;
          errEl.hidden = false;
        } else {
          errEl.hidden = true;
        }
      }
      if (btnDisconnect) btnDisconnect.hidden = !(connected || connecting);
      if (btnConnect) btnConnect.textContent = connected ? '🔄 เชื่อมต่อใหม่' : '☁️ เชื่อมต่อ EMQX Cloud';

      if (onStatusChange) onStatusChange(st);
    }

    triggerIds.forEach((id) => {
      const el = document.getElementById(id);
      if (el) el.addEventListener('click', openModal);
    });

    if (btnClose) btnClose.addEventListener('click', closeModal);
    backdrop.addEventListener('click', (e) => {
      if (e.target === backdrop) closeModal();
    });
    window.addEventListener('keydown', (e) => {
      if (!backdrop.hidden && e.key === 'Escape') {
        e.preventDefault();
        closeModal();
      }
    });

    if (btnConnect) {
      btnConnect.addEventListener('click', () => {
        const ok = bus.connectCloud({
          host: inpHost ? inpHost.value : CLOUD_DEFAULTS.host,
          port: inpPort ? Number(inpPort.value) : CLOUD_DEFAULTS.port,
          path: inpPath ? inpPath.value : CLOUD_DEFAULTS.path,
          topicPrefix: inpPrefix ? inpPrefix.value : CLOUD_DEFAULTS.topicPrefix,
          username: inpUser ? inpUser.value : '',
          password: inpPass ? inpPass.value : '',
          rememberPassword: Boolean(chkRemember && chkRemember.checked),
          autoConnect: Boolean(chkAuto && chkAuto.checked)
        });
        if (ok) {
          closeModal();
        }
      });
    }

    if (btnDisconnect) {
      btnDisconnect.addEventListener('click', () => {
        bus.disconnectCloud();
      });
    }

    bus.on('cloudStatus', renderCloudStatus);
    renderCloudStatus(bus.getCloudState());

    // Auto-connect ถ้าผู้ใช้เปิดไว้และมี Username/Password พร้อมแล้ว
    const saved = readCloudConfig();
    if (saved.autoConnect && saved.host && saved.username && saved.password) {
      setTimeout(() => {
        if (!bus.isCloudConnected()) {
          bus.connectCloud(saved);
        }
      }, 300);
    }

    return { openModal, closeModal, renderCloudStatus };
  }

  global.ArmFactoryBus = Object.freeze({
    CHANNEL_NAME,
    SNAPSHOT_KEY,
    EVENTS_KEY,
    DISPLAY_STORAGE_KEY,
    JOINT_LIMITS_STORAGE_KEY,
    CLOUD_CONFIG_KEY,
    CLOUD_DEFAULTS,
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
    readCloudConfig,
    saveCloudConfig,
    readSnapshot,
    writeSnapshot,
    readEvents,
    writeEvents,
    formatTime,
    formatDateTime,
    createBus,
    initCloudModal,
    registerServiceWorker,
    openNamedWindow,
    initNavigationDrawer
  });
})(window);
