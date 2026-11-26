/**
 * =============================================================================
 * @file        js/dashboard.js
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description ลอจิกหน้า Factory Digital Twin & Production Dashboard
 *              รับข้อมูลสดจาก index.html ผ่าน ArmFactoryBus (BroadcastChannel +
 *              localStorage Snapshot), วาดกราฟ Real-Time 4-DOF Servo Telemetry
 *              ด้วย HTML5 Canvas ล้วน, อัปเดต KPI/OEE/Gauges และจัดการ Event Log CSV
 * =============================================================================
 */
(function () {
  'use strict';

  const Bus = window.ArmFactoryBus;
  if (!Bus) return;

  Bus.applySharedDisplaySettings();
  Bus.registerServiceWorker();
  Bus.initNavigationDrawer({ windowName: 'arm4dof_dashboard' });

  const bus = Bus.createBus('dashboard');

  const ui = {
    liveDot: document.getElementById('dash-live-dot'),
    liveText: document.getElementById('dash-live-text'),
    standbyBanner: document.getElementById('standby-banner'),
    standbyDetail: document.getElementById('standby-detail'),
    btnCycleTheme: document.getElementById('btn-cycle-theme'),
    themeIcon: document.getElementById('theme-icon'),
    themeLabel: document.getElementById('theme-label'),
    machineBadge: document.getElementById('machine-state-badge'),
    machineDot: document.getElementById('machine-state-dot'),
    machineText: document.getElementById('machine-state-text'),
    metaActiveMode: document.getElementById('meta-active-mode'),
    metaCamState: document.getElementById('meta-cam-state'),
    metaSerialState: document.getElementById('meta-serial-state'),
    metaLastUpdate: document.getElementById('meta-last-update'),
    kpiCycleCount: document.getElementById('kpi-cycle-count'),
    kpiCubeStatus: document.getElementById('kpi-cube-status'),
    kpiUptime: document.getElementById('kpi-uptime'),
    kpiFpsRate: document.getElementById('kpi-fps-rate'),
    kpiTelemetryHz: document.getElementById('kpi-telemetry-hz'),
    kpiSerialHz: document.getElementById('kpi-serial-hz'),
    kpiOee: document.getElementById('kpi-oee'),
    kpiAvailability: document.getElementById('kpi-availability'),
    kpiQuality: document.getElementById('kpi-quality'),
    kpiSafetyStatus: document.getElementById('kpi-safety-status'),
    kpiEstopState: document.getElementById('kpi-estop-state'),
    kpiLimitState: document.getElementById('kpi-limit-state'),
    badgeHandState: document.getElementById('badge-hand-state'),
    tcpX: document.getElementById('dash-tcp-x'),
    tcpY: document.getElementById('dash-tcp-y'),
    tcpZ: document.getElementById('dash-tcp-z'),
    chartCanvas: document.getElementById('telemetry-chart'),
    btnChartMode: document.getElementById('btn-chart-mode'),
    btnChartPause: document.getElementById('btn-chart-pause'),
    btnChartClear: document.getElementById('btn-chart-clear'),
    legS1: document.getElementById('leg-s1'),
    legS2: document.getElementById('leg-s2'),
    legS3: document.getElementById('leg-s3'),
    legS4: document.getElementById('leg-s4'),
    logCount: document.getElementById('log-count'),
    logTbody: document.getElementById('event-log-tbody'),
    btnExportCsv: document.getElementById('btn-export-csv'),
    btnClearLog: document.getElementById('btn-clear-log')
  };

  const SERIES_COLORS = Object.freeze({
    s1: '#38bdf8',
    s2: '#10b981',
    s3: '#f59e0b',
    s4: '#f43f5e'
  });

  const MAX_POINTS = 140;
  let chartMode = 'pwm'; // 'pwm' | 'joint'
  let chartPaused = false;
  let activeLogFilter = 'all';
  let lastLiveReceivedAt = 0;
  let lastSnapshot = null;
  let eventsCache = Bus.readEvents();
  const seenStations = new Map();

  const history = {
    s1: [],
    s2: [],
    s3: [],
    s4: [],
    j1: [],
    j2: [],
    j3: [],
    j4: []
  };

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
      drawTelemetryChart();
    });
  }

  bus.on('theme', () => {
    syncThemeButton();
    drawTelemetryChart();
  });

  function formatUptime(sec) {
    const s = Math.max(0, Math.floor(Number(sec) || 0));
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    const rem = s % 60;
    if (h > 0) return `${h}h ${m}m ${rem}s`;
    if (m > 0) return `${m}m ${rem}s`;
    return `${rem}s`;
  }

  function updateConnectionBanner() {
    const now = Date.now();
    seenStations.forEach((ts, id) => {
      if (now - ts > Bus.LIVE_STALE_MS) seenStations.delete(id);
    });
    const isLive = lastSnapshot && lastSnapshot.live && (now - lastLiveReceivedAt <= Bus.LIVE_STALE_MS);
    if (isLive) {
      ui.liveDot.className = seenStations.size > 1 ? 'dot warn' : 'dot active';
      ui.liveText.textContent = seenStations.size > 1
        ? `Live (${seenStations.size} Stations)`
        : 'Live Station Connected';
      ui.standbyBanner.hidden = true;
    } else {
      ui.liveDot.className = 'dot warn';
      ui.liveText.textContent = 'Standby / Waiting';
      ui.standbyBanner.hidden = false;
      ui.machineText.textContent = 'STANDBY';
      ui.machineBadge.className = 'machine-state-badge state-standby';
      ui.machineDot.className = 'dot';
      ui.kpiFpsRate.textContent = '0 FPS';
      ui.kpiTelemetryHz.textContent = '0.0 Hz';
      ui.kpiSerialHz.textContent = '0.0 Hz';
      if (lastSnapshot && lastSnapshot.timestamp) {
        ui.standbyDetail.textContent = `ยังไม่ได้เชื่อมต่อสถานีสด — แสดงข้อมูล Snapshot ล่าสุดเมื่อ ${Bus.formatDateTime(lastSnapshot.timestamp)}`;
      } else {
        ui.standbyDetail.textContent = 'กำลังรอข้อมูลสดจากหน้าควบคุมหลัก (index.html) — ยังไม่มีข้อมูล Snapshot ที่บันทึกไว้';
      }
    }
  }

  function pushChartSample(snap) {
    if (chartPaused || !snap || !snap.pwm || !snap.joints) return;
    history.s1.push(snap.pwm.s1);
    history.s2.push(snap.pwm.s2);
    history.s3.push(snap.pwm.s3);
    history.s4.push(snap.pwm.s4);
    history.j1.push(snap.joints.j1);
    history.j2.push(snap.joints.j2);
    history.j3.push(snap.joints.j3);
    history.j4.push(snap.joints.j4);
    Object.keys(history).forEach((k) => {
      if (history[k].length > MAX_POINTS) history[k].shift();
    });
  }

  function renderGauge(axis, val, pwmVal, lim, clamped, hwPwm) {
    const hard = Bus.HARD_LIMITS[axis];
    const min = lim && typeof lim.min === 'number' ? lim.min : hard.min;
    const max = lim && typeof lim.max === 'number' ? lim.max : hard.max;
    const span = Math.max(1, max - min);
    const pct = Math.max(0, Math.min(100, Math.round(((val - min) / span) * 100)));

    const card = document.getElementById(`gauge-card-${axis}`);
    const pill = document.getElementById(`dash-limit-${axis}`);
    const valEl = document.getElementById(`dash-val-${axis}`);
    const track = document.getElementById(`gauge-track-${axis}`);
    const fill = document.getElementById(`gauge-fill-${axis}`);
    const minEl = document.getElementById(`dash-min-${axis}`);
    const maxEl = document.getElementById(`dash-max-${axis}`);
    const pctEl = document.getElementById(`dash-pct-${axis}`);

    if (card) card.classList.toggle('is-limited', Boolean(clamped));
    if (pill) pill.hidden = !clamped;
    if (valEl) {
      if (axis === 'j4') {
        const gl = hwPwm && typeof hwPwm.gripL === 'number' ? hwPwm.gripL : pwmVal;
        const gr = hwPwm && typeof hwPwm.gripR === 'number' ? hwPwm.gripR : pwmVal;
        valEl.textContent = `${Math.round(val)}% (L:${gl}° R:${gr}°)`;
      } else if (axis === 'j1') {
        valEl.textContent = `${Number(val).toFixed(1)}° (PWM ${pwmVal}°)`;
      } else {
        valEl.textContent = `${Number(val).toFixed(1)}° (Sim 3D)`;
      }
    }
    if (track) track.setAttribute('aria-valuenow', String(pct));
    if (fill) fill.style.width = `${pct}%`;
    if (minEl) minEl.textContent = `Min: ${min}${hard.unit}`;
    if (maxEl) maxEl.textContent = `Max: ${max > 0 && axis !== 'j4' ? '+' : ''}${max}${hard.unit}`;
    if (pctEl) pctEl.textContent = `${pct}%`;
  }

  function updateChartLegend(snap) {
    if (!snap || !snap.joints || !snap.pwm) return;
    const hw = snap.hwPwm || { base: snap.pwm.s1, gripL: snap.pwm.s4, gripR: snap.pwm.s4 };
    if (chartMode === 'pwm') {
      ui.legS1.textContent = `${hw.base}°`;
      ui.legS2.textContent = `${snap.pwm.s2}° (Sim ${Number(snap.joints.j2).toFixed(1)}°)`;
      ui.legS3.textContent = `${snap.pwm.s3}° (Sim ${Number(snap.joints.j3).toFixed(1)}°)`;
      ui.legS4.textContent = `L:${hw.gripL}° R:${hw.gripR}°`;
    } else {
      ui.legS1.textContent = `${Number(snap.joints.j1).toFixed(1)}°`;
      ui.legS2.textContent = `${Number(snap.joints.j2).toFixed(1)}°`;
      ui.legS3.textContent = `${Number(snap.joints.j3).toFixed(1)}°`;
      const glPct = typeof snap.joints.j4L === 'number' ? Math.round(snap.joints.j4L) : Math.round(snap.joints.j4);
      const grPct = typeof snap.joints.j4R === 'number' ? Math.round(snap.joints.j4R) : Math.round(snap.joints.j4);
      ui.legS4.textContent = `L:${glPct}% R:${grPct}%`;
    }
  }

  function renderSnapshot(snap, fromLiveBus = false) {
    if (!snap) return;
    lastSnapshot = snap;
    if (fromLiveBus && snap.live) {
      lastLiveReceivedAt = Date.now();
      if (snap.stationId) seenStations.set(snap.stationId, lastLiveReceivedAt);
    }

    const isLive = Boolean(snap.live) && (Date.now() - lastLiveReceivedAt <= Bus.LIVE_STALE_MS);
    const machineState = isLive ? (snap.machineState || 'STANDBY') : 'STANDBY';

    ui.machineText.textContent = machineState;
    ui.machineBadge.className = 'machine-state-badge';
    if (machineState === 'E-STOP') {
      ui.machineBadge.classList.add('state-estop');
      ui.machineDot.className = 'dot warn';
    } else if (machineState === 'LIMIT WARNING') {
      ui.machineBadge.classList.add('state-limit');
      ui.machineDot.className = 'dot warn';
    } else if (machineState === 'RUNNING') {
      ui.machineBadge.classList.add('state-running');
      ui.machineDot.className = 'dot active';
    } else {
      ui.machineBadge.classList.add('state-standby');
      ui.machineDot.className = 'dot';
    }

    const prod = snap.production || {};
    const cam = snap.camera || {};
    const ser = snap.serial || {};
    const safety = snap.safety || {};
    const lims = snap.jointLimits || Bus.readJointLimits();

    ui.metaActiveMode.textContent = prod.activeMode || 'Manual / Standby';
    ui.metaCamState.textContent = cam.running ? (cam.handDetected ? 'Live (Hand Locked)' : 'Live (No Hand)') : 'Standby';
    ui.metaSerialState.textContent = ser.connected ? (ser.boardReady ? 'S3-CAM Ready (3 Servo)' : 'Connected (115200)') : 'Disconnected';
    ui.metaLastUpdate.textContent = snap.timestamp ? Bus.formatTime(snap.timestamp) : '-';

    ui.kpiCycleCount.textContent = String(prod.cycleCount || 0);
    ui.kpiCubeStatus.textContent = prod.cubeGrabbed ? 'หัวคีบ: กำลังจับชิ้นงาน (Grabbed)' : 'หัวคีบ: ว่าง (Idle)';
    ui.kpiUptime.textContent = `Uptime: ${formatUptime(prod.uptimeSec || 0)}`;

    ui.kpiFpsRate.textContent = `${prod.fps || 0} FPS`;
    ui.kpiTelemetryHz.textContent = `${Number(prod.telemetryHz || 0).toFixed(1)} Hz`;
    ui.kpiSerialHz.textContent = `${Number(ser.txHz || 0).toFixed(1)} Hz`;

    ui.kpiOee.textContent = `${Number(prod.oeePct || 0).toFixed(1)}%`;
    ui.kpiAvailability.textContent = `${Number(prod.availabilityPct || 0).toFixed(1)}%`;
    ui.kpiQuality.textContent = `${Number(prod.qualityPct || 100).toFixed(1)}%`;

    if (safety.estop) {
      ui.kpiSafetyStatus.textContent = 'E-STOP';
      ui.kpiSafetyStatus.className = 'kpi-value accent-rose';
    } else if (safety.anyLimitClamped) {
      ui.kpiSafetyStatus.textContent = 'LIMIT WARN';
      ui.kpiSafetyStatus.className = 'kpi-value accent-amber';
    } else {
      ui.kpiSafetyStatus.textContent = 'NORMAL';
      ui.kpiSafetyStatus.className = 'kpi-value accent-emerald';
    }
    ui.kpiEstopState.textContent = safety.estop ? 'ENGAGED' : 'OFF';
    const clampedAxes = ['j1', 'j2', 'j3', 'j4'].filter((a) => safety.limitClamped && safety.limitClamped[a]);
    ui.kpiLimitState.textContent = clampedAxes.length ? clampedAxes.map((a) => a.toUpperCase()).join(', ') : 'NONE';

    ui.badgeHandState.textContent = cam.running
      ? `Hand: ${cam.handDetected ? 'Locked' : 'Scanning'} (${String(cam.handMode || 'ik').toUpperCase()})`
      : 'Hand: Standby';

    if (snap.tcp) {
      ui.tcpX.textContent = Number(snap.tcp.x || 0).toFixed(1);
      ui.tcpY.textContent = Number(snap.tcp.y || 0).toFixed(1);
      ui.tcpZ.textContent = Number(snap.tcp.z || 0).toFixed(1);
    }

    if (snap.joints && snap.pwm) {
      const hw = snap.hwPwm || { base: snap.pwm.s1, gripL: snap.pwm.s4, gripR: snap.pwm.s4 };
      renderGauge('j1', snap.joints.j1, hw.base, lims.j1, safety.limitClamped && safety.limitClamped.j1, hw);
      renderGauge('j2', snap.joints.j2, snap.pwm.s2, lims.j2, safety.limitClamped && safety.limitClamped.j2, hw);
      renderGauge('j3', snap.joints.j3, snap.pwm.s3, lims.j3, safety.limitClamped && safety.limitClamped.j3, hw);
      renderGauge('j4', snap.joints.j4, snap.pwm.s4, lims.j4, safety.limitClamped && safety.limitClamped.j4, hw);
      updateChartLegend(snap);
    }

    pushChartSample(snap);
    drawTelemetryChart();
    updateConnectionBanner();
  }

  function drawTelemetryChart() {
    const canvas = ui.chartCanvas;
    if (!canvas) return;
    const wrap = canvas.parentElement;
    if (!wrap) return;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(240, Math.floor(wrap.clientWidth));
    const height = Math.max(180, Math.floor(wrap.clientHeight));

    if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
      canvas.width = Math.floor(width * dpr);
      canvas.height = Math.floor(height * dpr);
    }

    const ctx = canvas.getContext('2d');
    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, width, height);

    const styles = getComputedStyle(document.documentElement);
    const isLight = document.documentElement.dataset.theme === 'light';
    const gridColor = isLight ? 'rgba(100, 116, 139, 0.22)' : 'rgba(148, 163, 184, 0.16)';
    const textColor = styles.getPropertyValue('--text-muted').trim() || '#94a3b8';

    const padL = 44;
    const padR = 14;
    const padT = 16;
    const padB = 24;
    const plotW = Math.max(40, width - padL - padR);
    const plotH = Math.max(40, height - padT - padB);

    const yMin = chartMode === 'pwm' ? 0 : -90;
    const yMax = chartMode === 'pwm' ? 180 : 100;
    const yTicks = chartMode === 'pwm' ? [0, 45, 90, 135, 180] : [-90, -45, 0, 50, 100];

    ctx.font = '11px JetBrains Mono, ui-monospace, monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    // เส้นตารางแนวนอนและตัวเลขแกน Y
    yTicks.forEach((tick) => {
      const norm = (tick - yMin) / (yMax - yMin);
      const y = padT + plotH * (1 - norm);
      ctx.strokeStyle = gridColor;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + plotW, y);
      ctx.stroke();

      ctx.fillStyle = textColor;
      ctx.fillText(String(tick), padL - 8, y);
    });

    // เส้นตารางแนวตั้ง
    const vSteps = 5;
    for (let i = 0; i <= vSteps; i++) {
      const x = padL + (plotW * i) / vSteps;
      ctx.strokeStyle = gridColor;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + plotH);
      ctx.stroke();
    }

    const keys = chartMode === 'pwm' ? ['s1', 's2', 's3', 's4'] : ['j1', 'j2', 'j3', 'j4'];
    const colorKeys = ['s1', 's2', 's3', 's4'];
    const sampleCount = history[keys[0]].length;

    if (sampleCount < 2) {
      ctx.fillStyle = textColor;
      ctx.textAlign = 'center';
      ctx.font = '12px Inter, Noto Sans Thai, sans-serif';
      ctx.fillText('รอสัญญาณ Telemetry จากหน้าควบคุมหลัก (index.html)...', padL + plotW / 2, padT + plotH / 2);
      ctx.restore();
      return;
    }

    keys.forEach((seriesKey, idx) => {
      const data = history[seriesKey];
      ctx.strokeStyle = SERIES_COLORS[colorKeys[idx]];
      ctx.lineWidth = 2.2;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      for (let i = 0; i < data.length; i++) {
        const x = padL + (plotW * i) / Math.max(1, MAX_POINTS - 1);
        const clampedVal = Math.max(yMin, Math.min(yMax, Number(data[i]) || 0));
        const norm = (clampedVal - yMin) / (yMax - yMin);
        const y = padT + plotH * (1 - norm);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });

    // ป้ายแกน X ด้านล่าง
    ctx.fillStyle = textColor;
    ctx.textAlign = 'left';
    ctx.font = '10.5px JetBrains Mono, ui-monospace, monospace';
    ctx.fillText(`-${Math.round((MAX_POINTS * 0.1))}s`, padL, height - 8);
    ctx.textAlign = 'right';
    ctx.fillText(chartPaused ? 'PAUSED' : 'LIVE (100 ms/sample)', padL + plotW, height - 8);

    ctx.restore();
  }

  function filterEvent(ev) {
    if (activeLogFilter === 'all') return true;
    if (activeLogFilter === 'alarm') return ev.level === 'warn' || ev.level === 'error' || ev.category === 'SAFETY';
    if (activeLogFilter === 'production') return ev.category === 'PRODUCTION';
    if (activeLogFilter === 'hardware') return ev.category === 'CAMERA' || ev.category === 'SERIAL';
    return true;
  }

  function renderEventLog() {
    const filtered = eventsCache.filter(filterEvent);
    if (ui.logCount) ui.logCount.textContent = String(filtered.length);
    if (!ui.logTbody) return;
    ui.logTbody.textContent = '';

    if (filtered.length === 0) {
      const tr = document.createElement('tr');
      const td = document.createElement('td');
      td.colSpan = 5;
      td.className = 'empty-cell';
      td.textContent = 'ยังไม่มีเหตุการณ์ในหมวดที่เลือก';
      tr.appendChild(td);
      ui.logTbody.appendChild(tr);
      return;
    }

    filtered.slice(0, 120).forEach((ev) => {
      const tr = document.createElement('tr');

      const tdTime = document.createElement('td');
      tdTime.className = 'mono-text';
      tdTime.textContent = Bus.formatTime(ev.timestamp);

      const tdLevel = document.createElement('td');
      const badge = document.createElement('span');
      badge.className = `level-badge ${ev.level || 'info'}`;
      badge.textContent = String(ev.level || 'info').toUpperCase();
      tdLevel.appendChild(badge);

      const tdCat = document.createElement('td');
      tdCat.className = 'mono-text';
      tdCat.textContent = ev.category || 'SYSTEM';

      const tdMsg = document.createElement('td');
      tdMsg.textContent = ev.message || '';

      const tdDetail = document.createElement('td');
      tdDetail.className = 'mono-text';
      tdDetail.textContent = ev.detail || '-';

      tr.appendChild(tdTime);
      tr.appendChild(tdLevel);
      tr.appendChild(tdCat);
      tr.appendChild(tdMsg);
      tr.appendChild(tdDetail);
      ui.logTbody.appendChild(tr);
    });
  }

  function sanitizeCsvCell(val) {
    let s = String(val ?? '');
    if (/^[=+\-@\t\r]/.test(s)) {
      s = `'${s}`;
    }
    return `"${s.replace(/"/g, '""')}"`;
  }

  function exportEventsCsv() {
    const rows = [['Timestamp', 'ISO_Time', 'Level', 'Category', 'Message', 'Detail']];
    eventsCache.forEach((ev) => {
      rows.push([
        Bus.formatDateTime(ev.timestamp),
        new Date(ev.timestamp).toISOString(),
        ev.level || 'info',
        ev.category || 'SYSTEM',
        ev.message || '',
        ev.detail || ''
      ]);
    });
    const csvContent = '\uFEFF' + rows
      .map((row) => row.map(sanitizeCsvCell).join(','))
      .join('\r\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `arm4dof_factory_log_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  }

  // ผูกปุ่มควบคุมกราฟและ Event Log
  if (ui.btnChartMode) {
    ui.btnChartMode.addEventListener('click', () => {
      chartMode = chartMode === 'pwm' ? 'joint' : 'pwm';
      ui.btnChartMode.textContent = chartMode === 'pwm' ? 'โหมด: PWM (0–180°)' : 'โหมด: Joint (°/%)';
      ui.btnChartMode.setAttribute('aria-pressed', String(chartMode === 'pwm'));
      updateChartLegend(lastSnapshot);
      drawTelemetryChart();
    });
  }

  if (ui.btnChartPause) {
    ui.btnChartPause.addEventListener('click', () => {
      chartPaused = !chartPaused;
      ui.btnChartPause.textContent = chartPaused ? '▶️ เล่นกราฟต่อ' : '⏸️ พักกราฟ';
      ui.btnChartPause.classList.toggle('active', chartPaused);
      ui.btnChartPause.setAttribute('aria-pressed', String(chartPaused));
      drawTelemetryChart();
    });
  }

  if (ui.btnChartClear) {
    ui.btnChartClear.addEventListener('click', () => {
      Object.keys(history).forEach((k) => { history[k].length = 0; });
      drawTelemetryChart();
    });
  }

  document.querySelectorAll('[data-log-filter]').forEach((btn) => {
    btn.addEventListener('click', () => {
      activeLogFilter = btn.dataset.logFilter || 'all';
      document.querySelectorAll('[data-log-filter]').forEach((b) => {
        const isAct = b === btn;
        b.classList.toggle('active', isAct);
        b.setAttribute('aria-pressed', String(isAct));
      });
      renderEventLog();
    });
  });

  if (ui.btnExportCsv) {
    ui.btnExportCsv.addEventListener('click', exportEventsCsv);
  }

  if (ui.btnClearLog) {
    ui.btnClearLog.addEventListener('click', () => {
      eventsCache = [];
      bus.clearEvents();
      renderEventLog();
    });
  }

  // รับข้อมูลสดจาก BroadcastChannel
  bus.on('telemetry', (snap) => {
    renderSnapshot(snap, true);
  });

  bus.on('event', (ev) => {
    if (!ev) return;
    if (!eventsCache.some((x) => x.id === ev.id)) {
      eventsCache.unshift(ev);
      if (eventsCache.length > 200) eventsCache.length = 200;
      renderEventLog();
    }
  });

  bus.on('eventsCleared', () => {
    eventsCache = [];
    renderEventLog();
  });

  if (typeof ResizeObserver !== 'undefined' && ui.chartCanvas && ui.chartCanvas.parentElement) {
    const ro = new ResizeObserver(() => drawTelemetryChart());
    ro.observe(ui.chartCanvas.parentElement);
  }
  window.addEventListener('resize', drawTelemetryChart);

  // ตรวจสอบสถานะ Live/Standby ทุก 1 วินาที และปิด Bus เมื่อปิดหน้าต่าง
  setInterval(updateConnectionBanner, 1000);
  window.addEventListener('pagehide', () => bus.close());

  // โหลด Snapshot ล่าสุดทันทีตอนเปิดหน้า และขอข้อมูลสดจากสถานีหลัก
  syncThemeButton();
  renderEventLog();
  const savedSnap = Bus.readSnapshot();
  if (savedSnap) {
    renderSnapshot(savedSnap, false);
  } else {
    drawTelemetryChart();
    updateConnectionBanner();
  }
  bus.requestSnapshot();
})();
