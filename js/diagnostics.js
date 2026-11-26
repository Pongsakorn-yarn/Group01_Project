/**
 * =============================================================================
 * @file        js/diagnostics.js
 * @project     ISY49303 IoT — Group01_Project (4-DOF Robotic Arm Digital Twin)
 * @description ลอจิกหน้า IoT Hardware Diagnostics & Pinout Reference
 *              ติดตามแพ็กเก็ต USB WebSerial แบบสด, ตรวจสอบตาราง Joint Safety Limits
 *              เทียบ Hard Limits, แสดงผังขา Pin Arduino/ESP32 และส่งออก Diagnostics JSON
 * =============================================================================
 */
(function () {
  'use strict';

  const Bus = window.ArmFactoryBus;
  if (!Bus) return;

  Bus.applySharedDisplaySettings();
  Bus.registerServiceWorker();
  Bus.initNavigationDrawer({ windowName: 'arm4dof_diagnostics' });

  const bus = Bus.createBus('diagnostics');

  const ui = {
    liveDot: document.getElementById('diag-live-dot'),
    liveText: document.getElementById('diag-live-text'),
    standbyBanner: document.getElementById('standby-banner'),
    standbyDetail: document.getElementById('standby-detail'),
    btnCycleTheme: document.getElementById('btn-cycle-theme'),
    themeIcon: document.getElementById('theme-icon'),
    themeLabel: document.getElementById('theme-label'),
    btnExportJson: document.getElementById('btn-export-diag-json'),
    serialStatus: document.getElementById('diag-serial-status'),
    btnPauseStream: document.getElementById('btn-pause-stream'),
    txHz: document.getElementById('diag-tx-hz'),
    txCount: document.getElementById('diag-tx-count'),
    handshake: document.getElementById('diag-handshake'),
    lastPacket: document.getElementById('diag-last-packet'),
    lastLimit: document.getElementById('diag-last-limit'),
    packetTerminal: document.getElementById('packet-terminal'),
    limitsSource: document.getElementById('diag-limits-source'),
    limitsTbody: document.getElementById('limits-tbody'),
    pinoutTbody: document.getElementById('pinout-tbody')
  };

  let streamPaused = false;
  let lastLiveReceivedAt = 0;
  let lastSnapshot = null;
  let lastLimitsSig = '';
  let lastPacketsSig = '';

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

  function renderPinoutTable() {
    if (!ui.pinoutTbody) return;
    ui.pinoutTbody.textContent = '';
    Bus.PINOUT_SPEC.forEach((row) => {
      const tr = document.createElement('tr');

      const tdServo = document.createElement('td');
      const strongServo = document.createElement('strong');
      strongServo.textContent = row.servo;
      const subServo = document.createElement('span');
      subServo.className = 'helper-text';
      subServo.textContent = row.name;
      tdServo.appendChild(strongServo);
      tdServo.appendChild(document.createElement('br'));
      tdServo.appendChild(subServo);

      const tdArd = document.createElement('td');
      const ardCode = document.createElement('code');
      ardCode.textContent = row.arduinoPin;
      tdArd.appendChild(ardCode);

      const tdEsp = document.createElement('td');
      const espCode = document.createElement('code');
      espCode.textContent = row.esp32Pin;
      tdEsp.appendChild(espCode);

      const tdNote = document.createElement('td');
      const homeCode = document.createElement('code');
      homeCode.textContent = row.homeCmd;
      const subNote = document.createElement('span');
      subNote.className = 'helper-text';
      subNote.textContent = row.note;
      tdNote.appendChild(homeCode);
      tdNote.appendChild(document.createElement('br'));
      tdNote.appendChild(subNote);

      tr.appendChild(tdServo);
      tr.appendChild(tdArd);
      tr.appendChild(tdEsp);
      tr.appendChild(tdNote);
      ui.pinoutTbody.appendChild(tr);
    });
  }

  function renderSafetyLimitsTable(snap, force = false) {
    if (!ui.limitsTbody) return;
    const limits = (snap && snap.jointLimits) ? snap.jointLimits : Bus.readJointLimits();
    const pwm = (snap && snap.pwm) ? snap.pwm : { s1: 90, s2: 90, s3: 90, s4: 63 };
    const clampedMap = (snap && snap.safety && snap.safety.limitClamped) ? snap.safety.limitClamped : {};
    const isLiveSrc = Boolean(snap && snap.live && Date.now() - lastLiveReceivedAt <= Bus.LIVE_STALE_MS);

    if (ui.limitsSource) {
      ui.limitsSource.textContent = isLiveSrc ? 'Source: Live Station' : 'Source: localStorage';
    }

    const sig = JSON.stringify([limits, pwm, clampedMap]);
    if (!force && sig === lastLimitsSig) return;
    lastLimitsSig = sig;

    ui.limitsTbody.textContent = '';
    ['j1', 'j2', 'j3', 'j4'].forEach((axis, idx) => {
      const hard = Bus.HARD_LIMITS[axis];
      const soft = limits[axis] || { min: hard.min, max: hard.max };
      const softPwmMin = Bus.axisValueToPwm(axis, soft.min);
      const softPwmMax = Bus.axisValueToPwm(axis, soft.max);
      const curPwm = pwm[`s${idx + 1}`];
      const isClamped = Boolean(clampedMap[axis]);
      const isRestricted = soft.min > hard.min || soft.max < hard.max;

      const tr = document.createElement('tr');

      const tdAxis = document.createElement('td');
      tdAxis.textContent = hard.label;

      const tdSoft = document.createElement('td');
      const softCode = document.createElement('code');
      softCode.textContent = `${soft.min}${hard.unit} .. ${soft.max}${hard.unit} (PWM ${softPwmMin}°–${softPwmMax}°)`;
      tdSoft.appendChild(softCode);

      const tdHard = document.createElement('td');
      const hardCode = document.createElement('code');
      hardCode.textContent = axis === 'j4'
        ? `${hard.min}%..${hard.max}% (HW ${hard.hwMin}°–${hard.hwMax}°)`
        : `${hard.min}°..+${hard.max}° (PWM ${hard.pwmMin}°–${hard.pwmMax}°)`;
      tdHard.appendChild(hardCode);

      const tdPwm = document.createElement('td');
      tdPwm.className = 'mono-text';
      tdPwm.textContent = `${curPwm}°`;

      const tdStatus = document.createElement('td');
      const badge = document.createElement('span');
      if (isClamped) {
        badge.className = 'level-badge warn';
        badge.textContent = 'CLAMPED';
      } else if (isRestricted) {
        badge.className = 'level-badge info';
        badge.textContent = 'CUSTOM SAFE';
      } else {
        badge.className = 'level-badge success';
        badge.textContent = 'SAFE OK';
      }
      tdStatus.appendChild(badge);

      tr.appendChild(tdAxis);
      tr.appendChild(tdSoft);
      tr.appendChild(tdHard);
      tr.appendChild(tdPwm);
      tr.appendChild(tdStatus);
      ui.limitsTbody.appendChild(tr);
    });
  }

  function renderPacketStream(packets, force = false) {
    if (streamPaused || !ui.packetTerminal) return;
    const list = Array.isArray(packets) ? packets : [];
    const sig = list.length > 0 ? `${list.length}:${list[0].ts}:${list[0].dir}:${list[0].text}` : 'EMPTY';
    if (!force && sig === lastPacketsSig) return;
    lastPacketsSig = sig;

    ui.packetTerminal.textContent = '';

    if (list.length === 0) {
      const line = document.createElement('div');
      line.className = 'packet-line';
      const ts = document.createElement('span');
      ts.className = 'packet-ts';
      ts.textContent = '--:--:--';
      const dir = document.createElement('span');
      dir.className = 'packet-dir tx';
      dir.textContent = 'IDLE';
      const msg = document.createElement('span');
      msg.textContent = 'ยังไม่มีแพ็กเก็ต USB Serial — เชื่อมต่อบอร์ดบนหน้า 3D Control เพื่อดูสตรีมสด';
      line.appendChild(ts);
      line.appendChild(dir);
      line.appendChild(msg);
      ui.packetTerminal.appendChild(line);
      return;
    }

    list.forEach((pkt) => {
      const line = document.createElement('div');
      line.className = 'packet-line';

      const ts = document.createElement('span');
      ts.className = 'packet-ts';
      ts.textContent = Bus.formatTime(pkt.ts);

      const dir = document.createElement('span');
      const dirKey = String(pkt.dir || 'TX').toLowerCase();
      dir.className = `packet-dir ${dirKey === 'rx' ? 'rx' : (dirKey === 'lim' ? 'lim' : 'tx')}`;
      dir.textContent = String(pkt.dir || 'TX').toUpperCase();

      const msg = document.createElement('span');
      msg.textContent = pkt.text || '';

      line.appendChild(ts);
      line.appendChild(dir);
      line.appendChild(msg);
      ui.packetTerminal.appendChild(line);
    });
  }

  function updateConnectionBanner() {
    const isLive = lastSnapshot && lastSnapshot.live && (Date.now() - lastLiveReceivedAt <= Bus.LIVE_STALE_MS);
    if (isLive) {
      ui.liveDot.className = 'dot active';
      ui.liveText.textContent = 'Live Station Connected';
      ui.standbyBanner.hidden = true;
    } else {
      ui.liveDot.className = 'dot warn';
      ui.liveText.textContent = 'Standby / Waiting';
      ui.standbyBanner.hidden = false;
      if (lastSnapshot && lastSnapshot.timestamp) {
        ui.standbyDetail.textContent = `ยังไม่ได้เชื่อมต่อสถานีสด — แสดงข้อมูล Snapshot ล่าสุดเมื่อ ${Bus.formatDateTime(lastSnapshot.timestamp)}`;
      } else {
        ui.standbyDetail.textContent = 'กำลังรอข้อมูลสดจากหน้าควบคุมหลัก (index.html) — แสดงค่าขอบเขตและผังต่อสายที่บันทึกไว้ในเครื่อง';
      }
    }
  }

  function renderSnapshot(snap, fromLiveBus = false) {
    if (!snap) return;
    lastSnapshot = snap;
    if (fromLiveBus && snap.live) {
      lastLiveReceivedAt = Date.now();
    }

    const ser = snap.serial || {};
    ui.serialStatus.textContent = ser.connected
      ? (ser.boardReady ? 'Connected • READY:4DOF_BRIDGE' : 'Connected • 115200')
      : 'Disconnected';
    ui.txHz.textContent = `${Number(ser.txHz || 0).toFixed(1)} Hz`;
    ui.txCount.textContent = String(ser.txCount || 0);
    ui.handshake.textContent = ser.limitsAcked
      ? 'READY:4DOF_BRIDGE / OK:LIMITS'
      : (ser.lastHandshake || 'Waiting');
    ui.lastPacket.textContent = ser.lastPacket || '90,90,90,63';
    ui.lastLimit.textContent = ser.lastLimitCmd || Bus.buildLimitCommandFromLimits(snap.jointLimits);

    renderPacketStream(ser.recentPackets || []);
    renderSafetyLimitsTable(snap);
    updateConnectionBanner();
  }

  function exportDiagnosticsJson() {
    const isLive = Boolean(lastSnapshot && lastSnapshot.live && (Date.now() - lastLiveReceivedAt <= Bus.LIVE_STALE_MS));
    const currentLimits = (lastSnapshot && lastSnapshot.jointLimits) ? lastSnapshot.jointLimits : Bus.readJointLimits();
    const report = {
      report_type: '4DOF_Robotic_Arm_IoT_Diagnostics',
      generated_at: new Date().toISOString(),
      station_live: isLive,
      browser_webserial_supported: 'serial' in navigator,
      display_settings: Bus.readDisplaySettings(),
      joint_limits_soft: currentLimits,
      joint_limits_hard: Bus.HARD_LIMITS,
      limit_command_preview: Bus.buildLimitCommandFromLimits(currentLimits),
      pinout_reference: Bus.PINOUT_SPEC,
      latest_snapshot: lastSnapshot || Bus.readSnapshot(),
      recent_events: Bus.readEvents().slice(0, 50)
    };
    const blob = new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `arm4dof_iot_diagnostics_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }

  if (ui.btnPauseStream) {
    ui.btnPauseStream.addEventListener('click', () => {
      streamPaused = !streamPaused;
      ui.btnPauseStream.textContent = streamPaused ? '▶️ เล่นสตรีมต่อ' : '⏸️ พักสตรีม';
      ui.btnPauseStream.classList.toggle('active', streamPaused);
      ui.btnPauseStream.setAttribute('aria-pressed', String(streamPaused));
      if (!streamPaused && lastSnapshot && lastSnapshot.serial) {
        renderPacketStream(lastSnapshot.serial.recentPackets || [], true);
      }
    });
  }

  if (ui.btnExportJson) {
    ui.btnExportJson.addEventListener('click', exportDiagnosticsJson);
  }

  bus.on('telemetry', (snap) => {
    renderSnapshot(snap, true);
  });

  window.addEventListener('storage', (e) => {
    if (e && e.key === Bus.JOINT_LIMITS_STORAGE_KEY) {
      renderSafetyLimitsTable(lastSnapshot);
      ui.lastLimit.textContent = Bus.buildLimitCommandFromLimits(Bus.readJointLimits());
    }
  });

  setInterval(updateConnectionBanner, 1000);
  window.addEventListener('pagehide', () => bus.close());

  syncThemeButton();
  renderPinoutTable();
  const savedSnap = Bus.readSnapshot();
  if (savedSnap) {
    renderSnapshot(savedSnap, false);
  } else {
    ui.lastLimit.textContent = Bus.buildLimitCommandFromLimits(Bus.readJointLimits());
    renderSafetyLimitsTable(null);
    updateConnectionBanner();
  }
  bus.requestSnapshot();
})();
