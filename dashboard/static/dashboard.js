/**
 * dashboard.js — QuadBot-AI Dashboard Client
 * =============================================
 * WebSocket connections for telemetry, camera, and commands.
 * IMU gauge rendering and UI updates.
 */

// ── State ──────────────────────────────────────────────────────

const state = {
    connected: false,
    telemetryWS: null,
    cameraWS: null,
    commandWS: null,
    camFrames: 0,
    lastFPSTime: Date.now(),
    fps: 0,
    controlMode: 'autonomous',
    joystickActive: false,
};

// ── DOM Elements ───────────────────────────────────────────────

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

const dom = {
    connectionStatus: $('#connectionStatus'),
    statusDot: $('#connectionStatus .status-dot'),
    statusText: $('#connectionStatus .status-text'),
    modeBadge: $('#modeBadge'),
    btnEmergencyStop: $('#btnEmergencyStop'),
    cameraCanvas: $('#cameraCanvas'),
    cameraOverlay: $('#cameraOverlay'),
    camFPS: $('#camFPS'),
    sceneDescription: $('#sceneDescription'),
    robotState: $('#robotState'),
    currentGait: $('#currentGait'),
    currentSpeed: $('#currentSpeed'),
    safetyStatus: $('#safetyStatus'),
    rollValue: $('#rollValue'),
    pitchValue: $('#pitchValue'),
    yawValue: $('#yawValue'),
    distanceValue: $('#distanceValue'),
    obstacleIndicator: $('#obstacleIndicator'),
    obstacleText: $('#obstacleText'),
    speedSlider: $('#speedSlider'),
    speedLabel: $('#speedLabel'),
    commandInput: $('#commandInput'),
    btnSendCommand: $('#btnSendCommand'),
    commandLog: $('#commandLog'),
    decisionLog: $('#decisionLog'),
    joystickPanel: $('#joystickPanel'),
    joystickZone: $('#joystickZone'),
    joystickStick: $('#joystickStick'),
    manualBadge: $('#manualBadge'),
    joyX: $('#joyX'),
    joyY: $('#joyY'),
    joyAction: $('#joyAction'),
};

const cameraCtx = dom.cameraCanvas.getContext('2d');

// ── WebSocket Connections ──────────────────────────────────────

function connectTelemetry() {
    const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);

    ws.onopen = () => {
        state.connected = true;
        state.telemetryWS = ws;
        updateConnectionUI(true);
    };

    ws.onmessage = (e) => {
        try {
            const data = JSON.parse(e.data);
            updateTelemetry(data);
        } catch (err) {
            console.error('Telemetry parse error:', err);
        }
    };

    ws.onclose = () => {
        state.connected = false;
        updateConnectionUI(false);
        setTimeout(connectTelemetry, 3000);
    };

    ws.onerror = () => ws.close();
}

function connectCamera() {
    const ws = new WebSocket(`ws://${location.host}/ws/camera`);

    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        state.cameraWS = ws;
        dom.cameraOverlay.classList.add('hidden');
    };

    ws.onmessage = (e) => {
        const blob = new Blob([e.data], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);
        const img = new Image();
        img.onload = () => {
            cameraCtx.drawImage(img, 0, 0,
                dom.cameraCanvas.width, dom.cameraCanvas.height);
            URL.revokeObjectURL(url);

            // FPS counter
            state.camFrames++;
            const now = Date.now();
            if (now - state.lastFPSTime > 1000) {
                state.fps = state.camFrames;
                state.camFrames = 0;
                state.lastFPSTime = now;
                dom.camFPS.textContent = state.fps + ' FPS';
            }
        };
        img.src = url;
    };

    ws.onclose = () => {
        dom.cameraOverlay.classList.remove('hidden');
        setTimeout(connectCamera, 3000);
    };

    ws.onerror = () => ws.close();
}

function connectCommand() {
    const ws = new WebSocket(`ws://${location.host}/ws/command`);

    ws.onopen = () => { state.commandWS = ws; };

    ws.onmessage = (e) => {
        try {
            const data = JSON.parse(e.data);
            addLogEntry('system', `Robot: ${data.robot_state || 'ok'}`);
        } catch (err) {
            addLogEntry('system', e.data);
        }
    };

    ws.onclose = () => setTimeout(connectCommand, 3000);
    ws.onerror = () => ws.close();
}

// ── UI Updates ─────────────────────────────────────────────────

function updateConnectionUI(connected) {
    dom.statusDot.className = 'status-dot ' + (connected ? 'connected' : '');
    dom.statusText.textContent = connected ? 'Connected' : 'Reconnecting...';
}

function updateTelemetry(data) {
    // State
    if (data.state_machine) {
        dom.robotState.textContent = (data.state_machine.state || 'idle').toUpperCase();
    }

    // Gait / Body
    if (data.body) {
        const gait = data.body.gait || {};
        dom.currentGait.textContent = gait.gait || 'stand';
        dom.currentSpeed.textContent = (gait.speed || 0).toFixed(1);
    }

    // Safety
    if (data.safety) {
        const safe = data.safety.is_safe;
        dom.safetyStatus.textContent = safe ? 'SAFE' : 'ALERT';
        dom.safetyStatus.className = 'state-value ' + (safe ? 'safe' : 'danger');
    }

    // IMU
    if (data.imu) {
        const roll = data.imu.roll || 0;
        const pitch = data.imu.pitch || 0;
        const yaw = data.imu.yaw || 0;
        dom.rollValue.textContent = roll.toFixed(1) + '°';
        dom.pitchValue.textContent = pitch.toFixed(1) + '°';
        dom.yawValue.textContent = yaw.toFixed(1) + '°';
        drawGauge('rollGauge', roll, 'Roll');
        drawGauge('pitchGauge', pitch, 'Pitch');
        drawGauge('yawGauge', yaw, 'Yaw');
    }

    // Distance
    if (data.distance) {
        const dist = data.distance.distance_cm;
        dom.distanceValue.textContent = dist !== undefined ? dist.toFixed(0) : '--';
        const obstacle = data.distance.obstacle;
        dom.obstacleText.textContent = obstacle ? 'OBSTACLE!' : 'Clear';
        dom.obstacleIndicator.className = 'sensor-item ' + (obstacle ? 'danger' : '');
    }

    // Mode
    if (data.mode) {
        dom.modeBadge.textContent = data.mode.toUpperCase();
    }

    // Vision
    if (data.vision && data.vision.scene) {
        dom.sceneDescription.textContent = data.vision.scene;
    }

    // Decision
    if (data.decision && data.decision.last_action) {
        addDecisionEntry(data.decision.last_action, data.decision.last_reasoning);
    }

    // Control Mode
    if (data.control) {
        const mode = data.control.mode || 'autonomous';
        state.controlMode = mode;
        updateModeUI(mode);
    }
}

// ── IMU Gauge Drawing ──────────────────────────────────────────

function drawGauge(canvasId, angle, label) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    const cx = w / 2, cy = h / 2, r = w / 2 - 10;

    ctx.clearRect(0, 0, w, h);

    // Background arc
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 6;
    ctx.stroke();

    // Value arc
    const startAngle = -Math.PI / 2;
    const sweepAngle = (angle / 180) * Math.PI;
    ctx.beginPath();
    ctx.arc(cx, cy, r, startAngle, startAngle + sweepAngle);
    const color = Math.abs(angle) > 30 ? '#ef4444' :
        Math.abs(angle) > 15 ? '#f59e0b' : '#06b6d4';
    ctx.strokeStyle = color;
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.stroke();

    // Needle
    const needleAngle = startAngle + sweepAngle;
    const nx = cx + (r - 15) * Math.cos(needleAngle);
    const ny = cy + (r - 15) * Math.sin(needleAngle);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(nx, ny);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    // Value text
    ctx.fillStyle = '#e2e8f0';
    ctx.font = '600 14px "JetBrains Mono"';
    ctx.textAlign = 'center';
    ctx.fillText(angle.toFixed(1) + '°', cx, cy + r + 4);
}

// ── Command Log ────────────────────────────────────────────────

function addLogEntry(type, text) {
    const entry = document.createElement('div');
    entry.className = 'log-entry ' + type;
    entry.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
    dom.commandLog.appendChild(entry);
    dom.commandLog.scrollTop = dom.commandLog.scrollHeight;

    // Keep max 50 entries
    while (dom.commandLog.children.length > 50) {
        dom.commandLog.removeChild(dom.commandLog.firstChild);
    }
}

function addDecisionEntry(action, reasoning) {
    const entry = document.createElement('div');
    entry.className = 'decision-entry';
    entry.innerHTML = `<span class="action-name">${action}</span> <span class="reasoning">— ${reasoning || ''}</span>`;
    dom.decisionLog.insertBefore(entry, dom.decisionLog.firstChild);

    while (dom.decisionLog.children.length > 20) {
        dom.decisionLog.removeChild(dom.decisionLog.lastChild);
    }
}

// ── Controls ───────────────────────────────────────────────────

function sendAction(action) {
    const speed = parseFloat(dom.speedSlider.value);
    const payload = { action, speed, duration: 0 };

    fetch('/api/action', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
    }).catch(err => console.error('Action send error:', err));

    addLogEntry('user', `Action: ${action} (speed=${speed})`);
}

function sendCommand(text) {
    if (!text.trim()) return;

    addLogEntry('user', text);

    if (state.commandWS && state.commandWS.readyState === WebSocket.OPEN) {
        state.commandWS.send(text);
    } else {
        fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: text }),
        }).catch(err => addLogEntry('error', 'Send failed'));
    }
}

// ── Event Listeners ────────────────────────────────────────────

// Gait control buttons
$$('.ctrl-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        const action = btn.dataset.action;
        if (action) sendAction(action);
    });
});

// Emergency stop
dom.btnEmergencyStop.addEventListener('click', () => {
    fetch('/api/emergency_stop', { method: 'POST' })
        .catch(err => console.error('E-stop error:', err));
    addLogEntry('error', 'EMERGENCY STOP');
});

// Speed slider
dom.speedSlider.addEventListener('input', (e) => {
    dom.speedLabel.textContent = parseFloat(e.target.value).toFixed(1);
});

// Command input
dom.commandInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        sendCommand(dom.commandInput.value);
        dom.commandInput.value = '';
    }
});

dom.btnSendCommand.addEventListener('click', () => {
    sendCommand(dom.commandInput.value);
    dom.commandInput.value = '';
});

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {
    if (document.activeElement === dom.commandInput) return;

    const keyMap = {
        'w': 'walk_forward',
        's': 'stop',
        'a': 'turn_left',
        'd': 'turn_right',
        'q': 'side_left',
        'e': 'side_right',
        'r': 'trot',
        ' ': 'stand',
    };

    const action = keyMap[e.key.toLowerCase()];
    if (action) {
        e.preventDefault();
        sendAction(action);
    }

    // Escape = emergency stop
    if (e.key === 'Escape') {
        fetch('/api/emergency_stop', { method: 'POST' });
        addLogEntry('error', 'EMERGENCY STOP (Esc)');
    }
});

// ── Initialize ─────────────────────────────────────────────────

function init() {
    connectTelemetry();
    connectCamera();
    connectCommand();

    // Draw initial gauges
    drawGauge('rollGauge', 0, 'Roll');
    drawGauge('pitchGauge', 0, 'Pitch');
    drawGauge('yawGauge', 0, 'Yaw');

    addLogEntry('system', 'QuadBot-AI Dashboard initialized');
    addLogEntry('system', 'Keyboard: W/A/S/D = move, R = trot, Space = stand, Esc = E-stop');
}

init();

// ── Mode Switcher ──────────────────────────────────────────────

function updateModeUI(mode) {
    $$('.mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });

    const isManual = mode === 'manual' || mode === 'hybrid';
    dom.joystickPanel.classList.toggle('disabled-panel', !isManual);
    dom.manualBadge.textContent = isManual ? 'ACTIVE' : 'DISABLED';
    dom.manualBadge.className = 'badge ' + (isManual ? 'badge-active' : '');
}

function switchMode(mode) {
    fetch('/api/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
    })
        .then(r => r.json())
        .then(data => {
            if (data.status === 'ok') {
                state.controlMode = mode;
                updateModeUI(mode);
                addLogEntry('system', `Mode switched to: ${mode.toUpperCase()}`);
            } else {
                addLogEntry('error', `Mode switch failed: ${data.error}`);
            }
        })
        .catch(err => addLogEntry('error', 'Mode switch failed'));
}

$$('.mode-btn').forEach(btn => {
    btn.addEventListener('click', () => switchMode(btn.dataset.mode));
});

// ── Virtual Joystick ───────────────────────────────────────────

(function initJoystick() {
    const zone = dom.joystickZone;
    const stick = dom.joystickStick;
    if (!zone || !stick) return;

    let active = false;
    let baseRect = null;
    let sendTimer = null;

    function getPos(e) {
        const touch = e.touches ? e.touches[0] : e;
        return { x: touch.clientX, y: touch.clientY };
    }

    function onStart(e) {
        e.preventDefault();
        if (state.controlMode !== 'manual' && state.controlMode !== 'hybrid') return;
        active = true;
        state.joystickActive = true;
        baseRect = zone.querySelector('.joystick-base').getBoundingClientRect();
    }

    function onMove(e) {
        if (!active || !baseRect) return;
        e.preventDefault();
        const pos = getPos(e);
        const cx = baseRect.left + baseRect.width / 2;
        const cy = baseRect.top + baseRect.height / 2;
        const maxR = baseRect.width / 2 - 15;

        let dx = pos.x - cx;
        let dy = -(pos.y - cy); // Invert Y: up = positive
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist > maxR) {
            dx = dx / dist * maxR;
            dy = dy / dist * maxR;
        }

        const normX = dx / maxR;
        const normY = dy / maxR;

        stick.style.transform = `translate(${dx}px, ${-dy}px)`;

        dom.joyX.textContent = normX.toFixed(2);
        dom.joyY.textContent = normY.toFixed(2);

        // Determine action
        let action = 'stop';
        if (Math.abs(normY) > 0.15 || Math.abs(normX) > 0.15) {
            if (Math.abs(normY) > Math.abs(normX)) {
                action = normY > 0 ? 'walk_forward' : 'stop';
            } else {
                action = normX > 0 ? 'turn_right' : 'turn_left';
            }
        }
        dom.joyAction.textContent = action;

        // Throttled send
        clearTimeout(sendTimer);
        sendTimer = setTimeout(() => {
            const speed = Math.min(1.0, dist / maxR);
            fetch('/api/manual', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action, speed: parseFloat(speed.toFixed(2)) }),
            }).catch(() => { });
        }, 80);
    }

    function onEnd() {
        if (!active) return;
        active = false;
        state.joystickActive = false;
        stick.style.transform = 'translate(0, 0)';
        dom.joyX.textContent = '0.00';
        dom.joyY.textContent = '0.00';
        dom.joyAction.textContent = 'stop';
        clearTimeout(sendTimer);
        fetch('/api/manual', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action: 'stop', speed: 0 }),
        }).catch(() => { });
    }

    zone.addEventListener('mousedown', onStart);
    zone.addEventListener('touchstart', onStart, { passive: false });
    document.addEventListener('mousemove', onMove);
    document.addEventListener('touchmove', onMove, { passive: false });
    document.addEventListener('mouseup', onEnd);
    document.addEventListener('touchend', onEnd);
})();
