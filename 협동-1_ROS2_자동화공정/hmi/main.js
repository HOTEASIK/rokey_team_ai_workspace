import './styles.css';

const state = {
  running: true,
  estop: false,
  mode: 'AUTO',
  speed: 42,
  progress: 68,
  x: 61,
  y: 45,
  heading: 82,
  linear: 0.42,
  angular: 0.08,
  latency: 23,
  loss: 0.12,
  force: 18.4,
  distance: 1.84,
  collision: false,
  filter: 'ALL',
  logs: [
    { time: '10:09:55', type: 'INFO', source: 'ROS2', message: 'odom stream synchronized · 10 Hz', state: 'OK' },
    { time: '10:09:54', type: 'INFO', source: 'CONTROL', message: 'AUTO mode confirmed · route MT-2481 active', state: 'OK' },
    { time: '10:09:52', type: 'INFO', source: 'SENSOR', message: 'force monitoring nominal · 18.4 N', state: 'OK' },
    { time: '10:09:49', type: 'WARN', source: 'NETWORK', message: 'latency threshold crossed briefly · 32 ms', state: 'WATCH' },
    { time: '10:09:45', type: 'INFO', source: 'PLC', message: 'conveyer handshake accepted · dock B', state: 'OK' },
    { time: '10:09:40', type: 'INFO', source: 'COMMAND', message: 'NAVIGATE_TO(B) queued with priority NORMAL', state: 'OK' },
    { time: '10:09:37', type: 'INFO', source: 'SYSTEM', message: 'M0609 safety controller armed', state: 'OK' },
    { time: '10:09:31', type: 'INFO', source: 'ROS2', message: '/cmd_vel publisher connected', state: 'OK' },
  ],
  devices: { Conveyer: true, Motor: true, PLC: true },
};

const $ = (id) => document.getElementById(id);
const nowTime = () => new Date().toLocaleTimeString('en-GB', { hour12: false });

function addLog(type, source, message, logState = type === 'ERROR' ? 'FAULT' : type === 'WARN' ? 'WATCH' : 'OK') {
  state.logs.unshift({ time: nowTime(), type, source, message, state: logState });
  state.logs = state.logs.slice(0, 36);
  renderLogs();
  renderHistory();
}

function renderLogs() {
  const visible = state.filter === 'ALL' ? state.logs : state.logs.filter((log) => log.type === state.filter);
  $('consoleLog').innerHTML = visible.map((log) => `<div class="log-row"><time>${log.time}</time><span class="log-type ${log.type.toLowerCase()}">${log.type}</span><span class="log-source">${log.source}</span><span class="log-message">${log.message}</span><span class="log-state ${log.type.toLowerCase()}">${log.state}</span></div>`).join('');
  $('logCount').textContent = String(visible.length).padStart(2, '0');
  $('visibleCount').textContent = visible.length;
  $('totalCount').textContent = state.logs.length;
  const log = $('consoleLog');
  log.scrollTop = 0;
}

function renderHistory() {
  $('historyList').innerHTML = state.logs.slice(0, 4).map((log) => `<div class="history-item"><time>${log.time.slice(0, 5)}</time><b>${log.type}</b><span>${log.source}</span></div>`).join('');
}

function updateLatencyBars() {
  const container = $('latencyBars');
  if (!container.children.length) {
    for (let i = 0; i < 25; i += 1) {
      const bar = document.createElement('i');
      container.appendChild(bar);
    }
  }
  [...container.children].forEach((bar, index) => {
    const seed = Math.sin(Date.now() / 1200 + index * 1.7) * 13;
    const value = Math.max(12, Math.min(94, 38 + seed + (index > 21 ? state.latency - 23 : 0)));
    bar.style.height = `${value}%`;
  });
}

function updateForceTrace() {
  const points = [];
  for (let i = 0; i <= 280; i += 16) {
    const base = 24 + Math.sin((Date.now() / 800) + i / 24) * 5;
    const noise = Math.sin(i * 1.6 + Date.now() / 370) * 4;
    points.push(`${i},${Math.max(7, Math.min(39, base + noise))}`);
  }
  $('forceTrace').setAttribute('points', points.join(' '));
}

function evaluateSafety() {
  const wasCollision = state.collision;
  state.collision = state.force > 22.5 || state.distance < 1.68;
  if (state.collision && !wasCollision) {
    addLog('ERROR', 'SAFETY', 'collision envelope threshold crossed · motion constrained', 'FAULT');
  } else if (!state.collision && wasCollision) {
    addLog('INFO', 'SAFETY', 'collision envelope clear · sensor values nominal', 'OK');
  }
}

function setRunning(running, source = 'CONTROL') {
  if (running && state.estop) {
    addLog('WARN', source, 'start request blocked · emergency stop is active');
    updateUI();
    return;
  }
  state.running = running;
  state.progress = running ? Math.max(state.progress, 68) : state.progress;
  addLog('INFO', source, running ? 'START command accepted · mission resumed' : 'STOP command accepted · motion held');
  updateUI();
}

function triggerEstop() {
  if (!state.estop) {
    state.estop = true;
    state.running = false;
    addLog('ERROR', 'SAFETY', 'EMERGENCY STOP engaged · /cmd_vel zeroed', 'STOPPED');
  } else {
    state.estop = false;
    state.running = false;
    addLog('WARN', 'SAFETY', 'emergency stop reset · operator acknowledgement received', 'ARMED');
  }
  updateUI();
}

function changeMode(mode) {
  if (state.mode === mode) return;
  state.mode = mode;
  addLog('INFO', 'CONTROL', `${mode} mode selected · authority handoff complete`);
  updateUI();
}

function moveRobot(direction) {
  if (state.estop) {
    addLog('WARN', 'CONTROL', `${direction.toUpperCase()} ignored · emergency stop active`);
    return;
  }
  if (!state.running) {
    addLog('WARN', 'CONTROL', `${direction.toUpperCase()} ignored · robot is stopped`);
    return;
  }
  const deltas = { up: [0, -2.2], down: [0, 2.2], left: [-2.2, 0], right: [2.2, 0], stop: [0, 0] };
  const [dx, dy] = deltas[direction] || [0, 0];
  state.x = Math.max(8, Math.min(88, state.x + dx));
  state.y = Math.max(14, Math.min(80, state.y + dy));
  state.heading = direction === 'up' ? 0 : direction === 'right' ? 90 : direction === 'down' ? 180 : direction === 'left' ? 270 : state.heading;
  const labels = { up: 'MANUAL / FORWARD', down: 'MANUAL / REVERSE', left: 'MANUAL / LEFT', right: 'MANUAL / RIGHT', stop: 'MANUAL / HOLD' };
  $('lastCommand').textContent = labels[direction] || 'MANUAL / HOLD';
  $('activeCommand').textContent = direction === 'stop' ? 'HOLD POSITION' : `MANUAL ${direction.toUpperCase()}`;
  $('activeCommandIcon').textContent = direction === 'up' ? '↑' : direction === 'down' ? '↓' : direction === 'left' ? '←' : direction === 'right' ? '→' : '•';
  if (state.mode !== 'MANUAL') changeMode('MANUAL');
  addLog('INFO', 'COMMAND', `manual ${direction} vector sent · speed ${Math.round(state.speed)}%`);
  updateUI();
}

function updateUI() {
  const status = state.estop ? 'E-STOP' : state.running ? 'RUNNING' : 'IDLE';
  $('statusText').textContent = status;
  $('stageState').textContent = status;
  $('progressValue').textContent = `${Math.round(state.progress)}%`;
  $('progressBar').style.width = `${state.progress}%`;
  $('progressBar').nextElementSibling.style.left = `${state.progress}%`;
  $('linearSpeed').textContent = state.estop ? '0.00' : state.mode === 'MANUAL' ? (state.speed / 100).toFixed(2) : state.linear.toFixed(2);
  $('angularSpeed').textContent = state.estop ? '0.00' : state.mode === 'MANUAL' ? '0.15' : state.angular.toFixed(2);
  $('modeValue').innerHTML = `<span></span>${state.mode}`;
  $('speedValue').textContent = state.speed;
  $('speedSlider').value = state.speed;
  $('robotMotion').textContent = state.estop ? 'STOPPED · SAFETY' : state.running ? `${state.mode === 'AUTO' ? 'MOVING' : 'MANUAL'} · ${state.mode}` : 'HOLD · IDLE';
  $('robot').style.left = `${state.x}%`;
  $('robot').style.top = `${state.y}%`;
  $('robot').style.transform = `translate(-50%, -50%) rotate(${state.heading}deg)`;
  $('headingValue').textContent = `${String(Math.round(state.heading)).padStart(3, '0')}°`;
  $('posX').textContent = (4.218 + (state.x - 61) / 18).toFixed(3);
  $('posY').textContent = (12.880 + (45 - state.y) / 20).toFixed(3);
  $('latencyValue').textContent = state.latency;
  $('packetLoss').textContent = `${state.loss.toFixed(2)}% LOSS`;
  $('forceValue').textContent = state.force.toFixed(1);
  $('forcePercent').textContent = `${Math.round(state.force)}%`;
  $('forceRing').style.transform = `rotate(${Math.min(280, state.force * 9)}deg)`;
  $('distanceValue').textContent = state.distance.toFixed(2);
  $('collisionValue').textContent = state.collision ? 'DETECTED' : 'CLEAR';
  $('collisionValue').className = state.collision ? 'error-text' : 'clear-text';
  const speedLimited = state.speed > 80;
  $('speedLimitValue').textContent = speedLimited ? 'LIMITED / 80%' : 'WITHIN LIMIT';
  $('speedLimitValue').className = speedLimited ? 'error-text' : 'safe-text';
  $('faultValue').textContent = state.collision ? '1 ACTIVE' : '0 ACTIVE';
  $('faultValue').className = state.collision ? 'error-text' : '';
  $('statusStamp').classList.toggle('alert', state.estop);
  $('statusStamp').style.color = state.estop ? 'var(--red)' : 'var(--green)';
  $('statusStamp').style.borderColor = state.estop ? 'rgba(240,93,94,.55)' : 'rgba(83,224,179,.55)';
  $('statusStamp').querySelector('i').style.background = state.estop ? 'var(--red)' : 'var(--green)';
  $('estopBtn').classList.toggle('triggered', state.estop);
  $('estopLabel').textContent = state.estop ? 'RESET E-STOP' : 'EMERGENCY STOP';
  $('safetyBanner').className = `safety-banner ${state.estop ? 'alert' : 'safe'}`;
  $('safetyBanner').querySelector('.safety-symbol').textContent = state.estop ? '!' : '✓';
  $('safetyTitle').textContent = state.estop ? 'EMERGENCY STOP ACTIVE' : 'ALL SYSTEMS NOMINAL';
  $('safetyDetail').textContent = state.estop ? 'Motion inhibited · operator reset required' : 'Emergency stop circuit is armed';
  $('safetyEstop').textContent = state.estop ? 'TRIGGERED / STOPPED' : 'ARMED / READY';
  $('safetyEstop').className = state.estop ? 'error-text' : 'safe-text';
  document.querySelectorAll('.mode-btn').forEach((btn) => btn.classList.toggle('active', btn.dataset.mode === state.mode));
  document.querySelectorAll('.device-row').forEach((row) => {
    const on = state.devices[row.dataset.device];
    row.classList.toggle('offline', !on);
    row.querySelector('small').textContent = on ? 'CONNECTED' : 'OFFLINE';
  });
  $('etaValue').textContent = state.running ? `00:${String(Math.max(12, 42 - Math.floor((state.progress - 68) * 1.5))).padStart(2, '0')}` : '--:--';
  updateLatencyBars();
}

function setFilter(filter) {
  state.filter = filter;
  document.querySelectorAll('[data-filter]').forEach((button) => button.classList.toggle('active', button.dataset.filter === filter));
  renderLogs();
}

$('startBtn').addEventListener('click', () => setRunning(true));
$('stopBtn').addEventListener('click', () => setRunning(false));
$('estopBtn').addEventListener('click', triggerEstop);
$('speedSlider').addEventListener('input', (event) => {
  state.speed = Number(event.target.value);
  state.linear = state.speed / 100;
  updateUI();
});
document.querySelectorAll('.mode-btn').forEach((button) => button.addEventListener('click', () => changeMode(button.dataset.mode)));
document.querySelectorAll('.d-pad button').forEach((button) => button.addEventListener('click', () => moveRobot(button.dataset.key)));
document.querySelectorAll('.link-toggle').forEach((button) => button.addEventListener('click', () => {
  const device = button.dataset.device;
  state.devices[device] = !state.devices[device];
  addLog(state.devices[device] ? 'INFO' : 'WARN', device.toUpperCase(), `${device} link ${state.devices[device] ? 'reconnected' : 'disconnected'} by operator`);
  updateUI();
}));
document.querySelectorAll('[data-filter]').forEach((button) => button.addEventListener('click', () => setFilter(button.dataset.filter)));
$('refreshBtn').addEventListener('click', () => { addLog('INFO', 'SYSTEM', 'telemetry stream manually refreshed'); updateUI(); });
$('focusBtn').addEventListener('click', () => $('simulationCard').classList.toggle('focused'));

document.addEventListener('keydown', (event) => {
  if (['INPUT', 'TEXTAREA'].includes(event.target.tagName)) return;
  const keyMap = { w: 'up', a: 'left', s: 'down', d: 'right', ArrowUp: 'up', ArrowLeft: 'left', ArrowDown: 'down', ArrowRight: 'right' };
  if (keyMap[event.key]) { event.preventDefault(); moveRobot(keyMap[event.key]); }
  if (event.key === ' ') { event.preventDefault(); setRunning(!state.running, 'KEYBOARD'); }
  if (event.key.toLowerCase() === 'e') triggerEstop();
});

setInterval(() => {
  $('topClock').textContent = nowTime();
  if (state.running && !state.estop) {
    state.progress = Math.min(98, state.progress + 0.18);
    state.x += 0.32;
    if (state.x > 76) state.x = 58;
    state.heading = 82 + Math.sin(Date.now() / 800) * 4;
    state.latency = Math.max(17, Math.round(22 + Math.sin(Date.now() / 1050) * 8 + Math.random() * 3));
    state.loss = Math.max(0.04, Math.min(0.48, 0.12 + Math.sin(Date.now() / 1700) * 0.08 + Math.random() * 0.04));
    state.force = 16.8 + Math.sin(Date.now() / 850) * 2.1 + Math.random() * 1.6;
    state.distance = 1.82 + Math.sin(Date.now() / 1300) * 0.09;
    evaluateSafety();
    updateUI();
  } else {
    updateLatencyBars();
  }
  updateForceTrace();
}, 1000);

renderLogs();
renderHistory();
updateUI();
updateForceTrace();
