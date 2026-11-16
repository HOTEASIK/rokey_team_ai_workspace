// rokey1 HMI front-end · hmi_server 의 /api 와 SSE(/api/events) 로 동작한다.

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmtTime = (t) => new Date(t * 1000).toLocaleTimeString('en-GB', { hour12: false });
const MAX_LOGS = 600;
const BAD_STATES = new Set([3, 5, 6, 9, 10]);
const REACH = 900; // M0609 reach (mm)

// ----- 화면 표시용 한글 변환 (내부 코드는 영어 그대로) -----
const KIND_KO = { process: '노드', service: '상시 실행', action: '액션' };
const MODE_KO = { virtual: '가상', real: '실제 로봇' };
const GROUP_KO = { navigation: '벽면 탐색', putty: '퍼티', sanding: '샌딩' };
const RUN_KO = { starting: '시작 중', running: '실행 중', done: '완료', failed: '실패', stopped: '정지', killed: '강제 종료' };
const STATUS_KO = { IDLE: '대기', RUNNING: '실행 중', INPUT: '입력 대기', ERROR: '오류', FAULT: '고장', 'E-STOP': '비상 정지' };
const TYPE_KO = { INFO: '정보', WARN: '경고', ERROR: '오류' };
const LOGSTATE_KO = { OK: '정상', WATCH: '주의', FAULT: '오류', EXEC: '실행', INPUT: '입력', STOP: '정지', REJECT: '거부', STOPPED: '정지됨', ARMED: '대기', DONE: '완료', FAILED: '실패', KILLED: '강제종료', STARTIN: '시작', RUNNING: '실행' };
const ROBOT_KO = { INITIALIZING: '초기화 중', STANDBY: '대기', MOVING: '이동 중', SAFE_OFF: '서보 꺼짐', TEACHING: '티칭', SAFE_STOP: '안전 정지', EMERGENCY_STOP: '비상 정지', HOMING: '원점 복귀', RECOVERY: '복구 모드', SAFE_STOP2: '안전 정지2', SAFE_OFF2: '서보 꺼짐2', NOT_READY: '준비 안 됨', UNKNOWN: '알 수 없음' };
const robotKo = (name) => (name && ROBOT_KO[name] ? `${ROBOT_KO[name]} (${name})` : name || '알 수 없음');
const ko = (map, v) => map[v] ?? v;

const ui = {
  tasks: [],
  group: 'navigation',
  taskId: null,
  mode: 'virtual',
  values: {},          // task_id → {key: value}
  logs: [],
  filter: 'ALL',
  faults: 0,
  state: null,
  connected: false,
  latency: [],
  forceHist: [],
  trail: [],
  lastRunsKey: '',
};

// ------------------------------------------------------------------ API
async function api(path, body) {
  const opts = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const res = await fetch(path, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function toast(msg, kind = 'error') {
  const el = $('toast');
  el.textContent = msg;
  el.className = `toast ${kind}`;
  el.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { el.hidden = true; }, 4500);
}

// ------------------------------------------------------------------ 작업 선택 / 파라미터 폼
function tasksInGroup() { return ui.tasks.filter((t) => t.group === ui.group); }
function currentTask() { return ui.tasks.find((t) => t.id === ui.taskId); }

function renderTaskList() {
  document.querySelectorAll('.task-tab').forEach((b) => b.classList.toggle('active', b.dataset.group === ui.group));
  const list = tasksInGroup();
  if (!list.some((t) => t.id === ui.taskId)) ui.taskId = list[0]?.id ?? null;
  $('taskList').innerHTML = list.map((t) => `<button class="task-item ${t.id === ui.taskId ? 'active' : ''}" data-task="${t.id}"><b>${esc(t.label)}</b><small>${esc(ko(KIND_KO, t.kind))}</small></button>`).join('');
  $('taskList').querySelectorAll('.task-item').forEach((b) => b.addEventListener('click', () => { ui.taskId = b.dataset.task; saveUi(); renderTaskList(); }));
  renderParams();
}

function renderParams() {
  const task = currentTask();
  $('taskDesc').textContent = task ? task.desc : '';
  if (!task) { $('paramGrid').innerHTML = ''; return; }
  const vals = (ui.values[task.id] ||= {});
  $('paramGrid').innerHTML = task.params.map((p) => {
    const v = vals[p.key] ?? p.default;
    const id = `p_${task.id}_${p.key}`;
    let input;
    if (p.type === 'select') input = `<select id="${id}" data-key="${p.key}">${p.options.map((o) => `<option ${String(o) === String(v) ? 'selected' : ''}>${esc(o)}</option>`).join('')}</select>`;
    else if (p.type === 'bool') input = `<label class="switch"><input type="checkbox" id="${id}" data-key="${p.key}" ${v ? 'checked' : ''}/><span></span></label>`;
    else input = `<input id="${id}" data-key="${p.key}" type="${p.type === 'number' ? 'number' : 'text'}" step="any" value="${esc(v)}" />`;
    const wide = p.type === 'text' ? ' wide' : '';
    return `<label class="param-field${wide} ${p.type}" title="${esc(p.help || '')}"><span>${esc(p.label)}</span>${input}</label>`;
  }).join('') || '<div class="param-empty">설정할 파라미터 없음</div>';
  $('paramGrid').querySelectorAll('[data-key]').forEach((el) => el.addEventListener('change', () => {
    vals[el.dataset.key] = el.type === 'checkbox' ? el.checked : el.value;
    saveUi();
  }));
}

function collectParams() {
  const task = currentTask();
  const out = {};
  task.params.forEach((p) => {
    const el = $(`p_${task.id}_${p.key}`);
    if (!el) return;
    out[p.key] = p.type === 'bool' ? el.checked : p.type === 'number' ? Number(el.value) : el.value;
  });
  return out;
}

function setMode(mode) {
  ui.mode = mode;
  document.querySelectorAll('.mode-btn').forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $('realWarning').hidden = mode !== 'real';
  $('modeFoot').textContent = MODE_KO[mode];
  saveUi();
}

function saveUi() {
  try { localStorage.setItem('rokey1-hmi', JSON.stringify({ group: ui.group, taskId: ui.taskId, mode: ui.mode, values: ui.values })); } catch (e) { /* ignore */ }
}
function loadUi() {
  try { Object.assign(ui, JSON.parse(localStorage.getItem('rokey1-hmi')) || {}); } catch (e) { /* ignore */ }
}

// ------------------------------------------------------------------ 명령
async function startTask() {
  const task = currentTask();
  if (!task) return;
  if (ui.mode === 'real' && !confirm(`[실제 로봇] ${task.label} 을(를) 실제 로봇으로 실행합니다.\n주변이 안전한지 확인했습니까?`)) return;
  try {
    const { run } = await api('/api/start', { task_id: task.id, mode: ui.mode, params: collectParams() });
    $('lastCommand').textContent = `${task.label} / ${MODE_KO[ui.mode]}`;
    $('lastCommandSub').textContent = `실행 #${run.run_id} 시작`;
  } catch (e) { toast(e.message); }
}

async function stopRun(runId) {
  try { await api('/api/stop', runId == null ? {} : { run_id: runId }); } catch (e) { toast(e.message, 'warn'); }
}

async function toggleEstop() {
  const active = ui.state?.estop;
  try {
    if (active) {
      if (!confirm('비상 정지를 해제합니다. 로봇 주변과 상태를 확인했습니까?')) return;
      await api('/api/estop/reset', {});
    } else {
      await api('/api/estop', {});
    }
  } catch (e) { toast(e.message); }
}

async function sendStdin(text) {
  const runId = Number($('stdinRun').value);
  if (!runId) { toast('입력을 보낼 실행 중인 작업이 없습니다.', 'warn'); return; }
  try { await api('/api/stdin', { run_id: runId, text }); $('stdinText').value = ''; } catch (e) { toast(e.message); }
}

// ------------------------------------------------------------------ 로그
function addLogs(entries) {
  entries.forEach((l) => { if (l.type === 'ERROR') ui.faults += 1; });
  ui.logs.push(...entries);
  if (ui.logs.length > MAX_LOGS) ui.logs.splice(0, ui.logs.length - MAX_LOGS);
  renderLogs();
}

function renderLogs() {
  const visible = ui.filter === 'ALL' ? ui.logs : ui.logs.filter((l) => l.type === ui.filter);
  const box = $('consoleLog');
  box.innerHTML = visible.slice(-300).map((l) => `<div class="log-row${l.state === 'INPUT' ? ' prompt' : ''}"><time>${fmtTime(l.t)}</time><span class="log-type ${l.type.toLowerCase()}">${ko(TYPE_KO, l.type)}</span><span class="log-source" title="${esc(l.source)}">${esc(l.source)}</span><span class="log-message" title="${esc(l.message)}">${esc(l.message)}</span><span class="log-state ${l.type.toLowerCase()}">${esc(ko(LOGSTATE_KO, l.state))}</span></div>`).join('');
  if ($('autoScroll').checked) box.scrollTop = box.scrollHeight;
  $('logCount').textContent = String(visible.length).padStart(2, '0');
  $('visibleCount').textContent = Math.min(visible.length, 300);
  $('totalCount').textContent = ui.logs.length;
  $('faultValue').textContent = `${ui.faults}건`;
  $('faultValue').className = ui.faults ? 'error-text' : '';
}

function setFilter(filter) {
  ui.filter = filter;
  document.querySelectorAll('[data-filter]').forEach((b) => b.classList.toggle('active', b.dataset.filter === filter));
  renderLogs();
}

// ------------------------------------------------------------------ 상태 렌더링
function renderState(s) {
  ui.state = s;
  const runs = s.runs || [];
  const active = runs.filter((r) => r.state === 'starting' || r.state === 'running');
  const main = active.find((r) => r.motion) || active[active.length - 1] || runs[runs.length - 1];
  const waiting = active.find((r) => r.awaiting_input);
  const robot = s.robot || {};
  const links = s.links || {};
  const badRobot = BAD_STATES.has(robot.state);

  // --- 상태 판정
  let status = 'IDLE';
  if (s.estop) status = 'E-STOP';
  else if (badRobot) status = 'FAULT';
  else if (waiting) status = 'INPUT';
  else if (active.length) status = 'RUNNING';
  else if (main && main.state === 'failed') status = 'ERROR';
  $('statusText').textContent = STATUS_KO[status];
  $('stageState').textContent = STATUS_KO[status];
  const alert = ['E-STOP', 'FAULT', 'ERROR'].includes(status);
  const warn = status === 'INPUT';
  const color = alert ? 'var(--red)' : warn ? 'var(--amber)' : active.length ? 'var(--green)' : 'var(--muted)';
  const stamp = $('statusStamp');
  stamp.classList.toggle('alert', alert);
  stamp.style.color = color;
  stamp.style.borderColor = color;
  stamp.querySelector('i').style.background = color;

  // --- 현재 작업 / 진행률
  if (main) {
    $('taskName').textContent = main.label;
    $('taskSub').textContent = `${ko(GROUP_KO, main.group)} · ${ko(MODE_KO, main.mode)} · 실행 #${main.run_id} · ${ko(RUN_KO, main.state)}`;
    $('elapsedValue').textContent = main.elapsed.toFixed(1);
    $('runIdValue').textContent = `#${main.run_id}`;
    $('stageValue').textContent = main.stage || ko(RUN_KO, main.state);
    if (main.command) { $('lastCommandSub').textContent = main.command; $('lastCommandSub').title = main.command; }
  } else {
    $('taskName').textContent = '작업 없음';
    $('taskSub').textContent = '대기 중';
    $('elapsedValue').textContent = '0.0';
    $('runIdValue').textContent = '—';
    $('stageValue').textContent = '—';
  }
  const track = $('progressTrack');
  let pct = null;
  if (main && main.total > 0) pct = Math.min(100, Math.round((main.step / main.total) * 100));
  else if (main && main.state === 'done') pct = 100;
  const running = main && (main.state === 'running' || main.state === 'starting');
  track.classList.toggle('indeterminate', Boolean(running && pct === null));
  $('progressLabel').textContent = main && main.total > 0 ? `단계 ${main.step}/${main.total}` : '작업 진행률';
  $('progressValue').textContent = pct === null ? (running ? '실행 중' : '--') : `${pct}%`;
  $('progressBar').style.width = `${pct ?? 0}%`;
  $('progressBar').nextElementSibling.style.left = `${pct ?? 0}%`;

  // --- 로봇 상태 / 링크
  $('robotStateValue').innerHTML = `<span></span>${esc(robotKo(robot.state_name))}`;
  $('robotStateValue').classList.toggle('bad', badRobot);
  document.querySelectorAll('.device-row[data-link]').forEach((row) => {
    const on = Boolean(links[row.dataset.link]);
    row.classList.toggle('offline', !on);
    row.querySelector('small').textContent = on ? '연결됨' : '끊김';
  });
  setDot('dotDsr', links.controller ? 'green' : 'red');
  $('commDsr').textContent = links.controller ? '정상' : links.dsr_msgs ? '서비스 없음' : 'dsr_msgs2 없음';
  setDot('dotJoint', robot.joint_hz > 1 ? 'green' : 'amber');
  $('commJoint').textContent = `${(robot.joint_hz || 0).toFixed(1)} Hz`;
  $('jointHzFoot').textContent = `${(robot.joint_hz || 0).toFixed(0)} Hz`;
  setDot('dotTelemetry', robot.stale ? 'amber' : 'green');
  $('commTelemetry').textContent = robot.stale ? '수신 없음' : '수신 중';
  $('robotIdChip').textContent = `${s.robot_id} · m0609`;
  $('activeCount').textContent = active.length;

  // --- 힘
  const f = robot.force;
  if (f) {
    const mag = Math.hypot(f[0], f[1], f[2]);
    $('forceValue').textContent = mag.toFixed(1);
    $('forceMetric').textContent = mag.toFixed(1);
    $('forcePercent').textContent = `${Math.min(999, Math.round(mag))}`;
    $('forceRing').style.transform = `rotate(${Math.min(280, mag * 4)}deg)`;
    ['fx', 'fy', 'fz'].forEach((k, i) => { $(k).textContent = `${f[i].toFixed(1)} N`; });
    ui.forceHist.push(mag);
    if (ui.forceHist.length > 60) ui.forceHist.shift();
    drawForce();
  }

  // --- 위치
  const p = robot.posx;
  if (p) {
    ['posX', 'posY', 'posZ', 'rotA', 'rotB', 'rotC'].forEach((k, i) => { $(k).textContent = p[i].toFixed(1); });
    const last = ui.trail[ui.trail.length - 1];
    if (!last || Math.hypot(last[0] - p[0], last[1] - p[1], last[2] - p[2]) > 1) {
      ui.trail.push(p.slice(0, 3));
      if (ui.trail.length > 400) ui.trail.shift();
    }
  }
  drawViews(p);
  renderJoints(robot.joints);

  // --- 안전
  const banner = $('safetyBanner');
  banner.className = `safety-banner ${s.estop || badRobot ? 'alert' : 'safe'}`;
  banner.querySelector('.safety-symbol').textContent = s.estop || badRobot ? '!' : '✓';
  $('safetyTitle').textContent = s.estop ? '비상 정지 작동 중' : badRobot ? `컨트롤러 ${robotKo(robot.state_name)}` : '모든 시스템 정상';
  $('safetyDetail').textContent = s.estop ? '작업 시작 금지 · 확인 후 비상 정지 해제' : badRobot ? '티칭 펜던트에서 상태 확인/해제 필요' : '비상 정지 대기 중';
  $('safetyEstop').textContent = s.estop ? '작동 / 정지됨' : '대기 / 정상';
  $('safetyEstop').className = s.estop ? 'error-text' : 'safe-text';
  $('safetyRobot').textContent = robotKo(robot.state_name);
  $('safetyRobot').className = badRobot ? 'error-text' : robot.state == null ? '' : 'safe-text';
  $('safetyFoot').textContent = s.estop ? '비상 정지' : badRobot ? '고장' : '정상';
  $('estopBtn').classList.toggle('triggered', s.estop);
  $('estopLabel').textContent = s.estop ? '비상 정지 해제' : '비상 정지';
  $('armedTag').textContent = s.estop ? '잠김' : '준비';
  $('startBtn').disabled = s.estop;

  renderRuns(runs, active, waiting);
}

function setDot(id, color) { $(id).className = `link-dot ${color}`; }

function renderRuns(runs, active, waiting) {
  const key = JSON.stringify(runs.map((r) => [r.run_id, r.state, r.awaiting_input, Math.floor(r.elapsed)]));
  if (key === ui.lastRunsKey) return;
  ui.lastRunsKey = key;
  $('runList').innerHTML = runs.slice().reverse().slice(0, 8).map((r) => {
    const live = r.state === 'running' || r.state === 'starting';
    const cls = live ? 'live' : r.state === 'done' ? 'ok' : r.state === 'stopped' ? 'muted' : 'bad';
    return `<div class="run-item ${cls}"><b>#${r.run_id}</b><span title="${esc(r.command)}">${esc(r.label)}</span><em>${ko(RUN_KO, r.state)}${r.returncode != null && r.state !== 'done' ? ` (${r.returncode})` : ''}</em><time>${r.elapsed.toFixed(0)}초</time>${live ? `<button data-stop="${r.run_id}" title="이 작업 정지">■</button>` : ''}</div>`;
  }).join('') || '<div class="param-empty">실행 기록 없음</div>';
  $('runList').querySelectorAll('[data-stop]').forEach((b) => b.addEventListener('click', () => stopRun(Number(b.dataset.stop))));

  // stdin 대상
  const sel = $('stdinRun');
  const prev = sel.value;
  const procs = active.filter((r) => r.kind !== 'action');
  sel.innerHTML = procs.map((r) => `<option value="${r.run_id}">#${r.run_id} ${esc(r.label)}${r.awaiting_input ? ' ⏎' : ''}</option>`).join('') || '<option value="">(실행 중 없음)</option>';
  sel.value = waiting && waiting.kind !== 'action' ? String(waiting.run_id) : procs.some((r) => String(r.run_id) === prev) ? prev : sel.options[0]?.value ?? '';
  $('stdinForm').classList.toggle('waiting', Boolean(waiting));
  $('stdinLabel').textContent = waiting ? '입력 대기 ⏎' : '노드 입력';
  if (waiting && (document.activeElement === document.body || !document.activeElement)) $('stdinText').focus({ preventScroll: true });
}

function renderJoints(j) {
  $('jointList').innerHTML = [0, 1, 2, 3, 4, 5].map((i) => {
    const v = j ? j[i] : null;
    const pct = v == null ? 50 : Math.max(0, Math.min(100, 50 + v / 3.6));
    return `<div class="joint-row"><span>J${i + 1}</span><div class="joint-bar"><i style="left:${pct}%"></i></div><b>${v == null ? '--' : v.toFixed(1)}</b></div>`;
  }).join('');
}

// ------------------------------------------------------------------ 그래프
function drawForce() {
  const h = ui.forceHist;
  const max = Math.max(20, ...h);
  const pts = h.map((v, i) => `${(i / 59) * 280},${44 - (v / max) * 40}`);
  $('forceTrace').setAttribute('points', pts.join(' '));
}

function drawViews(p) {
  const grid = (minX, maxX, minY, maxY) => {
    let g = '';
    for (let x = Math.ceil(minX / 200) * 200; x <= maxX; x += 200) g += `<line x1="${x}" y1="${minY}" x2="${x}" y2="${maxY}" class="g${x === 0 ? ' axis' : ''}"/>`;
    for (let y = Math.ceil(minY / 200) * 200; y <= maxY; y += 200) g += `<line x1="${minX}" y1="${y}" x2="${maxX}" y2="${y}" class="g${y === 0 ? ' axis' : ''}"/>`;
    return g;
  };
  // TOP: SVG x = X, SVG y = -Y
  const topTrail = ui.trail.map((q) => `${q[0]},${-q[1]}`).join(' ');
  let top = grid(-1000, 1000, -1000, 1000);
  top += `<circle cx="0" cy="0" r="${REACH}" class="reach"/><circle cx="0" cy="0" r="60" class="base"/><text x="70" y="-70" class="lbl">BASE</text>`;
  top += `<polyline points="${topTrail}" class="trail"/>`;
  if (p) top += `<line x1="0" y1="0" x2="${p[0]}" y2="${-p[1]}" class="arm"/><circle cx="${p[0]}" cy="${-p[1]}" r="28" class="tcp"/><text x="${p[0] + 40}" y="${-p[1] - 40}" class="lbl tcp-l">TCP</text>`;
  else top += '<text x="0" y="0" class="lbl nodata" text-anchor="middle">위치 데이터 없음</text>';
  top += '<text x="940" y="-20" class="lbl">+X</text><text x="20" y="-940" class="lbl">+Y</text>';
  $('topView').innerHTML = top;

  // SIDE: SVG x = X, SVG y = -Z
  const sideTrail = ui.trail.map((q) => `${q[0]},${-q[2]}`).join(' ');
  let side = grid(-1000, 1000, -1300, 300);
  side += `<rect x="-90" y="-0" width="180" height="40" class="base"/><line x1="-1000" y1="0" x2="1000" y2="0" class="floor"/>`;
  side += `<polyline points="${sideTrail}" class="trail"/>`;
  if (p) side += `<line x1="0" y1="-150" x2="${p[0]}" y2="${-p[2]}" class="arm"/><circle cx="${p[0]}" cy="${-p[2]}" r="28" class="tcp"/>`;
  side += '<text x="940" y="-20" class="lbl">+X</text><text x="20" y="-1240" class="lbl">+Z</text>';
  $('sideView').innerHTML = side;
}

function updateLatencyBars() {
  const box = $('latencyBars');
  if (!box.children.length) for (let i = 0; i < 25; i += 1) box.appendChild(document.createElement('i'));
  const vals = ui.latency.slice(-25);
  [...box.children].forEach((bar, i) => {
    const v = vals[i - (25 - vals.length)];
    bar.style.height = v == null ? '4%' : `${Math.max(6, Math.min(100, v * 2))}%`;
  });
}

// ------------------------------------------------------------------ 연결
function setConnected(on) {
  ui.connected = on;
  $('bridgeText').textContent = on ? 'HMI 서버 연결됨' : '연결 끊김';
  $('bridgeChip').classList.toggle('down', !on);
  $('railStatus').textContent = on ? 'HMI 서버 · 연결됨' : 'HMI 서버 · 연결 끊김';
  setDot('dotBridge', on ? 'green' : 'red');
  $('commBridge').textContent = on ? '연결됨' : '재연결 중';
  $('streamText').textContent = on ? '실시간 연결됨' : '재연결 중…';
  $('streamDot').className = on ? 'green' : '';
}

function connect() {
  const es = new EventSource('/api/events');
  es.addEventListener('open', () => setConnected(true));
  es.addEventListener('error', () => setConnected(false));
  es.addEventListener('backlog', (e) => { ui.logs = []; ui.faults = 0; addLogs(JSON.parse(e.data)); });
  es.addEventListener('log', (e) => addLogs([JSON.parse(e.data)]));
  es.addEventListener('state', (e) => renderState(JSON.parse(e.data)));
  es.addEventListener('runs', (e) => { if (ui.state) renderState({ ...ui.state, runs: JSON.parse(e.data) }); });
}

async function measureLatency() {
  const t0 = performance.now();
  try {
    await fetch('/api/state', { cache: 'no-store' });
    const ms = Math.round(performance.now() - t0);
    ui.latency.push(ms);
    if (ui.latency.length > 25) ui.latency.shift();
    $('latencyValue').textContent = ms;
  } catch (e) {
    ui.latency.push(null);
    $('latencyValue').textContent = '--';
  }
  updateLatencyBars();
}

// ------------------------------------------------------------------ 이벤트 바인딩
function bind() {
  document.querySelectorAll('.task-tab').forEach((b) => b.addEventListener('click', () => { ui.group = b.dataset.group; ui.taskId = null; saveUi(); renderTaskList(); }));
  document.querySelectorAll('.mode-btn').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
  $('startBtn').addEventListener('click', startTask);
  $('stopBtn').addEventListener('click', () => stopRun(null));
  $('estopBtn').addEventListener('click', toggleEstop);
  document.querySelectorAll('[data-filter]').forEach((b) => b.addEventListener('click', () => setFilter(b.dataset.filter)));
  $('clearBtn').addEventListener('click', () => { ui.logs = []; ui.faults = 0; ui.trail = []; renderLogs(); });
  $('focusBtn').addEventListener('click', () => $('simulationCard').classList.toggle('focused'));
  $('stdinForm').addEventListener('submit', (e) => { e.preventDefault(); sendStdin($('stdinText').value); });
  document.querySelectorAll('[data-quick]').forEach((b) => b.addEventListener('click', () => sendStdin(b.dataset.quick)));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && !ui.state?.estop) { e.preventDefault(); api('/api/estop', {}).catch((err) => toast(err.message)); }
  });
  $('footerHost').textContent = location.host;
}

async function init() {
  loadUi();
  bind();
  setMode(ui.mode === 'real' ? 'real' : 'virtual');
  setConnected(false);
  renderJoints(null);
  drawViews(null);
  updateLatencyBars();
  setInterval(() => {
    const d = new Date();
    $('topClock').textContent = d.toLocaleTimeString('en-GB', { hour12: false });
    $('topDate').textContent = d.toLocaleDateString('ko-KR', { year: 'numeric', month: 'long', day: 'numeric', weekday: 'short' });
  }, 1000);
  setInterval(measureLatency, 2000);
  for (;;) {
    try {
      const data = await api('/api/tasks');
      ui.tasks = data.tasks;
      break;
    } catch (e) {
      $('taskDesc').textContent = 'hmi_server 에 연결할 수 없습니다. 재시도 중…';
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
  if (!['navigation', 'putty', 'sanding'].includes(ui.group)) ui.group = 'navigation';
  renderTaskList();
  connect();
}

init();
