// rokey1 HMI front-end · hmi_server 의 /api 와 SSE(/api/events) 로 동작한다.

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmtTime = (t) => new Date(t * 1000).toLocaleTimeString('en-GB', { hour12: false });
const MAX_LOGS = 600;
const BAD_STATES = new Set([3, 5, 6, 9, 10]);
const REACH = 900; // M0609 reach (mm)

// ----- 화면 표시용 한글 변환 (내부 코드는 영어 그대로) -----
const KIND_KO = { process: '노드', service: '상시 실행', action: '액션', db: 'DB 기록' };
const MODE_KO = { virtual: '가상', real: '실제 로봇' };
const GROUP_KO = { navigation: '벽면 탐색', putty: '퍼티', sanding: '샌딩', process: '공정' };
const RUN_KO = { starting: '시작 중', running: '실행 중', done: '완료', failed: '실패', stopped: '정지', killed: '강제 종료' };
const STATUS_KO = { IDLE: '대기', RUNNING: '실행 중', INPUT: '입력 대기', ERROR: '오류', FAULT: '고장', 'E-STOP': '비상 정지' };
const TYPE_KO = { INFO: '정보', WARN: '경고', ERROR: '오류' };
const LOGSTATE_KO = { OK: '정상', WATCH: '주의', FAULT: '오류', EXEC: '실행', INPUT: '입력', STOP: '정지', REJECT: '거부', STOPPED: '정지됨', ARMED: '대기', DONE: '완료', FAILED: '실패', KILLED: '강제종료', STARTIN: '시작', RUNNING: '실행' };
const ROBOT_KO = { INITIALIZING: '초기화 중', STANDBY: '대기', MOVING: '이동 중', SAFE_OFF: '서보 꺼짐', TEACHING: '티칭', SAFE_STOP: '안전 정지', EMERGENCY_STOP: '비상 정지', HOMING: '원점 복귀', RECOVERY: '복구 모드', SAFE_STOP2: '안전 정지2', SAFE_OFF2: '서보 꺼짐2', NOT_READY: '준비 안 됨', UNKNOWN: '알 수 없음' };
const robotKo = (name) => (name && ROBOT_KO[name] ? `${ROBOT_KO[name]} (${name})` : name || '알 수 없음');
const ko = (map, v) => map[v] ?? v;

const ui = {
  tasks: [],
  group: 'process',
  taskId: null,
  mode: 'virtual',
  values: {},          // task_id → {key: value}
  temp: {},            // remember=false 항목 (저장 안 함, 새로고침하면 비움)
  logs: [],
  blogs: [],            // 로봇 bringup 출력 (따로 보여 줌)
  channel: 'main',
  filter: 'ALL',
  faults: 0,
  state: null,
  connected: false,
  latency: [],
  forceHist: [],
  trail: [],
  lastRunsKey: '',
  layout: 'stack',     // stack(위·아래) | split(좌·우)
  view: 'coords',      // TCP 카드: coords(자세 그림 + 6좌표) | path(위에서 본 경로)
  page: 'control',     // control(작업 제어판) | monitor(실시간 작업 현황)
  speed: { pct: 100, unlocked: false, max: 100, min: 10, default: 100 },
  speedDragging: false,
  alertOn: true,
};
const MAX_ALERTS = 4;

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

const fmtDefault = (p) => (p.type === 'bool' ? (p.default ? '켬' : '끔') : p.default === '' ? (p.type === 'number' ? '없음 · 직접 입력' : '(비움 = 자동)') : String(p.default));
const storeOf = (task, p) => (p.remember === false ? (ui.temp[task.id] ||= {}) : (ui.values[task.id] ||= {}));
const rangeText = (p) => [p.min != null ? `${p.min} 이상` : '', p.max != null ? `${p.max} 이하` : '', p.integer ? '정수' : '', p.maxlen ? `${p.maxlen}자 이하` : ''].filter(Boolean).join(' · ');
const isDefault = (p, v) => (p.type === 'bool' ? Boolean(v) === Boolean(p.default) : p.type === 'number' ? Number(v) === Number(p.default) : String(v) === String(p.default));

function renderParams() {
  const task = currentTask();
  $('taskDesc').textContent = task ? task.desc : '';
  $('resetParamsBtn').hidden = !task || !task.params.length;
  if (!task) { $('paramGrid').innerHTML = ''; $('paramChanged').textContent = ''; return; }
  $('paramGrid').innerHTML = task.params.map((p) => {
    const v = storeOf(task, p)[p.key] ?? p.default;
    const id = `p_${task.id}_${p.key}`;
    let input;
    if (p.type === 'select') input = `<select id="${id}" data-key="${p.key}">${p.options.map((o) => `<option ${String(o) === String(v) ? 'selected' : ''}>${esc(o)}</option>`).join('')}</select>`;
    else if (p.type === 'bool') input = `<label class="switch"><input type="checkbox" id="${id}" data-key="${p.key}" ${v ? 'checked' : ''}/><span></span></label>`;
    else if (p.type === 'number') input = `<input id="${id}" data-key="${p.key}" type="number" step="${p.integer ? 1 : 'any'}"${p.min != null ? ` min="${p.min}"` : ''}${p.max != null ? ` max="${p.max}"` : ''} required value="${esc(v)}" placeholder="${p.default === '' ? '입력 필요' : esc(p.default)}" />`;
    else input = `<input id="${id}" data-key="${p.key}" type="text"${p.maxlen ? ` maxlength="${p.maxlen}"` : ''} value="${esc(v)}" placeholder="${esc(p.default)}" />`;
    const wide = p.type === 'text' ? ' wide' : '';
    const changed = isDefault(p, v) ? '' : ' changed';
    const range = rangeText(p);
    const help = `${p.help ? `${esc(p.help)} · ` : ''}<em>기본 ${esc(fmtDefault(p))}</em>${range ? ` · ${esc(range)}` : ''}${p.scale ? ' · <em class="scale-tag">속도% 적용</em>' : ''}`;
    const head = `<span class="pf-label">${esc(p.label)}</span>`;
    const body = p.type === 'bool' ? `<div class="pf-row">${head}${input}</div>` : `${head}${input}`;
    return `<label class="param-field${wide} ${p.type}${changed}" data-field="${p.key}">${body}<small class="pf-help">${help}</small></label>`;
  }).join('') || '<div class="param-empty">설정할 파라미터 없음 · 바로 시작할 수 있습니다</div>';
  const update = () => {
    let n = 0;
    task.params.forEach((p) => {
      const d = isDefault(p, storeOf(task, p)[p.key] ?? p.default);
      if (!d) n += 1;
      $('paramGrid').querySelector(`[data-field="${p.key}"]`)?.classList.toggle('changed', !d);
    });
    $('paramChanged').textContent = n ? `기본값에서 ${n}개 변경됨` : '모두 기본값';
    $('paramChanged').classList.toggle('changed', n > 0);
  };
  $('paramGrid').querySelectorAll('[data-key]').forEach((el) => el.addEventListener('change', () => {
    const p = task.params.find((x) => x.key === el.dataset.key);
    storeOf(task, p)[el.dataset.key] = el.type === 'checkbox' ? el.checked : el.value;
    saveUi();
    update();
  }));
  update();
}

function resetParams() {
  const task = currentTask();
  if (!task) return;
  ui.values[task.id] = {};
  ui.temp[task.id] = {};
  saveUi();
  renderParams();
  toast(`${task.label} · 입력값을 기본값으로 되돌렸습니다.`, 'info');
}

function collectParams() {
  const task = currentTask();
  const out = {};
  task.params.forEach((p) => {
    const el = $(`p_${task.id}_${p.key}`);
    if (!el) return;
    // 빈 숫자 칸은 0 으로 바꾸지 않고 '' 로 보냄 → 서버가 '값을 입력하세요' 로 거부
    out[p.key] = p.type === 'bool' ? el.checked : p.type === 'number' ? (el.value.trim() === '' ? '' : Number(el.value)) : el.value;
  });
  return out;
}

function setMode(mode) {
  ui.mode = mode;
  document.querySelectorAll('.mode-btn').forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $('modeFoot').textContent = MODE_KO[mode];
  saveUi();
}

function saveUi() {
  try { localStorage.setItem('rokey1-hmi', JSON.stringify({ group: ui.group, taskId: ui.taskId, mode: ui.mode, values: ui.values, layout: ui.layout, view: ui.view, alertOn: ui.alertOn, page: ui.page, uiMode: ui.uiMode })); } catch (e) { /* ignore */ }
}
function loadUi() {
  try { Object.assign(ui, JSON.parse(localStorage.getItem('rokey1-hmi')) || {}); } catch (e) { /* ignore */ }
}

// ------------------------------------------------------------------ 명령
// 화면 정중앙 예/아니요 팝업 → 예 = true, 아니요/Esc = false
function ask(msg) {
  const dlg = $('askDlg');
  $('askMsg').textContent = msg;
  dlg.returnValue = '';
  dlg.showModal();
  return new Promise((done) => dlg.addEventListener('close', () => done(dlg.returnValue === 'yes'), { once: true }));
}

async function startTask() {
  const task = currentTask();
  if (!task) return;
  if (ui.mode === 'real' && !(await ask(`[실제 로봇] ${task.label} 을(를) 실제 로봇으로 실행합니다.\n주변이 안전한지 확인했습니까?`))) return;
  try {
    const { run } = await api('/api/start', { task_id: task.id, mode: ui.mode, params: collectParams() });
    $('lastCommand').textContent = `${task.label} / ${MODE_KO[ui.mode]}`;
    $('lastCommandSub').textContent = `실행 #${run.run_id} 시작`;
  } catch (e) { toast(e.message); }
}

async function goHome() {
  if (ui.state?.estop) { toast('비상 정지 상태입니다. 해제 후 홈 이동하세요.'); return; }
  const where = ui.mode === 'real' ? '[실제 로봇]' : '[가상]';
  if (!confirm(`${where} 관절 홈 자세 (0, 0, 90, 0, 90, 0) 로 이동합니다.\n속도 ${ui.speed.pct}% · 주변이 안전한지 확인했습니까?`)) return;
  try {
    const { run } = await api('/api/start', { task_id: 'homeset', mode: ui.mode, params: {} });
    $('lastCommand').textContent = `홈 위치로 이동 / ${MODE_KO[ui.mode]}`;
    $('lastCommandSub').textContent = `실행 #${run.run_id} 시작`;
  } catch (e) { toast(e.message); }
}

async function gripper(action) {
  if (ui.state?.estop) { toast('비상 정지 상태입니다. 해제 후 사용하세요.'); return; }
  const where = ui.mode === 'real' ? '[실제 로봇]' : '[가상]';
  const ko = action === 'open' ? '열기' : '닫기';
  const warn = action === 'open' ? '\n쥐고 있는 도구·물건이 떨어집니다. 받칠 준비가 됐습니까?' : '\n손가락 사이에 손이 없는지 확인했습니까?';
  if (!confirm(`${where} 그리퍼 ${ko}${warn}`)) return;
  try {
    await api('/api/gripper', { action, mode: ui.mode });
    $('lastCommand').textContent = `그리퍼 ${ko} / ${MODE_KO[ui.mode]}`;
    $('lastCommandSub').textContent = '완료';
  } catch (e) { toast(e.message); }
}

async function bringup(mode) {
  if (mode === 'real' && !confirm('[실제 로봇] 실제 로봇 bringup 을 시작합니다.\n주변이 안전한지 확인했습니까?')) return;
  try {
    await api('/api/bringup', { mode });
    setMode(mode);
    $('lastCommand').textContent = `bringup / ${MODE_KO[mode]}`;
    $('lastCommandSub').textContent = '액션 서버는 공정 시작 때 필요한 것만 자동으로 켜집니다';
  } catch (e) { toast(e.message); }
}

async function stopBringup() {
  const run = (ui.state?.runs || []).find((r) => r.task_id === 'bringup' && ['starting', 'running'].includes(r.state));
  if (!run) { toast('HMI 에서 띄운 bringup 이 없습니다.', 'warn'); return; }
  if (!confirm('bringup 을 끕니다. 로봇 연결이 끊깁니다.')) return;
  stopRun(run.run_id);
}

// ------------------------------------------------------------------ 속도
function renderSpeed(sp) {
  if (!sp) return;
  ui.speed = sp;
  const range = $('speedRange');
  range.max = String(sp.max);
  range.min = String(sp.min);
  if (!ui.speedDragging) range.value = String(sp.pct);
  const shown = ui.speedDragging ? Number(range.value) : sp.pct;
  $('speedValue').textContent = shown;
  $('monSpeed').textContent = sp.pct;
  $('monSpeed').parentElement.classList.toggle('error-text', sp.pct > 100);
  $('speedMax').textContent = `${sp.max}%`;
  $('speedMid').textContent = `${Math.round((sp.min + sp.max) / 2)}%`;
  const over = shown > 100;
  $('speedBlock').classList.toggle('unlocked', sp.unlocked);
  $('speedBlock').classList.toggle('over', over);
  $('speedWarning').hidden = !sp.unlocked;
  $('speedUnlockBtn').textContent = sp.unlocked ? '🔓 안전 잠금' : '🔒 안전 해제';
  $('speedUnlockBtn').classList.toggle('active', sp.unlocked);
  const pct = ((shown - sp.min) / (sp.max - sp.min)) * 100;
  range.style.setProperty('--fill', `${pct}%`);
}

async function setSpeed(pct, unlocked = ui.speed.unlocked) {
  try {
    const { speed } = await api('/api/speed', { pct, unlocked });
    renderSpeed(speed);
  } catch (e) { toast(e.message); renderSpeed(ui.speed); }
}

async function toggleSpeedLock() {
  if (ui.speed.unlocked) { await setSpeed(Math.min(ui.speed.pct, 100), false); return; }
  if (ui.state?.estop) { toast('비상 정지 상태에서는 안전 해제를 할 수 없습니다.'); return; }
  if (!confirm('속도 안전 해제\n\n100% 를 넘는 속도(최대 200%)를 허용합니다.\n충돌 시 피해가 커질 수 있습니다. 주변에 사람이 없고 경로가 검증되었습니까?')) return;
  await setSpeed(ui.speed.pct, true);
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

// ------------------------------------------------------------------ 경고/오류 팝업 (로그 원문 그대로)
function showAlert(l) {
  if (!ui.alertOn) return;
  const list = $('alertList');
  const raw = l.raw || l.message;
  const key = `${l.type}|${l.source}|${raw}`;
  const same = [...list.children].find((c) => c.dataset.key === key);
  if (same) {
    const n = Number(same.dataset.count) + 1;
    same.dataset.count = n;
    same.querySelector('.alert-count').textContent = `×${n}`;
    same.querySelector('time').textContent = fmtTime(l.t);
    return;
  }
  const el = document.createElement('div');
  const kind = l.type === 'ERROR' ? 'error' : 'warn';
  const close = () => { el.remove(); renderAlertBar(); };
  el.className = `alert-pop ${kind}`;
  el.dataset.key = key;
  el.dataset.count = 1;
  el.innerHTML = `<div class="alert-head"><span class="alert-icon">${kind === 'error' ? '✕' : '!'}</span><b>${esc(ko(TYPE_KO, l.type))}</b><span class="alert-src">${esc(l.source)}${l.run_id ? ` · 실행 #${l.run_id}` : ''}</span><span class="alert-count"></span><time>${fmtTime(l.t)}</time><button class="alert-close" title="닫기">✕</button></div><pre class="alert-raw">${esc(raw)}</pre><div class="alert-foot"><span>${kind === 'warn' ? '12초 뒤 자동으로 닫힘' : '확인 후 직접 닫아 주세요'}</span><button class="alert-copy">복사</button><button class="alert-log">로그에서 보기</button></div>`;
  el.querySelector('.alert-close').addEventListener('click', close);
  el.querySelector('.alert-copy').addEventListener('click', () => { copyText(raw).then(() => toast('로그를 복사했습니다.', 'info')).catch(() => {}); });
  el.querySelector('.alert-log').addEventListener('click', () => { setFilter(l.type); close(); setPage('monitor'); $('consoleLog').scrollIntoView({ behavior: 'smooth', block: 'center' }); });
  list.prepend(el);
  while (list.children.length > MAX_ALERTS) list.lastElementChild.remove();
  // 경고는 12초 뒤 자동으로 닫힘, 오류는 직접 닫을 때까지 유지
  if (kind === 'warn') setTimeout(close, 12000);
  renderAlertBar();
}

function renderAlertBar() {
  $('alertClear').hidden = $('alertList').children.length < 2;
}

function clearAlerts() {
  $('alertList').innerHTML = '';
  renderAlertBar();
}

// ------------------------------------------------------------------ 페이지 (작업 제어판 / 실시간 작업 현황)
const PAGES = ['control', 'monitor'];
function setPage(page, push = true) {
  ui.page = PAGES.includes(page) && ui.uiMode !== 'simple' ? page : 'control';
  $('workspace').dataset.page = ui.page;
  document.querySelectorAll('.page-tab').forEach((b) => b.classList.toggle('active', b.dataset.page === ui.page));
  if (push && location.hash !== `#${ui.page}`) history.replaceState(null, '', `#${ui.page}`);
  if (ui.page === 'monitor') { const box = $('consoleLog'); if ($('autoScroll').checked) box.scrollTop = box.scrollHeight; }
  window.scrollTo(0, 0);
  saveUi();
}

// ------------------------------------------------------------------ 간편 / 전문가 모드
function setUiMode(mode) {
  ui.uiMode = mode === 'expert' ? 'expert' : 'simple';
  document.body.classList.toggle('simple', ui.uiMode === 'simple');
  document.querySelectorAll('[data-uimode]').forEach((b) => b.classList.toggle('active', b.dataset.uimode === ui.uiMode));
  if (ui.uiMode === 'simple') setPage('control'); else saveUi();
}

// ------------------------------------------------------------------ 화면 배치
function setLayout(layout) {
  ui.layout = layout === 'split' ? 'split' : 'stack';
  $('operationsStage').classList.toggle('layout-split', ui.layout === 'split');
  document.querySelectorAll('.layout-btn').forEach((b) => b.classList.toggle('active', b.dataset.layout === ui.layout));
  saveUi();
}

// ------------------------------------------------------------------ 로그
function addLogs(entries, live = false) {
  if (live) entries.forEach((l) => { if (l.popup) showAlert(l); });
  entries.forEach((l) => {
    if (l.channel === 'bringup') { ui.blogs.push(l); if (l.type === 'ERROR' && ui.channel !== 'bringup') ui.bringupErr = true; return; }
    if (l.type === 'ERROR') ui.faults += 1;
    ui.logs.push(l);
  });
  [ui.logs, ui.blogs].forEach((a) => { if (a.length > MAX_LOGS) a.splice(0, a.length - MAX_LOGS); });
  renderLogs();
}

function renderLogs() {
  const logs = ui.channel === 'bringup' ? ui.blogs : ui.logs;
  const visible = ui.filter === 'ALL' ? logs : logs.filter((l) => l.type === ui.filter);
  const box = $('consoleLog');
  box.innerHTML = visible.slice(-300).map((l) => `<div class="log-row${l.state === 'INPUT' ? ' prompt' : ''}"><time>${fmtTime(l.t)}</time><span class="log-type ${l.type.toLowerCase()}">${ko(TYPE_KO, l.type)}</span><span class="log-source" title="${esc(l.source)}">${esc(l.source)}</span><span class="log-message" title="${esc(l.message)}">${esc(l.message)}</span><span class="log-state ${l.type.toLowerCase()}">${esc(ko(LOGSTATE_KO, l.state))}</span></div>`).join('');
  if ($('autoScroll').checked) box.scrollTop = box.scrollHeight;
  $('logCount').textContent = String(ui.logs.length).padStart(2, '0');
  $('bringupCount').textContent = String(ui.blogs.length).padStart(2, '0');
  $('bringupErrDot').hidden = !ui.bringupErr;
  $('visibleCount').textContent = Math.min(visible.length, 300);
  $('totalCount').textContent = logs.length;
  $('faultValue').textContent = `${ui.faults}건`;
  $('faultValue').className = ui.faults ? 'error-text' : '';
}

function setChannel(channel) {
  ui.channel = channel === 'bringup' ? 'bringup' : 'main';
  if (ui.channel === 'bringup') ui.bringupErr = false;
  document.querySelectorAll('.log-channel').forEach((b) => b.classList.toggle('active', b.dataset.channel === ui.channel));
  renderLogs();
}

// http://<IP>:8080 (보안 컨텍스트 아님) 에서는 navigator.clipboard 가 없으므로 textarea 방식으로 대신한다
function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) return navigator.clipboard.writeText(text);
  const ta = document.createElement('textarea');
  ta.value = text;
  ta.style.cssText = 'position:fixed;top:0;left:0;opacity:0';
  document.body.appendChild(ta);
  ta.select();
  const ok = document.execCommand('copy');
  ta.remove();
  return ok ? Promise.resolve() : Promise.reject(new Error('copy failed'));
}

function copyLogs() {
  const logs = ui.channel === 'bringup' ? ui.blogs : ui.logs;
  if (!logs.length) { toast('복사할 로그가 없습니다.', 'warn'); return; }
  const text = logs.map((l) => `${fmtTime(l.t)} [${l.type}] ${l.source}${l.run_id ? ` #${l.run_id}` : ''}: ${l.raw || l.message}`).join('\n');
  const name = ui.channel === 'bringup' ? '로봇 bringup' : '작업';
  copyText(text).then(() => toast(`${name} 로그 ${logs.length}줄을 복사했습니다.`, 'info')).catch(() => toast('복사하지 못했습니다.'));
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
  const alive = runs.filter((r) => r.state === 'starting' || r.state === 'running');
  const active = alive.filter((r) => r.kind !== 'service');     // 계속 떠 있는 서버/bringup 은 '작업' 이 아님
  const work = runs.filter((r) => r.kind !== 'service');
  const main = active.find((r) => r.motion) || active[active.length - 1] || work[work.length - 1];
  // 서비스만 보이고 /joint_states 가 없으면(꺼진 bringup 의 잔여·고아 컨트롤러) 연결 안 된 것으로 본다
  renderModeChip(alive, { ...(s.links || {}), controller: Boolean(s.links?.controller && !(s.robot || {}).stale) });
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

  // --- 실시간 작업 현황 상단 바
  $('monStatus').textContent = STATUS_KO[status];
  $('monDot').style.background = color;
  $('monDot').style.boxShadow = `0 0 8px ${color}`;
  $('monitorStrip').classList.toggle('alert', alert);
  $('monTask').textContent = $('taskName').textContent;
  $('monSub').textContent = $('taskSub').textContent;
  $('monElapsed').textContent = $('elapsedValue').textContent;
  $('monProgressLabel').textContent = $('progressLabel').textContent;
  $('monProgressValue').textContent = $('progressValue').textContent;
  $('monProgressTrack').classList.toggle('indeterminate', track.classList.contains('indeterminate'));
  $('monProgressBar').style.width = `${pct ?? 0}%`;
  $('monProgressBar').nextElementSibling.style.left = `${pct ?? 0}%`;
  $('monitorBadge').hidden = !active.length;
  $('monitorBadge').textContent = active.length;

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
    $('monForce').textContent = mag.toFixed(1);
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
  drawArm(robot.joints, p);
  if (ui.view === 'path') drawPath(p);
  $('posxText').textContent = p ? p.map((v) => v.toFixed(1)).join(', ') : '--';
  renderJoints(robot.joints);

  // --- 안전
  const banner = $('safetyBanner');
  banner.className = `safety-banner ${s.estop || badRobot ? 'alert' : 'safe'}`;
  banner.querySelector('.safety-symbol').textContent = s.estop || badRobot ? '!' : '✓';
  $('safetyTitle').textContent = s.estop ? '비상 정지 작동 중' : badRobot ? `컨트롤러 ${robotKo(robot.state_name)}` : '이상 없음';
  $('safetyDetail').textContent = s.estop ? '주변 확인 후 비상 정지를 해제하세요. 해제 전에는 작업을 시작할 수 없습니다.' : badRobot ? '티칭 펜던트에서 상태를 확인하고 해제하세요.' : '비상 정지 버튼 사용 가능';
  $('safetyEstop').textContent = s.estop ? '작동 중' : '해제됨';
  $('safetyEstop').className = s.estop ? 'error-text' : 'safe-text';
  $('safetyRobot').textContent = robotKo(robot.state_name);
  $('safetyRobot').className = badRobot ? 'error-text' : robot.state == null ? '' : 'safe-text';
  $('safetyFoot').textContent = s.estop ? '비상 정지' : badRobot ? '고장' : '정상';
  $('estopBtn').classList.toggle('triggered', s.estop);
  $('estopLabel').textContent = s.estop ? '비상 정지 해제' : '비상 정지';
  $('armedTag').textContent = s.estop ? '잠김' : '준비';
  const robotReady = Boolean((links.controller && !robot.stale) || links.fake);   // /joint_states 까지 와야 연결. fake:=true 는 bringup 없이 DB 흐름 시험
  $('startBtn').disabled = s.estop || !robotReady;
  $('startBtn').title = robotReady ? '고른 작업을 시작 (공정은 필요한 액션 서버가 없으면 먼저 켬)' : '로봇 bringup 이 꺼져 있습니다. sodvir / sodreal 을 먼저 누르세요';
  $('startTaskName').textContent = robotReady ? (currentTask()?.label || '') : 'bringup 필요';
  $('homeBtn').disabled = s.estop || !(links.controller && !robot.stale);
  $('gripOpenBtn').disabled = $('gripCloseBtn').disabled = $('homeBtn').disabled;
  $('speedUnlockBtn').disabled = s.estop && !s.speed?.unlocked;
  renderSpeed(s.speed);

  renderRuns(runs, active, waiting);
}

const SERVER_IDS = ['explore_server', 'putty_task_server', 'tool_task_server'];
function renderModeChip(alive, links) {
  const up = alive.find((r) => r.task_id === 'bringup');
  if (up && up.mode !== ui.mode) setMode(up.mode);          // 모드는 bringup 을 따라간다
  const servers = alive.filter((r) => SERVER_IDS.includes(r.task_id)).length;
  const chip = $('modeChip');
  let text; let sub;
  const srv = servers ? ` · 서버 ${servers}개 실행 중` : '';
  if (up) { text = MODE_KO[up.mode]; sub = links.controller ? `bringup 연결됨${srv}` : 'bringup 시작 중…'; }
  else if (links.controller) { text = `${MODE_KO[ui.mode]} (외부 bringup)`; sub = `모드가 맞는지 확인${srv}`; }
  else { text = 'bringup 꺼짐'; sub = 'sodvir 또는 sodreal 을 누르세요'; }
  $('modeChipText').textContent = text;
  $('modeChipSub').textContent = sub;
  chip.dataset.state = !up && !links.controller ? 'off' : links.controller ? 'ok' : 'wait';
  chip.classList.toggle('real', (up ? up.mode : ui.mode) === 'real' && chip.dataset.state !== 'off');
  $('bringupStopBtn').disabled = !up;
  $('sodvirBtn').classList.toggle('active', Boolean(up) && up.mode === 'virtual');
  $('sodrealBtn').classList.toggle('active', Boolean(up) && up.mode === 'real');
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

// ------------------------------------------------------------------ 로봇 자세 (관절 = 원, 링크 = 선, TCP = 그리퍼)
// M0609 URDF(dsr_description2/xacro/macro.m0609.*.xacro) 의 joint origin [xyz(m), rpy(rad)]
const M0609_JOINTS = [
  [[0, 0, 0.1345], [0, 0, 0]],
  [[0, 0.0062, 0], [0, -1.571, -1.571]],
  [[0.411, 0, 0], [0, 0, 1.571]],
  [[0, -0.368, 0], [1.571, 0, 0]],
  [[0, 0, 0], [-1.571, 0, 0]],
  [[0, -0.121, 0], [1.571, 0, 0]],
];
const HOME_Q = [0, 0, 90, 0, 90, 0];
const GRIP_DEFAULT_LEN = 150; // posx 가 없을 때 그리퍼 길이 (mm)

const mmul = (A, B) => A.map((row) => [0, 1, 2, 3].map((j) => row.reduce((acc, v, k) => acc + v * B[k][j], 0)));
function rpyMat(r, p, y, t = [0, 0, 0]) {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y)];
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, t[0]],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, t[1]],
    [-sp, cp * sr, cp * cr, t[2]],
    [0, 0, 0, 1],
  ];
}

/** 관절각(deg) → 각 관절 원점 위치(mm)와 플랜지 좌표계 */
function forwardKinematics(qDeg) {
  let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const pts = [];
  M0609_JOINTS.forEach(([xyz, rpy], i) => {
    T = mmul(mmul(T, rpyMat(rpy[0], rpy[1], rpy[2], xyz)), rpyMat(0, 0, (qDeg[i] * Math.PI) / 180));
    pts.push([T[0][3] * 1000, T[1][3] * 1000, T[2][3] * 1000]);
  });
  const axis = (c) => [T[0][c], T[1][c], T[2][c]];
  return { pts, x: axis(0), y: axis(1), z: axis(2) };
}

// 고정 시점: 로봇의 오른쪽 앞 위(+X, -Y, +Z 방향)에서 본 등각(3/4) 투영
//   화면에서 +X 는 오른쪽 아래(앞으로 나옴), +Y 는 오른쪽 위(뒤로 들어감), +Z 는 위
const ISO_C = Math.cos(Math.PI / 6);
const ISO_S = Math.sin(Math.PI / 6);
const iso = ([x, y, z]) => [(x + y) * ISO_C, (x - y) * ISO_S - z];
const isoStr = (p) => iso(p).map((v) => v.toFixed(1)).join(',');
const add = (a, b, k = 1) => [a[0] + b[0] * k, a[1] + b[1] * k, a[2] + b[2] * k];
const lerp = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

const GRID = { xMin: -200, xMax: 800, yMin: -500, yMax: 500, step: 100, zMax: 850 };

function drawFloor() {
  let g = '';
  for (let x = GRID.xMin; x <= GRID.xMax; x += GRID.step) g += `<line x1="${iso([x, GRID.yMin, 0])[0]}" y1="${iso([x, GRID.yMin, 0])[1]}" x2="${iso([x, GRID.yMax, 0])[0]}" y2="${iso([x, GRID.yMax, 0])[1]}" class="g${x === 0 ? ' axis' : ''}"/>`;
  for (let y = GRID.yMin; y <= GRID.yMax; y += GRID.step) g += `<line x1="${iso([GRID.xMin, y, 0])[0]}" y1="${iso([GRID.xMin, y, 0])[1]}" x2="${iso([GRID.xMax, y, 0])[0]}" y2="${iso([GRID.xMax, y, 0])[1]}" class="g${y === 0 ? ' axis' : ''}"/>`;
  // 좌표축 표시 (방향만, 바닥 격자 왼쪽 모서리에 150mm). 베이스 기둥에 가리지 않게 옮겨 둠
  const o = [GRID.xMin, GRID.yMin, 0];
  const [ox, oy] = iso(o);
  [['X', [150, 0, 0]], ['Y', [0, 150, 0]], ['Z', [0, 0, 150]]].forEach(([n, v]) => {
    const [x2, y2] = iso(add(o, v));
    const [lx, ly] = iso(add(o, v, 1.3));
    g += `<line x1="${ox}" y1="${oy}" x2="${x2}" y2="${y2}" class="axis-arrow" marker-end="url(#axisHead)"/><text x="${lx}" y="${ly}" class="lbl axis-l" text-anchor="middle" dominant-baseline="middle">${n}</text>`;
  });
  return g;
}

function viewBoxFor() {
  const corners = [];
  [GRID.xMin, GRID.xMax].forEach((x) => [GRID.yMin, GRID.yMax].forEach((y) => [0, GRID.zMax].forEach((z) => corners.push(iso([x, y, z])))));
  const xs = corners.map((c) => c[0]);
  const ys = corners.map((c) => c[1]);
  const pad = 40;
  const padRight = 520; // TCP 좌표 라벨 공간
  return `${Math.min(...xs) - pad} ${Math.min(...ys) - pad} ${Math.max(...xs) - Math.min(...xs) + pad + padRight} ${Math.max(...ys) - Math.min(...ys) + pad * 2}`;
}

function drawArm(joints, posx) {
  const svg = $('armView');
  if (!svg.dataset.ready) {
    svg.setAttribute('viewBox', viewBoxFor());
    svg.dataset.ready = '1';
  }
  const live = Array.isArray(joints) && joints.length >= 6 && joints.every(Number.isFinite);
  const q = live ? joints : HOME_Q;
  const fk = forwardKinematics(q);
  const [, p2, p3, p4, , p6] = fk.pts;          // p1≈p2 (어깨), p4=p5 (손목)
  const base = [0, 0, 0];

  // 관절 표시 위치: 각 관절의 회전축 위 (J1 은 기둥, J4 는 아래팔 축 위, J6 은 플랜지)
  const marks = [
    ['J1', [0, 0, 70]],
    ['J2', p2],
    ['J3', p3],
    ['J4', lerp(p3, p4, 0.72)],
    ['J5', p4],
    ['J6', p6],
  ];

  // 그리퍼: 플랜지 z 축 방향. 길이는 실제 TCP(posx) 가 있으면 그에 맞춤
  let len = GRIP_DEFAULT_LEN;
  if (live && posx) {
    const d = dot([posx[0] - p6[0], posx[1] - p6[1], posx[2] - p6[2]], fk.z);
    if (d > 60 && d < 350) len = d;
  }
  const bodyEnd = add(p6, fk.z, len * 0.45);
  const halfW = 42;
  const palmA = add(bodyEnd, fk.y, halfW);
  const palmB = add(bodyEnd, fk.y, -halfW);
  const tipA = add(palmA, fk.z, len * 0.55);
  const tipB = add(palmB, fk.z, len * 0.55);
  const tcp = add(p6, fk.z, len);

  let out = '<defs><marker id="axisHead" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="axis-head"/></marker></defs>';
  out += drawFloor();

  // 그림자 (바닥 투영) - 자세를 바닥에서 읽기 쉽게
  const shadow = [base, p2, p3, p4, p6, tcp].map((p) => isoStr([p[0], p[1], 0])).join(' ');
  out += `<polyline points="${shadow}" class="arm-shadow"/>`;

  // TCP 궤적
  if (ui.trail.length > 1) out += `<polyline points="${ui.trail.map((t) => isoStr(t)).join(' ')}" class="trail"/>`;

  // 베이스
  out += `<polygon points="${[[-90, -90, 0], [90, -90, 0], [90, 90, 0], [-90, 90, 0]].map(isoStr).join(' ')}" class="base-plate"/>`;

  // 링크
  const chain = [base, p2, p3, p4, p6];
  out += `<polyline points="${chain.map(isoStr).join(' ')}" class="link${live ? '' : ' ghost'}"/>`;

  // 그리퍼 (고정 모양: 몸통 + 손바닥 + 손가락 2개)
  out += `<g class="gripper${live ? '' : ' ghost'}">`;
  out += `<line x1="${iso(p6)[0]}" y1="${iso(p6)[1]}" x2="${iso(bodyEnd)[0]}" y2="${iso(bodyEnd)[1]}" class="grip-body"/>`;
  out += `<polyline points="${[tipA, palmA, palmB, tipB].map(isoStr).join(' ')}" class="grip-finger"/>`;
  out += `<circle cx="${iso(tcp)[0]}" cy="${iso(tcp)[1]}" r="9" class="tcp-dot"/>`;
  out += '</g>';

  // 관절 원 + 각도 라벨
  marks.forEach(([name, p], i) => {
    const [x, y] = iso(p);
    out += `<circle cx="${x}" cy="${y}" r="${i === 0 ? 20 : 17}" class="joint${live ? '' : ' ghost'}"/>`;
    // J4 는 J5 와 가까워 라벨을 원의 왼쪽 아래에 둔다
    const below = name === 'J4';
    out += `<text x="${below ? x - 24 : x + 28}" y="${below ? y + 50 : y - 22}" text-anchor="${below ? 'end' : 'start'}" class="lbl joint-l"><tspan class="jn">${name}</tspan> ${live ? `${q[i].toFixed(1)}°` : '--'}</text>`;
  });

  // TCP 좌표 라벨
  const [tx, ty] = iso(tcp);
  const tcpText = posx ? `X ${posx[0].toFixed(1)}  Y ${posx[1].toFixed(1)}  Z ${posx[2].toFixed(1)}` : '좌표 없음';
  out += `<text x="${tx + 30}" y="${ty + 40}" class="lbl tcp-l"><tspan class="jn">TCP</tspan> ${tcpText}</text>`;

  if (!live) {
    const [cx, cy] = iso([300, 0, 900]);
    out += `<text x="${cx}" y="${cy}" class="lbl nodata" text-anchor="middle">관절 데이터 없음 · 홈 자세로 표시</text>`;
  }
  svg.innerHTML = out;
}

// 위에서 본 TCP 경로: 화면 위 = +X(앞), 왼쪽 = +Y. 격자·궤적은 drawArm 과 같은 GRID / ui.trail 사용
const top2d = (p) => [-p[1], -p[0]];
function drawPath(posx) {
  const svg = $('pathView');
  if (!svg.dataset.ready) {
    svg.setAttribute('viewBox', `${GRID.yMin - 60} ${-GRID.xMax - 60} ${GRID.yMax - GRID.yMin + 120 + 380} ${GRID.xMax - GRID.xMin + 120}`);
    svg.dataset.ready = '1';
  }
  const pt = (p) => top2d(p).map((v) => v.toFixed(1)).join(',');
  const line = (a, b, cls) => { const [x1, y1] = top2d(a); const [x2, y2] = top2d(b); return `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" class="${cls}"/>`; };
  let out = '<defs><marker id="pathHead" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="axis-head"/></marker></defs>';
  for (let x = GRID.xMin; x <= GRID.xMax; x += GRID.step) out += line([x, GRID.yMin], [x, GRID.yMax], `g${x === 0 ? ' axis' : ''}`);
  for (let y = GRID.yMin; y <= GRID.yMax; y += GRID.step) out += line([GRID.xMin, y], [GRID.xMax, y], `g${y === 0 ? ' axis' : ''}`);
  const o = [GRID.xMin, GRID.yMax];
  [['X', [150, 0]], ['Y', [0, 150]]].forEach(([n, v]) => {
    const e = [o[0] + v[0], o[1] + v[1]];
    const [lx, ly] = top2d([o[0] + v[0] * 1.35, o[1] + v[1] * 1.35]);
    out += `${line(o, e, 'axis-arrow').replace('/>', ' marker-end="url(#pathHead)"/>')}<text x="${lx}" y="${ly}" class="lbl axis-l" text-anchor="middle" dominant-baseline="middle">${n}</text>`;
  });
  out += `<polygon points="${[[-90, -90], [90, -90], [90, 90], [-90, 90]].map(pt).join(' ')}" class="base-plate"/>`;
  if (ui.trail.length > 1) out += `<polyline points="${ui.trail.map(pt).join(' ')}" class="trail"/>`;
  if (posx) {
    const [tx, ty] = top2d(posx);
    out += `<circle cx="${tx}" cy="${ty}" r="12" class="tcp-dot"/>`;
    out += `<text x="${tx + 30}" y="${ty + 10}" class="lbl tcp-l"><tspan class="jn">TCP</tspan> X ${posx[0].toFixed(1)}  Y ${posx[1].toFixed(1)}  Z ${posx[2].toFixed(1)}</text>`;
  } else {
    const [cx, cy] = top2d([300, 0]);
    out += `<text x="${cx}" y="${cy}" class="lbl nodata" text-anchor="middle">좌표 없음</text>`;
  }
  svg.innerHTML = out;
}

function setView(view) {
  ui.view = view === 'path' ? 'path' : 'coords';
  $('pathBox').hidden = ui.view !== 'path';
  $('armBox').hidden = ui.view === 'path';
  document.querySelectorAll('.view-btn').forEach((b) => b.classList.toggle('active', b.dataset.view === ui.view));
  if (ui.view === 'path') drawPath(ui.state?.robot?.posx || null);
  saveUi();
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
  $('bridgeText').textContent = on ? '서버 연결됨' : '서버 연결 끊김';
  $('bridgeChip').classList.toggle('down', !on);
  $('railStatus').textContent = on ? 'HMI 서버 · 연결됨' : 'HMI 서버 · 연결 끊김';
  setDot('dotBridge', on ? 'green' : 'red');
  $('commBridge').textContent = on ? '연결됨' : '재연결 중';
  $('streamText').textContent = on ? '수신 중' : '다시 연결하는 중…';
  $('streamDot').className = on ? 'green' : '';
}

function connect() {
  const es = new EventSource('/api/events');
  es.addEventListener('open', () => setConnected(true));
  es.addEventListener('error', () => setConnected(false));
  es.addEventListener('backlog', (e) => { ui.logs = []; ui.blogs = []; ui.faults = 0; addLogs(JSON.parse(e.data)); });
  es.addEventListener('log', (e) => addLogs([JSON.parse(e.data)], true));
  es.addEventListener('speed', (e) => renderSpeed(JSON.parse(e.data)));
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
  $('homeBtn').addEventListener('click', goHome);
  $('gripOpenBtn').addEventListener('click', () => gripper('open'));
  $('gripCloseBtn').addEventListener('click', () => gripper('close'));
  $('sodvirBtn').addEventListener('click', () => bringup('virtual'));
  $('sodrealBtn').addEventListener('click', () => bringup('real'));
  $('bringupStopBtn').addEventListener('click', stopBringup);
  document.querySelectorAll('.page-tab').forEach((b) => b.addEventListener('click', () => setPage(b.dataset.page)));
  window.addEventListener('hashchange', () => setPage(location.hash.slice(1), false));
  document.querySelectorAll('[data-uimode]').forEach((b) => b.addEventListener('click', () => setUiMode(b.dataset.uimode)));
  $('resetParamsBtn').addEventListener('click', resetParams);
  const range = $('speedRange');
  range.addEventListener('input', () => { ui.speedDragging = true; renderSpeed(ui.speed); });
  range.addEventListener('change', () => { ui.speedDragging = false; setSpeed(Number(range.value)); });
  $('speedDefaultBtn').addEventListener('click', () => setSpeed(100));
  $('speedUnlockBtn').addEventListener('click', toggleSpeedLock);
  document.querySelectorAll('.layout-btn').forEach((b) => b.addEventListener('click', () => setLayout(b.dataset.layout)));
  $('alertPopup').addEventListener('change', (e) => { ui.alertOn = e.target.checked; if (!ui.alertOn) clearAlerts(); saveUi(); });
  $('alertClear').addEventListener('click', clearAlerts);
  document.querySelectorAll('[data-filter]').forEach((b) => b.addEventListener('click', () => setFilter(b.dataset.filter)));
  $('clearBtn').addEventListener('click', () => {
    if (ui.channel === 'bringup') { ui.blogs = []; ui.bringupErr = false; } else { ui.logs = []; ui.faults = 0; ui.trail = []; }
    renderLogs();
  });
  document.querySelectorAll('.log-channel').forEach((b) => b.addEventListener('click', () => setChannel(b.dataset.channel)));
  $('copyLogsBtn').addEventListener('click', copyLogs);
  $('logFullBtn').addEventListener('click', () => {
    const on = document.querySelector('.event-console').classList.toggle('log-full');
    $('logFullBtn').textContent = on ? '✕ 닫기' : '⛶ 전체 보기';
    if (on) { const box = $('consoleLog'); box.scrollTop = box.scrollHeight; }
  });
  document.querySelectorAll('.view-btn').forEach((b) => b.addEventListener('click', () => setView(b.dataset.view)));
  $('copyPosxBtn').addEventListener('click', () => {
    const t = $('posxText').textContent;
    if (t === '--') { toast('복사할 좌표가 없습니다.', 'warn'); return; }
    copyText(t).then(() => toast(`posx 복사: ${t}`, 'info')).catch(() => toast('복사하지 못했습니다.'));
  });
  $('focusBtn').addEventListener('click', () => $('simulationCard').classList.toggle('focused'));
  $('stdinForm').addEventListener('submit', (e) => { e.preventDefault(); sendStdin($('stdinText').value); });
  document.querySelectorAll('[data-quick]').forEach((b) => b.addEventListener('click', () => sendStdin(b.dataset.quick)));
  document.addEventListener('keydown', (e) => {
    const typing = ['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName);
    if (!typing && (e.key === '1' || e.key === '2') && !e.ctrlKey && !e.altKey && !e.metaKey) { setPage(e.key === '1' ? 'control' : 'monitor'); return; }
    if (e.key === 'Escape' && !ui.state?.estop) { e.preventDefault(); api('/api/estop', {}).catch((err) => toast(err.message)); }
  });
  $('footerHost').textContent = location.host;
}

async function init() {
  loadUi();
  bind();
  setMode(ui.mode === 'real' ? 'real' : 'virtual');
  setLayout(ui.layout);
  setPage(PAGES.includes(location.hash.slice(1)) ? location.hash.slice(1) : ui.page);
  setUiMode(ui.uiMode);
  $('alertPopup').checked = ui.alertOn !== false;
  ui.alertOn = ui.alertOn !== false;
  ui.speedDragging = false;
  renderSpeed(ui.speed);
  setConnected(false);
  renderJoints(null);
  drawArm(null, null);
  setView(ui.view);
  updateLatencyBars();
  setInterval(() => {
    const d = new Date();
    $('topClock').textContent = d.toLocaleTimeString('en-GB', { hour12: false });
    $('topDate').textContent = d.toLocaleDateString('ko-KR', { month: 'long', day: 'numeric', weekday: 'short' });
  }, 1000);
  setInterval(measureLatency, 2000);
  for (;;) {
    try {
      const data = await api('/api/tasks');
      ui.tasks = data.tasks;
      break;
    } catch (e) {
      $('taskDesc').textContent = 'hmi_server 에 연결하지 못했습니다. 2초 뒤 다시 시도합니다.';
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
  if (!['navigation', 'putty', 'sanding', 'process'].includes(ui.group)) ui.group = 'process';
  renderTaskList();
  connect();
}

init();
