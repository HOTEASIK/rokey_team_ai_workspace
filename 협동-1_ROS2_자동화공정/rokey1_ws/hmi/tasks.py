"""HMI 에서 실행할 수 있는 작업(navigation / putty_demo / sanding / 홈 이동) 카탈로그.

각 작업은
  - params : 화면에 그릴 입력 항목 (key, label, type, default, options, help)
  - build  : (mode, params) -> 실행할 명령 목록 [[argv...], ...]  (2개 이상이면 순차 실행)
  - motion : (mode) -> 로봇을 실제로 움직이는지 (동시에 하나만 허용)
  - kind   : process(끝나는 노드) | service(계속 떠 있는 노드) | action(HMI 가 직접 goal 전송)
를 가진다. 명령은 셸 없이 argv 리스트로 실행하므로 입력값이 셸로 해석되지 않는다.
"""

PKG = 'rokey1'
MODES = ('virtual', 'real')

DEFAULT_START = '364.96,-184.31,92.91,61.79,-177.53,61.44'
DEFAULT_END = '464.96,-184.31,92.91,61.79,-177.53,61.44'


def _run(exe, *args):
    return ['ros2', 'run', PKG, exe, *args]


def _ros_params(**kw):
    """{'mode': 'virtual', 'face': 500.0} -> ['--ros-args', '-p', 'mode:=virtual', ...] (빈 값은 생략)."""
    out = []
    for key, value in kw.items():
        if value is None or value == '':
            continue
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        out += ['-p', f'{key}:={value}']
    return ['--ros-args', *out] if out else []


def _p(key, label, type_, default, options=None, help_='', scale=False,
       min_=None, max_=None, integer=False, maxlen=None, remember=True):
    """scale=True 이면 HMI 속도가 100% 를 넘을 때(안전 해제) 그 배율만큼 값을 키운다.
    min_/max_ (포함), integer: 숫자 칸 범위/정수. maxlen: 텍스트 최대 글자 수.
    remember=False: 브라우저에 지난 입력값을 남기지 않음 (매번 새로 입력)."""
    item = {'key': key, 'label': label, 'type': type_, 'default': default, 'help': help_, 'scale': scale,
            'min': min_, 'max': max_, 'integer': integer, 'maxlen': maxlen, 'remember': remember}
    if options is not None:
        item['options'] = list(options)
    return item


# ----------------------------------------------------------------------------- putty_demo
def _initialize_args(mode, p):
    return _ros_params(execute_motion=True, virtual_gripper=mode != 'real')


def _pickup_args(mode, p):
    return ['--execute-motion', *(['--virtual-gripper'] if mode != 'real' else []),
            '--clearance', str(p['clearance']), '--settle', str(p['settle']),
            '--transfer-speed', str(p['transfer_speed']), '--pick-speed', str(p['pick_speed'])]


PICKUP_PARAMS = [
    _p('clearance', '접근 거리 (mm)', 'number', 50.0, min_=1.0, max_=200.0,
       help_='헤라 바로 위에서 멈추는 높이. 클수록 안전하지만 시간이 더 걸림'),
    _p('settle', '집게 대기 (s)', 'number', 1.0, min_=0.0, max_=10.0,
       help_='RG2 를 닫은 뒤 다음 동작까지 기다리는 시간'),
    _p('transfer_speed', '이송 속도 (mm/s)', 'number', 15.0, scale=True, min_=1.0, max_=100.0,
       help_='헤라 위까지 크게 이동할 때 속도 (속도 % 가 100 을 넘으면 비례 증가)'),
    _p('pick_speed', '집기 속도 (mm/s)', 'number', 5.0, scale=True, min_=1.0, max_=50.0,
       help_='집기 높이로 내려가고 들어 올릴 때 속도 (느릴수록 정확)'),
]


# ----------------------------------------------------------------------------- 홈 이동
HOME_J = '0,0,90,0,90,0'
HOME_BASE_VEL = 30.0     # 속도 100% 기준 관절 속도 [deg/s]


def _homeset(mode, p):
    return [_run('homeset', *_ros_params(
        mode=mode, home=p['home'], vel=p['vel'], acc=p['acc']))]


# ----------------------------------------------------------------------------- 공정(DB)
DB_PARAMS = [
    _p('purpose', '목적', 'select', 'TEST', ['TEST', 'EXPERIMENT', 'DEMO']),
    _p('material', '재료', 'select', 'clay', ['clay', 'putty']),
    _p('note', '메모', 'text', '', maxlen=200, help_='DB run.note (200자 이하)'),
]
GAP_PARAM = _p('gap_id', '틈 번호 (gap_id)', 'number', '', min_=1, integer=True, remember=False,
               help_='DB gap 테이블의 번호 (1 이상 정수). 탐색(EXPLORE/전체 공정) 로그에 찍힌 gap_id 를 매번 넣는다')


# [전체 공정] 시작 단계: 0 처음부터(탐색) → 1 내부 탐색 → 2 퍼티 → 3 샌딩
#   1 이상이면 벽 탐색을 건너뛰고 DB 의 마지막 벽(wall)과 그 틈(gap)을 쓴다.
START_STAGES = ('0: 처음부터 (탐색)', '1: 내부 탐색부터', '2: 퍼티 작업부터', '3: 샌딩 작업부터')
STAGE_PARAM = _p('start_stage', '시작 단계', 'select', START_STAGES[0], START_STAGES,
                 help_='0 = 벽 탐색부터 전부. 1~3 = 벽 탐색을 건너뛰고 DB 의 마지막 벽/틈으로 그 단계부터 진행 '
                       '(1 내부 탐색은 아직 기능 없음 → robot.yaml inner_explore.enabled 가 false 면 건너뜀)')


def start_stage(params):
    """'2: 퍼티 작업부터' → 2 (없으면 0)"""
    return int(str((params or {}).get('start_stage') or '0').split(':')[0])


def process_actions(process, params=None):
    """공정에 필요한 /process/* 액션 (시작 단계에 따라 앞 단계 서버는 필요 없음)"""
    if process != 'ALL':
        return PROCESS_ACTIONS[process]
    stage = start_stage(params)
    return (('EXPLORE',) if stage == 0 else ()) + (PUTTY_ACTIONS if stage <= 2 else ()) + ('SANDING',)


def _db_task(process, label, desc, params):
    return {'id': f'process_{process.lower()}', 'group': 'process', 'kind': 'db', 'process': process,
            'label': label, 'desc': desc, 'params': params, 'build': None, 'motion': lambda mode: True}


# ----------------------------------------------------------------------------- bringup ([sodvir]/[sodreal] 버튼)
ROBOT_HOST = {'virtual': '127.0.0.1', 'real': '192.168.1.100'}   # ~/.bashrc 의 sodvir / sodreal 과 같은 값
# 공정 ▶ 시작 때 필요한 /process/* 액션과, 그 액션을 제공하는 서버 (없으면 HMI 가 켠다)
PUTTY_ACTIONS = ('PUTTY_PICKUP', 'PUTTY_SCOOP', 'PUTTY', 'PUTTY_RETURN')
PROCESS_ACTIONS = {'EXPLORE': ('EXPLORE',), 'PUTTY': PUTTY_ACTIONS, 'SANDING': ('SANDING',),
                   'ALL': ('EXPLORE', *PUTTY_ACTIONS, 'SANDING')}
ACTION_SERVER = {'EXPLORE': 'explore_server', 'SANDING': 'tool_task_server',
                 **{a: 'putty_task_server' for a in PUTTY_ACTIONS}}


def _bringup(mode, p):
    return [['ros2', 'launch', 'm0609_rg2_bringup', 'bringup.launch.py', f'mode:={mode}',
             f'host:={ROBOT_HOST[mode]}', 'port:=12345', 'model:=m0609']]


# ----------------------------------------------------------------------------- 카탈로그
TASKS = [
    # ===== 공정 (DB 기록) =====
    _db_task('ALL', '전체 공정', '탐색 → 내부 탐색 → 찾은 틈마다 퍼티 → 샌딩. 결과는 DB(run/wall/gap/task_log)에 저장. '
                                 '[시작 단계] 1~3 은 DB 의 마지막 벽/틈으로 그 단계부터 진행.',
             [STAGE_PARAM] + DB_PARAMS),
    _db_task('EXPLORE', '탐색', '/process/explore 만 실행. 찾은 벽/틈을 DB에 저장.', DB_PARAMS),
    _db_task('PUTTY', '퍼티', '지정한 gap_id 한 틈에 헤라 집기 → 뜨기 → 도포 → 반납.', [GAP_PARAM] + DB_PARAMS),
    _db_task('SANDING', '샌딩', '지정한 gap_id 한 틈에 /process/sanding 실행.', [GAP_PARAM] + DB_PARAMS),

    # ===== NAVIGATION =====
    {
        'id': 'explore_server', 'group': 'navigation', 'kind': 'service',
        'label': '탐색 액션 서버',
        'desc': '/process/explore 액션 서버 (wall_tap 실행 → 벽/틈을 DB 로). [공정] 시작 때 없으면 자동으로 켜짐.',
        'params': [],
        'build': lambda mode, p: [_run('explore_server', *_ros_params(mode=mode))],
        'motion': lambda mode: False, 'mode_bound': True,
    },

    # ===== PUTTY =====
    {
        'id': 'putty_cycle', 'group': 'putty', 'kind': 'process',
        'label': '퍼티 전체 순서',
        'desc': '[초기화 → 헤라 집기] 를 순서대로 실행. 앞 단계가 실패하면 뒤 단계는 실행하지 않음. '
                '가상 모드는 로봇 없이 순서만 점검(dry-run).',
        'params': PICKUP_PARAMS,
        'build': lambda mode, p: [_run('initialize_device', *_initialize_args(mode, p)),
                                  _run('pickup_spatula', *_pickup_args(mode, p))],
        'motion': lambda mode: True,
    },
    {
        'id': 'initialize_device', 'group': 'putty', 'kind': 'process',
        'label': '초기화',
        'desc': '홈 자세로 돌아간 뒤 빈 RG2 그리퍼를 닫는다. 가상 모드는 에뮬레이터 + 가상 그리퍼.',
        'params': [],
        'build': lambda mode, p: [_run('initialize_device', *_initialize_args(mode, p))],
        'motion': lambda mode: True,
    },
    {
        'id': 'pickup_spatula', 'group': 'putty', 'kind': 'process',
        'label': '헤라 집기',
        'desc': '7번 위치의 헤라 위로 이동 → 내려가서 RG2 로 집기 → 들어 올리기.',
        'params': PICKUP_PARAMS,
        'build': lambda mode, p: [_run('pickup_spatula', *_pickup_args(mode, p))],
        'motion': lambda mode: True,
    },

    {
        'id': 'putty_task_server', 'group': 'putty', 'kind': 'service',
        'label': '퍼티 액션 서버',
        'desc': '/process/putty_pickup · putty_scoop · putty · putty_return, /dsr01/putty_task 액션 서버. '
                '[공정] 시작 때 없으면 자동으로 켜짐 (virtual 이면 가상 그리퍼).',
        'params': [],
        'build': lambda mode, p: [_run('putty_task_server', *_ros_params(process_virtual_mode=mode != 'real'))],
        'motion': lambda mode: False, 'mode_bound': True,
    },

    # ===== SANDING =====
    {
        'id': 'tool_task_server', 'group': 'sanding', 'kind': 'service',
        'label': '샌딩 액션 서버',
        'desc': '/dsr01/tool_task + /process/sanding 액션 서버 (sanding_server). '
                '[샌딩 작업 실행] 전에 시작 · [공정] 시작 때는 자동으로 켜짐 (RG2 = 디지털 출력, robot.yaml sanding.gripper).',
        'params': [],
        'build': lambda mode, p: [_run('tool_task_server', *_ros_params(virtual=mode != 'real'))],
        'motion': lambda mode: False, 'mode_bound': True,
    },
    {
        'id': 'sanding_goal', 'group': 'sanding', 'kind': 'action',
        'label': '샌딩 작업 실행',
        'desc': '도구 거치대에서 도구를 집고 → 시작점에서 종료점까지 직선으로 샌딩한다 (ToolTask goal 전송).',
        'params': [
            _p('tool_id', '도구 번호', 'select', '1', ['1'],
               help_='거치대에서 집을 도구 번호 (sanding_server TOOL_POSX 에 위치가 등록된 것만)'),
            _p('start_posx', '시작 posx x,y,z,a,b,c', 'text', DEFAULT_START,
               help_='작업 시작 위치 (BASE 기준, mm / deg)'),
            _p('end_posx', '종료 posx x,y,z,a,b,c', 'text', DEFAULT_END,
               help_='작업 종료 위치 (BASE 기준, mm / deg)'),
            _p('return_tool', '작업 후 도구 반납', 'bool', False, help_='끝나면 도구를 거치대에 다시 놓음'),
        ],
        'build': None, 'motion': lambda mode: True,
    },

    # ===== SYSTEM (탭에는 안 보이고 [홈 위치로 이동] / [sodvir]·[sodreal] 버튼이 사용) =====
    {
        'id': 'bringup', 'group': 'system', 'kind': 'service',
        'label': '로봇 bringup',
        'desc': 'm0609_rg2_bringup (sodvir/sodreal 과 같은 명령). 전체 정지·비상 정지로는 꺼지지 않음.',
        'params': [], 'build': _bringup,
        'motion': lambda mode: False, 'mode_bound': True, 'keep_on_stop': True,
    },
    {
        'id': 'homeset', 'group': 'system', 'kind': 'process',
        'label': '홈 위치로 이동',
        'desc': f'관절 홈 자세({HOME_J})로 이동한다. 이미 홈이면 움직이지 않음. (homeset)',
        'params': [
            _p('home', '홈 관절 J1..J6 (deg)', 'text', HOME_J),
            _p('vel', '관절 속도 (deg/s)', 'number', HOME_BASE_VEL, scale=True, min_=1.0, max_=60.0),
            _p('acc', '관절 가속도 (deg/s²)', 'number', HOME_BASE_VEL, scale=True, min_=1.0, max_=60.0),
        ],
        'build': _homeset, 'motion': lambda mode: True,
    },
]

TASK_MAP = {t['id']: t for t in TASKS}


def catalog():
    """화면에 보낼 JSON (함수 제외)."""
    keys = ('id', 'group', 'kind', 'label', 'desc', 'params')
    return [{k: t[k] for k in keys} for t in TASKS]


def parse_posx(text):
    vals = [float(v) for v in str(text).replace('[', '').replace(']', '').split(',') if v.strip()]
    if len(vals) != 6:
        raise ValueError('posx 는 x,y,z,a,b,c 6개 값이어야 합니다')
    return vals


def normalize_params(task, raw):
    """화면에서 받은 값을 타입에 맞게 검사/변환한다. 잘못되면 ValueError."""
    raw = raw or {}
    out = {}
    for spec in task['params']:
        key, type_ = spec['key'], spec['type']
        value = raw.get(key, spec['default'])
        if type_ == 'number':
            if value is None or str(value).strip() == '':
                raise ValueError(f'{spec["label"]}: 값을 입력하세요')
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError(f'{spec["label"]}: 숫자가 아닙니다') from None
            if value != value or value in (float('inf'), float('-inf')):
                raise ValueError(f'{spec["label"]}: 숫자가 아닙니다')
            if spec.get('integer'):
                if value != int(value):
                    raise ValueError(f'{spec["label"]}: 정수를 입력하세요 ({value:g})')
                value = int(value)
            if spec.get('min') is not None and value < spec['min']:
                raise ValueError(f'{spec["label"]}: {spec["min"]:g} 이상이어야 합니다 ({value:g})')
            if spec.get('max') is not None and value > spec['max']:
                raise ValueError(f'{spec["label"]}: {spec["max"]:g} 이하여야 합니다 ({value:g})')
        elif type_ == 'bool':
            value = value if isinstance(value, bool) else str(value).lower() in ('1', 'true', 'yes', 'on')
        elif type_ == 'select':
            value = str(value)
            if value not in spec['options']:
                raise ValueError(f'{spec["label"]}: 허용되지 않은 값 {value}')
        else:
            value = str(value).strip()
            if any(c in value for c in '\n\r\0'):
                raise ValueError(f'{spec["label"]}: 줄바꿈은 쓸 수 없습니다')
            if spec.get('maxlen') and len(value) > spec['maxlen']:
                raise ValueError(f'{spec["label"]}: {spec["maxlen"]}자 이하로 입력하세요 ({len(value)}자)')
        out[key] = value
    return out


def apply_speed(task, params, speed_pct):
    """속도 % 가 100 을 넘으면(안전 해제) scale 항목을 배율만큼 키운다.

    100% 이하는 컨트롤러 속도 오버라이드(change_operation_speed)로 줄이므로
    여기서는 건드리지 않는다 (이중으로 느려지는 것 방지).
    """
    boost = max(float(speed_pct), 100.0) / 100.0
    if boost == 1.0:
        return params
    out = dict(params)
    for spec in task['params']:
        if spec.get('scale') and spec['type'] == 'number':
            out[spec['key']] = round(out[spec['key']] * boost, 3)
    return out
