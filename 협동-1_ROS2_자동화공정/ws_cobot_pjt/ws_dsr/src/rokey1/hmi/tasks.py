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


def _p(key, label, type_, default, options=None, help_='', scale=False):
    """scale=True 이면 HMI 속도가 100% 를 넘을 때(안전 해제) 그 배율만큼 값을 키운다."""
    item = {'key': key, 'label': label, 'type': type_, 'default': default, 'help': help_, 'scale': scale}
    if options is not None:
        item['options'] = list(options)
    return item


# ----------------------------------------------------------------------------- navigation
def _wall_size(mode, p):
    return [_run('wall_size', *_ros_params(
        mode=mode, scan=p['scan'], face=p['face'], center=p['center'],
        lat_min=p['lat_min'], lat_max=p['lat_max'], z_limit=p['z_limit'],
        zs=p['zs'], xs=p['xs'], step=p['step'], inject=p['inject'], rviz=p['rviz']))]


def _wall_navigation(mode, p):
    return [_run('wall_navigation', *_ros_params(
        mode=mode, modeling=p['modeling'], show_plot=p['show_plot'], rviz=p['rviz']))]


def _table_navigation(mode, p):
    return [_run('table_navigation', *_ros_params(
        task=p['task'], mode=mode, area=p['area'], table_z=p['table_z'], speed=p['speed'],
        dry_run=p['dry_run'], go_home=p['go_home'], show_plot=p['show_plot'], rviz=p['rviz']))]


def _wall_modeling(mode, p):
    args = ['--no-show'] if not p['show_plot'] else []
    if p['rviz']:
        args.append('--rviz')
    return [_run('wall_modeling', *args)]


# ----------------------------------------------------------------------------- putty_demo
def _initialize_args(mode, p):
    if mode == 'real':
        return _ros_params(execute_motion=True)
    return ['--dry-run', '--scenario', p.get('scenario', 'away')]


def _pickup_args(mode, p):
    return ['--execute-motion' if mode == 'real' else '--dry-run',
            '--clearance', str(p['clearance']), '--settle', str(p['settle']),
            '--transfer-speed', str(p['transfer_speed']), '--pick-speed', str(p['pick_speed'])]


PICKUP_PARAMS = [
    _p('clearance', '접근 거리 (mm)', 'number', 50.0,
       help_='헤라 바로 위에서 멈추는 높이. 클수록 안전하지만 시간이 더 걸림'),
    _p('settle', '집게 대기 (s)', 'number', 1.0,
       help_='RG2 를 닫은 뒤 다음 동작까지 기다리는 시간'),
    _p('transfer_speed', '이송 속도 (mm/s)', 'number', 15.0, scale=True,
       help_='헤라 위까지 크게 이동할 때 속도 (속도 % 가 100 을 넘으면 비례 증가)'),
    _p('pick_speed', '집기 속도 (mm/s)', 'number', 5.0, scale=True,
       help_='집기 높이로 내려가고 들어 올릴 때 속도 (느릴수록 정확)'),
]


# ----------------------------------------------------------------------------- 홈 이동
HOME_J = '0,0,90,0,90,0'
HOME_BASE_VEL = 30.0     # 속도 100% 기준 관절 속도 [deg/s]


def _homeset(mode, p):
    return [_run('homeset', *_ros_params(
        mode=mode, home=p['home'], vel=p['vel'], acc=p['acc']))]


# ----------------------------------------------------------------------------- 카탈로그
TASKS = [
    # ===== NAVIGATION =====
    {
        'id': 'wall_size', 'group': 'navigation', 'kind': 'process',
        'label': '벽 크기 측정',
        'desc': '로봇 끝(TCP)으로 벽을 살짝 더듬어 벽의 높이·너비를 잰다. 결과는 size_*.json 으로 저장. '
                '(wall_size)',
        'params': [
            _p('scan', '스캔 방식', 'select', 'both', ['both', 'width', 'height', 'line_v', 'line_h', 'lines'],
               help_='both=너비+높이 / width=너비만 / height=높이만 / line_v·line_h=한 줄 / lines=여러 줄'),
            _p('face', '벽 표면 추정 (mm)', 'number', 500.0,
               help_='로봇 베이스에서 벽까지 대략 거리(X). 이 근처부터 천천히 접근'),
            _p('center', '옆 중심 (mm)', 'number', 0.0, help_='벽 가운데의 Y 좌표'),
            _p('lat_min', '옆 최소 (mm)', 'number', -320.0, help_='너비 스윕을 시작하는 Y 최소값'),
            _p('lat_max', '옆 최대 (mm)', 'number', 320.0, help_='너비 스윕을 끝내는 Y 최대값'),
            _p('z_limit', '높이 한계 (mm)', 'number', 450.0, help_='이 높이(Z) 이상은 올라가지 않음'),
            _p('zs', '너비 측정 높이들', 'text', '50,150,250', help_='너비를 잴 Z 높이 목록 (쉼표로 구분)'),
            _p('xs', '높이 측정 위치들', 'text', '', help_='높이를 잴 위치 목록. 비우면 자동'),
            _p('step', '스윕 간격 (mm)', 'number', 10.0, help_='한 번에 옮기는 거리. 작을수록 정밀·느림'),
            _p('inject', '사고 주입 시험 (가상)', 'select', 'none',
               ['none', 'tool_side', 'tool_overload', 'tool_torque', 'arm', 'estop', 'sensor'],
               help_='가상 모드에서 일부러 사고를 일으켜 안전 처리를 시험. 평소엔 none'),
            _p('rviz', 'RViz 마커', 'bool', True, help_='측정 과정을 RViz 에 표시'),
        ],
        'build': _wall_size, 'motion': lambda mode: True,
    },
    {
        'id': 'wall_navigation', 'group': 'navigation', 'kind': 'process',
        'label': '벽면 스캔',
        'desc': '벽면을 눌러 보며(프로빙) 미끄러지듯 스캔한 뒤 벽 3D 모델을 만든다. (wall_navigation)',
        'params': [
            _p('modeling', '끝나고 모델링', 'bool', True, help_='스캔이 끝나면 바로 벽 모델링까지 실행'),
            _p('show_plot', '그림 창 띄우기', 'bool', False, help_='matplotlib 그래프 창 표시 (창을 닫아야 진행)'),
            _p('rviz', 'RViz 마커', 'bool', True, help_='스캔 점을 RViz 에 표시'),
        ],
        'build': _wall_navigation, 'motion': lambda mode: True,
    },
    {
        'id': 'table_navigation', 'group': 'navigation', 'kind': 'process',
        'label': '상판 스캔',
        'desc': '테이블 상판 영역을 티칭하거나(teach), 탐색해서(explore) 높이 지도를 만든다(model). '
                '(table_navigation)',
        'params': [
            _p('task', '작업', 'select', 'explore', ['explore', 'teach', 'model'],
               help_='teach=영역 꼭짓점 기록 / explore=영역 탐색 / model=저장된 데이터로 모델링'),
            _p('area', '영역 x_min,x_max,y_min,y_max', 'text', '', help_='비우면 티칭 파일 사용'),
            _p('table_z', '상판 대략 높이 (mm)', 'text', '', help_='비우면 자동으로 찾음'),
            _p('speed', '속도 배율', 'number', 1.0,
               help_='노드 자체 속도 배율 (0.05~1.0). 상단 속도 % 와 곱해짐'),
            _p('dry_run', '닿지 않고 경로만 순회', 'bool', False, help_='벽/상판에 닿지 않고 이동 경로만 확인'),
            _p('go_home', '끝나고 홈 복귀', 'bool', False, help_='작업이 끝나면 홈 자세로 돌아감'),
            _p('show_plot', '그림 창 띄우기', 'bool', False, help_='matplotlib 그래프 창 표시'),
            _p('rviz', 'RViz 마커', 'bool', True, help_='탐색 점을 RViz 에 표시'),
        ],
        'build': _table_navigation, 'motion': lambda mode: True,
    },
    {
        'id': 'wall_modeling', 'group': 'navigation', 'kind': 'process',
        'label': '벽 모델링',
        'desc': '최근 벽 탐색 데이터로 모델링만 다시 한다. 로봇은 움직이지 않음.',
        'params': [
            _p('show_plot', '그림 창 띄우기', 'bool', False, help_='결과 그래프 창 표시'),
            _p('rviz', 'RViz 표시 (정지할 때까지)', 'bool', False, help_='정지 버튼을 누를 때까지 RViz 에 계속 표시'),
        ],
        'build': _wall_modeling, 'motion': lambda mode: False,
    },
    {
        'id': 'wall_size_rviz', 'group': 'navigation', 'kind': 'service',
        'label': '측정 결과 다시 보기',
        'desc': '가장 최근 size_*.json 측정 결과를 RViz 에 다시 표시한다. 정지할 때까지 계속 실행.',
        'params': [],
        'build': lambda mode, p: [_run('wall_size_rviz')], 'motion': lambda mode: False,
    },
    {
        'id': 'navigation_rviz_sim', 'group': 'navigation', 'kind': 'service',
        'label': 'RViz 시뮬레이션',
        'desc': '탐색 / 틈 감지 과정을 RViz 마커로 시뮬레이션한다. 로봇은 움직이지 않음.',
        'params': [],
        'build': lambda mode, p: [_run('navigation_rviz_sim')], 'motion': lambda mode: False,
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
        'motion': lambda mode: mode == 'real',
    },
    {
        'id': 'initialize_device', 'group': 'putty', 'kind': 'process',
        'label': '초기화',
        'desc': '홈 자세로 돌아간 뒤 빈 RG2 그리퍼를 닫는다. 가상 모드는 시나리오별 응답으로 점검.',
        'params': [
            _p('scenario', '가상 시나리오', 'select', 'away',
               ['away', 'already_home', 'joint_failure', 'move_failure', 'home_not_reached', 'io_failure'],
               help_='가상 모드 전용. away=정상 / 나머지=각 실패 상황 재현'),
        ],
        'build': lambda mode, p: [_run('initialize_device', *_initialize_args(mode, p))],
        'motion': lambda mode: mode == 'real',
    },
    {
        'id': 'pickup_spatula', 'group': 'putty', 'kind': 'process',
        'label': '헤라 집기',
        'desc': '7번 위치의 헤라 위로 이동 → 내려가서 RG2 로 집기 → 들어 올리기.',
        'params': PICKUP_PARAMS,
        'build': lambda mode, p: [_run('pickup_spatula', *_pickup_args(mode, p))],
        'motion': lambda mode: mode == 'real',
    },

    # ===== SANDING =====
    {
        'id': 'tool_task_server', 'group': 'sanding', 'kind': 'service',
        'label': '샌딩 액션 서버',
        'desc': '/dsr01/tool_task 액션 서버를 띄운다. [샌딩 작업 실행] 전에 먼저 시작해 두어야 함 '
                '(RG2 /onrobot 필요).',
        'params': [],
        'build': lambda mode, p: [_run('tool_task_server')], 'motion': lambda mode: False,
    },
    {
        'id': 'sanding_goal', 'group': 'sanding', 'kind': 'action',
        'label': '샌딩 작업 실행',
        'desc': '도구 거치대에서 도구를 집고 → 시작점에서 종료점까지 직선으로 샌딩한다 (ToolTask goal 전송).',
        'params': [
            _p('tool_id', '도구 번호', 'select', '1', ['1', '2', '3'], help_='거치대에서 집을 도구 번호'),
            _p('start_posx', '시작 posx x,y,z,a,b,c', 'text', DEFAULT_START,
               help_='작업 시작 위치 (BASE 기준, mm / deg)'),
            _p('end_posx', '종료 posx x,y,z,a,b,c', 'text', DEFAULT_END,
               help_='작업 종료 위치 (BASE 기준, mm / deg)'),
            _p('return_tool', '작업 후 도구 반납', 'bool', False, help_='끝나면 도구를 거치대에 다시 놓음'),
        ],
        'build': None, 'motion': lambda mode: True,
    },

    # ===== SYSTEM (탭에는 안 보이고 [홈 위치로 이동] 버튼이 사용) =====
    {
        'id': 'homeset', 'group': 'system', 'kind': 'process',
        'label': '홈 위치로 이동',
        'desc': f'관절 홈 자세({HOME_J})로 이동한다. 이미 홈이면 움직이지 않음. (homeset)',
        'params': [
            _p('home', '홈 관절 J1..J6 (deg)', 'text', HOME_J),
            _p('vel', '관절 속도 (deg/s)', 'number', HOME_BASE_VEL, scale=True),
            _p('acc', '관절 가속도 (deg/s²)', 'number', HOME_BASE_VEL, scale=True),
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
            value = float(value)
            if value != value or value in (float('inf'), float('-inf')):
                raise ValueError(f'{spec["label"]}: 숫자가 아닙니다')
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
