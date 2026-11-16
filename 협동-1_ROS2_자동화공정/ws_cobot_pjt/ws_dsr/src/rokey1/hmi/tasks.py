"""HMI 에서 실행할 수 있는 작업(navigation / putty_demo / sanding) 카탈로그.

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


def _p(key, label, type_, default, options=None, help_=''):
    item = {'key': key, 'label': label, 'type': type_, 'default': default, 'help': help_}
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
    _p('clearance', '접근 거리 (mm)', 'number', 50.0),
    _p('settle', '집게 대기 (s)', 'number', 1.0),
    _p('transfer_speed', '이송 속도 (mm/s)', 'number', 15.0),
    _p('pick_speed', '집기 속도 (mm/s)', 'number', 5.0),
]


# ----------------------------------------------------------------------------- 카탈로그
TASKS = [
    # ===== NAVIGATION =====
    {
        'id': 'wall_size', 'group': 'navigation', 'kind': 'process',
        'label': '벽 크기 측정', 'desc': '벽을 더듬어 높이/너비 측정 (wall_size)',
        'params': [
            _p('scan', '스캔 방식', 'select', 'both', ['both', 'width', 'height', 'line_v', 'line_h', 'lines']),
            _p('face', '벽 표면 추정 (mm)', 'number', 500.0),
            _p('center', '옆 중심 (mm)', 'number', 0.0),
            _p('lat_min', '옆 최소 (mm)', 'number', -320.0),
            _p('lat_max', '옆 최대 (mm)', 'number', 320.0),
            _p('z_limit', '높이 한계 (mm)', 'number', 450.0),
            _p('zs', '너비 측정 높이들', 'text', '50,150,250'),
            _p('xs', '높이 측정 위치들', 'text', ''),
            _p('step', '스윕 간격 (mm)', 'number', 10.0),
            _p('inject', '사고 주입 시험 (가상)', 'select', 'none',
               ['none', 'tool_side', 'tool_overload', 'tool_torque', 'arm', 'estop', 'sensor']),
            _p('rviz', 'RViz 마커', 'bool', True),
        ],
        'build': _wall_size, 'motion': lambda mode: True,
    },
    {
        'id': 'wall_navigation', 'group': 'navigation', 'kind': 'process',
        'label': '벽면 스캔', 'desc': '벽면 프로빙 + 슬라이딩 스캔 → 모델링 (wall_navigation)',
        'params': [
            _p('modeling', '끝나고 모델링', 'bool', True),
            _p('show_plot', '그림 창 띄우기', 'bool', False),
            _p('rviz', 'RViz 마커', 'bool', True),
        ],
        'build': _wall_navigation, 'motion': lambda mode: True,
    },
    {
        'id': 'table_navigation', 'group': 'navigation', 'kind': 'process',
        'label': '상판 스캔', 'desc': '상판 영역 티칭 / 탐색 / 모델링 (table_navigation)',
        'params': [
            _p('task', '작업', 'select', 'explore', ['explore', 'teach', 'model']),
            _p('area', '영역 x_min,x_max,y_min,y_max', 'text', '', help_='비우면 티칭 파일 사용'),
            _p('table_z', '상판 대략 높이 (mm)', 'text', ''),
            _p('speed', '속도 배율', 'number', 1.0),
            _p('dry_run', '닿지 않고 경로만 순회', 'bool', False),
            _p('go_home', '끝나고 홈 복귀', 'bool', False),
            _p('show_plot', '그림 창 띄우기', 'bool', False),
            _p('rviz', 'RViz 마커', 'bool', True),
        ],
        'build': _table_navigation, 'motion': lambda mode: True,
    },
    {
        'id': 'wall_modeling', 'group': 'navigation', 'kind': 'process',
        'label': '벽 모델링', 'desc': '최근 벽 탐색 데이터로 모델링만 다시 (로봇 불필요)',
        'params': [
            _p('show_plot', '그림 창 띄우기', 'bool', False),
            _p('rviz', 'RViz 표시 (정지할 때까지)', 'bool', False),
        ],
        'build': _wall_modeling, 'motion': lambda mode: False,
    },
    {
        'id': 'wall_size_rviz', 'group': 'navigation', 'kind': 'service',
        'label': '측정 결과 다시 보기', 'desc': '최근 size_*.json 결과를 RViz 에 다시 표시',
        'params': [],
        'build': lambda mode, p: [_run('wall_size_rviz')], 'motion': lambda mode: False,
    },
    {
        'id': 'navigation_rviz_sim', 'group': 'navigation', 'kind': 'service',
        'label': 'RViz 시뮬레이션', 'desc': '탐색/틈 감지 시뮬레이션 마커 (rviz_sim_node)',
        'params': [],
        'build': lambda mode, p: [_run('navigation_rviz_sim')], 'motion': lambda mode: False,
    },

    # ===== PUTTY =====
    {
        'id': 'putty_cycle', 'group': 'putty', 'kind': 'process',
        'label': '퍼티 전체 순서', 'desc': '초기화 → 헤라 집기 를 순서대로 실행 (실패 시 중단)',
        'params': PICKUP_PARAMS,
        'build': lambda mode, p: [_run('initialize_device', *_initialize_args(mode, p)),
                                  _run('pickup_spatula', *_pickup_args(mode, p))],
        'motion': lambda mode: mode == 'real',
    },
    {
        'id': 'initialize_device', 'group': 'putty', 'kind': 'process',
        'label': '초기화', 'desc': '홈 자세 복귀 후 빈 RG2 닫기 (initialize_device)',
        'params': [
            _p('scenario', '가상 시나리오', 'select', 'away',
               ['away', 'already_home', 'joint_failure', 'move_failure', 'home_not_reached', 'io_failure']),
        ],
        'build': lambda mode, p: [_run('initialize_device', *_initialize_args(mode, p))],
        'motion': lambda mode: mode == 'real',
    },
    {
        'id': 'pickup_spatula', 'group': 'putty', 'kind': 'process',
        'label': '헤라 집기', 'desc': '7번 위치 헤라를 RG2 로 집어 들어올리기',
        'params': PICKUP_PARAMS,
        'build': lambda mode, p: [_run('pickup_spatula', *_pickup_args(mode, p))],
        'motion': lambda mode: mode == 'real',
    },

    # ===== SANDING =====
    {
        'id': 'tool_task_server', 'group': 'sanding', 'kind': 'service',
        'label': '샌딩 액션 서버', 'desc': '/dsr01/tool_task 액션 서버 (RG2 /onrobot 필요)',
        'params': [],
        'build': lambda mode, p: [_run('tool_task_server')], 'motion': lambda mode: False,
    },
    {
        'id': 'sanding_goal', 'group': 'sanding', 'kind': 'action',
        'label': '샌딩 작업 실행', 'desc': '도구 집기 → 시작→종료 직선 작업 (ToolTask goal 전송)',
        'params': [
            _p('tool_id', '도구 번호', 'select', '1', ['1', '2', '3']),
            _p('start_posx', '시작 posx x,y,z,a,b,c', 'text', DEFAULT_START),
            _p('end_posx', '종료 posx x,y,z,a,b,c', 'text', DEFAULT_END),
            _p('return_tool', '작업 후 도구 반납', 'bool', False),
        ],
        'build': None, 'motion': lambda mode: True,
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
