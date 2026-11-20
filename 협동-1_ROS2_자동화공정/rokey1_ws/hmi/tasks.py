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

# 팀 샌딩 서버: 빈 시작/끝은 서버에 등록된 기본 작업을 사용한다.
DEFAULT_START = ''
DEFAULT_END = ''


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
    return _ros_params(execute_motion=True, virtual_gripper=(mode == 'virtual'))


def _pickup_args(mode, p):
    args = ['--execute-motion',
            '--clearance', str(p['clearance']), '--settle', str(p['settle']),
            '--transfer-speed', str(p['transfer_speed']), '--pick-speed', str(p['pick_speed'])]
    if mode == 'virtual':
        args.append('--virtual-gripper')
    return args


def putty_arguments(task, params):
    """HMI 입력을 PuttyTask Goal의 기존 작업 설정 토큰으로 변환한다."""
    names = {
        'initialize': (),
        'pickup': ('clearance', 'settle', 'transfer_speed', 'pick_speed'),
        'scoop': ('scoop_speed',),
        'apply': ('floor_force', 'wall_force', 'press_force', 'apply_speed'),
        'return': ('place_speed',),
    }[task]
    return [token for name in names for token in ('--'+name.replace('_', '-'), str(params[name]))]


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
        'id': 'wall_scan_server', 'group': 'navigation', 'kind': 'service',
        'label': '벽 내부 탐색 서버',
        'desc': '/process/explore 액션 서버(wall_scan_server)를 띄운다. [벽 내부 탐색 실행] 전에 먼저 시작. '
                'Goal 전에는 움직이지 않음.',
        'params': [], 'build': lambda mode, p: [_run('wall_scan_server', *_ros_params(mode=mode))],
        'motion': lambda mode: False,
    },
    {
        'id': 'wall_scan_goal', 'group': 'navigation', 'kind': 'action', 'process_goal': 'EXPLORE',
        'label': '벽 내부 탐색 실행 (탐침, 홈에서 시작) [임시]',
        'desc': '탐침을 쥔 채 홈 자세에서 시작. 벽 테두리 안쪽을 누르며 가로 줄·세로 열로 훑어 표면을 기록하고 '
                '틈/홈/돌출을 찾는다 (Process 액션 /process/explore, task_code=EXPLORE). '
                '탐지 점은 로봇 화면에 실시간 표시 (파랑=접촉, 주황=접촉 끊김, 빨강=틈/홈, 보라=돌출, 노랑 네모=테두리에 걸림, 청록 마름모=틈/단차 추정). '
                '첫 줄에서만 테두리 판정(걸리면 10 mm씩 안쪽으로 옮겨 다시), 그 뒤로 걸린 곳은 '
                '틈/단차 추정으로 저장하고 30 mm 건너뛰어 계속.',
        'params': [
            _p('dirs', '스캔 방향', 'select', 'both', ['both', 'rows', 'cols'],
               help_='both=가로 줄+세로 열(약 13분) / rows=가로만(약 6분, 세로 틈) / cols=세로만(가로 틈)'),
            _p('row_step', '가로 줄 간격 (mm)', 'number', 20.0),
            _p('col_step', '세로 열 간격 (mm)', 'number', 20.0),
            _p('margin', '테두리 여유 (mm)', 'number', 15.0, help_='테두리에서 이만큼 안쪽만 탐색'),
            _p('probe_length', '탐침 길이 (mm)', 'number', 0.0,
               help_='TCP 끝에서 탐침봉이 더 나온 길이. 탐침 TCP 를 따로 등록했으면 0'),
            _p('scan_method', '누르는 방식', 'select', 'force', ['force', 'spring'],
               help_='force=벽에 닿은 뒤 힘제어로 누르며 이동 / spring=순응제어 스프링(목표를 벽 안쪽에)'),
            _p('scan_force', '힘제어 누르는 힘 (N)', 'number', 2.0, help_='접촉 힘에 더해지는 힘(상대). 0.5~5 N'),
            _p('top_force_ratio', '맨 위 줄 힘 비율', 'number', 1.0,
               help_='위로 갈수록 힘이 약해지는 것 보정. 맨 위 줄 = 누르는 힘 × 비율, 사이 줄은 높이에 비례 (1.0 = 보정 없음)'),
            _p('tool_roll', '공구 Z축 회전 (°)', 'number', 90.0,
               help_='탐색 때 공구를 자기 Z축 기준으로 돌림 (탐침 끝 위치는 그대로). 안 되면 반대(−) 방향 자동 시도'),
            _p('tcp_name', 'TCP 이름', 'text', 'GripperDA_v1', help_='펜던트에 등록된 TCP'),
            _p('floor_detect', '바닥 높이 측정', 'bool', True, help_='시작 전에 순응제어로 바닥을 재서 맨 아래 줄을 띄움'),
            _p('go_home', '끝나면 홈 복귀', 'bool', True),
        ],
        'build': None, 'motion': lambda mode: True,
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
        'id': 'putty_task_server', 'group': 'putty', 'kind': 'service',
        'label': '퍼티 액션 서버',
        'desc': '/dsr01/putty_task 서버 시작. 작업 Goal 전에는 움직이지 않습니다. 먼저 시작하세요.',
        'params': [], 'build': lambda mode, p: [_run('putty_task_server')],
        'motion': lambda mode: False,
    },
    {
        'id': 'putty_cycle', 'group': 'putty', 'kind': 'action',
        'putty_tasks': ('initialize', 'pickup'),
        'label': '퍼티 초기화·집기',
        'desc': '[초기화 → 헤라 집기] 를 순서대로 실행. 앞 단계가 실패하면 뒤 단계는 실행하지 않음. '
                '가상 모드에서도 컨트롤러에 이동 요청을 전송합니다.',
        'params': PICKUP_PARAMS,
        'build': None,
        'motion': lambda mode: True,
    },
    {
        'id': 'initialize_device', 'group': 'putty', 'kind': 'action',
        'putty_tasks': ('initialize',),
        'label': '초기화',
        'desc': '홈 자세로 돌아간 뒤 RG2를 닫는다. 가상 모드는 가상 집게 서비스를 사용한다.',
        'params': [],
        'build': None,
        'motion': lambda mode: True,
    },
    {
        'id': 'pickup_spatula', 'group': 'putty', 'kind': 'action',
        'putty_tasks': ('pickup',),
        'label': '헤라 집기',
        'desc': '현재 보관 위치 [308.42,120.70,152.43] 위로 이동 → 헤라 집기 → 상승.',
        'params': PICKUP_PARAMS,
        'build': None,
        'motion': lambda mode: True,
    },
    {
        'id': 'scoop_putty', 'group': 'putty', 'kind': 'action', 'putty_tasks': ('scoop',),
        'label': '퍼티 뜨기', 'desc': '측정 경로로 퍼티 뜨기. 실물에서는 파지 확인이 필요합니다.',
        'params': [_p('tool_held_confirmed', '헤라 파지 확인', 'bool', False),
                   _p('scoop_speed', '스쿱 속도 (mm/s)', 'number', 3.0, scale=True)],
        'build': None, 'motion': lambda mode: True,
    },
    {
        'id': 'apply_putty_plane', 'group': 'putty', 'kind': 'action', 'putty_tasks': ('apply',),
        'label': '벽면 퍼티 도포',
        'desc': 'YZ 벽 탐색→6 N 실측 확인→상향 도포. 현재 접촉 자세의 특이점 문제는 미해결입니다.',
        'params': [_p('floor_force', '바닥 감지 변화량 (N)', 'number', 2.3),
                   _p('wall_force', '벽 감지 변화량 (N)', 'number', 2.3),
                   _p('press_force', '도포 목표 힘 (N)', 'number', 6.0),
                   _p('apply_speed', '도포 속도 (mm/s)', 'number', 10.0, scale=True)],
        'build': None, 'motion': lambda mode: True,
    },
    {
        'id': 'return_spatula', 'group': 'putty', 'kind': 'action', 'putty_tasks': ('return',),
        'label': '헤라 반납 (가상 전용)', 'desc': '보관 위치로 헤라 반납. 홈 이동은 초기화를 별도 요청합니다.',
        'params': [_p('place_speed', '반납 속도 (mm/s)', 'number', 10.0, scale=True)],
        'build': None, 'motion': lambda mode: True,
    },

    # ===== SANDING =====
    {
        'id': 'sanding_server', 'group': 'sanding', 'kind': 'service',
        'label': '샌딩 액션 서버',
        'desc': '/dsr01/tool_task 액션 서버(sanding_server)를 띄운다. '
                '[샌딩 작업 실행] 전에 먼저 시작해 두어야 함 (RG2 /onrobot 필요).',
        'params': [
            _p('retreat_on_cancel', '정지(STOP) 시 벽에서 후퇴', 'bool', False,
               help_='끄면 STOP/E-STOP 때 그 자리에서 멈춤 (E-STOP 도 같은 취소로 오므로 기본 끔)'),
            _p('go_home', '작업 전 항상 홈 자세로', 'bool', False,
               help_='끄면 현재 팔 형태로 도구에 못 갈 때만 홈 [0,0,90,0,90,0] 으로 먼저 이동'),
            _p('floor_z', '벽 아래 바닥 높이 z (mm)', 'number', 35.0,
               help_='베이스 기준 바닥 높이. 바닥 근처(바닥+반지름+여유 아래) 시작/종료 z 는 전부 바닥에 붙여서 작업'),
            _p('tool_radius', '원형 도구 반지름 (mm)', 'number', 45.0,
               help_='도구 중앙을 잡으므로 벽 작업 때 도구 아래끝 = TCP - 반지름'),
            _p('floor_clear', '바닥 여유 (mm)', 'number', 1.0,
               help_='도구 아래끝과 바닥 사이 간격. 맨 아래가 덜 갈리면 줄이고(0~1), 바닥에 걸리면 늘리세요'),
        ],
        'build': lambda mode, p: [_run('sanding_server', *_ros_params(retreat_on_cancel=p['retreat_on_cancel'],
                                                                      go_home=p['go_home'],
                                                                      floor_z=p['floor_z'],
                                                                      tool_radius=p['tool_radius'],
                                                                      floor_clear=p['floor_clear']))],
        'motion': lambda mode: False,
    },
    {
        'id': 'sanding_goal', 'group': 'sanding', 'kind': 'action',
        'label': '샌딩 작업 실행',
        'desc': '도구 집기 → 벽에 90도로 접촉 → 순응/힘제어 샌딩 (시작/종료 왕복 + 중간 원) → 도구 반납. '
                '시작/종료를 비우면 정해 둔 기본 작업.',
        'params': [
            _p('tool_id', '도구 번호', 'select', '1', ['1', '2', '3'], help_='거치대에서 집을 도구 번호'),
            _p('start_posx', '시작 posx x,y,z,a,b,c', 'text', DEFAULT_START,
               help_='비우면 기본 작업 (정해 둔 시작 관절각에서 +Y 10cm). '
                     '넣으면: TCP 가 벽(또는 바닥)을 향하게 교시한 값, 툴 Z 로 벽 방향 자동 판단해 90도 보정'),
            _p('end_posx', '종료 posx x,y,z,a,b,c', 'text', DEFAULT_END,
               help_='종료 위치 (BASE 기준, 자세는 시작 자세 사용). 시작을 비우면 무시'),
            _p('return_tool', '작업 후 도구 반납', 'bool', True, help_='끝나면 도구를 거치대에 다시 놓음'),
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
            if key.endswith('_posx') and value:
                parse_posx(value)   # 시작 전에 형식 검사 (비우면 샌딩 기본 작업)
        out[key] = value
    if 'start_posx' in out and out['start_posx'] and not out.get('end_posx'):
        raise ValueError('종료 posx 를 넣어 주세요 (시작/종료 둘 다 비우면 기본 작업)')
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
