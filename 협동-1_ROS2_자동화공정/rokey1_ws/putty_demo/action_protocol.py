"""퍼티 액션 인자를 기존 노드 설정으로 변환한다. 로봇 통신이나 이동은 수행하지 않는다."""

import math
import sys

from .action_worker import MODULES


# 임의 실행 파일/ROS 옵션 주입 없이, 작업별 설정만 허용한다.
COMMON = {'--transfer-speed': 1, '--transfer-z': 1}
OPTIONS = {
    'initialize': {},
    'pickup': {**COMMON, '--clearance': 1, '--settle': 1, '--pick-speed': 1},
    'scoop': {**COMMON, '--taught-pose': 6, '--lift': 1, '--entry-height': 1,
              '--reference-blade-angle': 1, '--scoop-angle': 1, '--entry-angle': 1,
              '--scoop-speed': 1, '--acceleration': 1, '--edge-offset': 3,
              '--path': 1, '--container-center': 3, '--container-size': 3,
              '--blade-width': 1, '--blade-length': 1, '--scoop-depth': 1,
              '--scoop-distance': 1, '--surface-z': 1, '--samples': 1},
    'apply': {**COMMON, '--start': 3, '--end': 3, '--stroke-height': 1,
              '--blade-angle': 1, '--blade-end-angle': 1, '--samples': 1,
              '--approach-distance': 1, '--contact-search': 0,
              '--floor-force': 1, '--wall-force': 1, '--search-distance': 1,
              '--search-speed': 1, '--press-force': 1, '--force-tolerance': 1,
              '--force-wait-timeout': 1, '--force-stable-time': 1,
              '--force-loss-timeout': 1, '--apply-speed': 1,
              '--acceleration': 1, '--edge-offset': 3, '--plane': 1,
              '--normal-sign': 1, '--surface-z': 1, '--force-mode': 1, '--force-stiffness': 6},
    'load': {'--joint-speed': 1, '--joint-acceleration': 1, '--transfer-z': 1, '--transfer-speed': 1, '--contact-pose': 6, '--force-stiffness': 6, **{
        '--'+name: 1 for name in ('approach-height', 'distance', 'search-distance', 'search-speed',
        'contact-force', 'hold-force', 'force-limit', 'force-tolerance', 'force-wait-timeout',
        'force-stable-time', 'force-loss-timeout', 'scrape-speed', 'lift', 'lift-speed', 'acceleration')}},
    'return': {**COMMON, '--place-speed': 1, '--settle': 1,
               '--surface-z': 1, '--edge-offset': 3},
}
# pickup에는 transfer-z CLI 옵션이 없다.
OPTIONS['pickup'].pop('--transfer-z')
CHOICES = {'--path': ('taught', 'geometry'), '--plane': ('xy', 'yz', 'xz'),
           '--normal-sign': ('positive', 'negative'), '--force-mode': ('absolute', 'relative')}


def worker_command(goal):
    """Action Goal을 검증하고 shell=False용 argv를 돌려준다."""
    if goal.task not in MODULES:
        raise ValueError('지원 작업: initialize, pickup, scoop, load, apply, return')
    if goal.task == 'return' and not goal.virtual_mode:
        raise ValueError('현재 헤라 반납은 가상 컨트롤러 전용입니다.')
    if goal.task in ('scoop', 'load') and not goal.virtual_mode and not goal.tool_held_confirmed:
        raise ValueError('실물 퍼티 뜨기/묻히기는 tool_held_confirmed=true 파지 확인이 필요합니다.')
    tokens = list(goal.arguments)
    if len(tokens) > 100:
        raise ValueError('작업 인자가 너무 많습니다.')
    index, seen = 0, set()
    while index < len(tokens):
        option = tokens[index]
        if option not in OPTIONS[goal.task] or option in seen:
            raise ValueError(f'허용하지 않거나 중복된 작업 옵션: {option}')
        seen.add(option)
        count = OPTIONS[goal.task][option]
        values = tokens[index+1:index+1+count]
        if option in CHOICES:
            if len(values) != 1 or values[0] not in CHOICES[option]:
                raise ValueError(f'{option}은 {CHOICES[option]} 중 하나여야 합니다.')
        elif len(values) != count or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f'{option}의 유한한 숫자 {count}개가 필요합니다.')
        index += count+1
    if goal.task == 'initialize':
        args = ['--ros-args', '-p', 'execute_motion:=true', '-p',
                f'virtual_gripper:={str(goal.virtual_mode).lower()}']
    else:
        args = ['--execute-motion', *tokens]
        if goal.task == 'pickup' and goal.virtual_mode:
            args.append('--virtual-gripper')
        if goal.task == 'load':
            from .load_putty import make_parser, validate
            try:
                validate(make_parser().parse_args(args))
            except SystemExit as exc:
                raise ValueError('판 묻히기 필수 설정값이 필요합니다.') from exc
        if goal.task == 'apply':
            # 현재 실물 벽 도포의 탐색/힘 검증을 액션에서도 생략하지 않는다.
            if '--contact-search' not in tokens and (not goal.virtual_mode or
                    '--floor-force' in tokens or '--wall-force' in tokens):
                args.append('--contact-search')
            from .apply_putty_plane import make_plane_parser, resolve_plane_args, validate
            from .contact_search import validate_search
            try:
                parsed = make_plane_parser().parse_args(args)
                resolve_plane_args(parsed)
                validate(parsed)
                validate_search(parsed)
                if parsed.contact_search:
                    from .contact_search import press_force_request
                    press_force_request(parsed.press_force,
                                        mode=0 if parsed.force_mode == 'absolute' else 1)
            except SystemExit as exc:
                raise ValueError('도포 인자 형식이 잘못됐습니다.') from exc
    return [sys.executable, '-u', '-m', 'putty_demo.action_worker', goal.task, *args]


def process_putty_goal(request, settings, virtual_mode=False):
    """팀 Process Goal의 홈 좌표를 현장 도포 작업에 전달한다. 준비/반납 동작은 추가하지 않는다."""
    from types import SimpleNamespace
    if request.task_code == 'PUTTY_LOAD':
        return process_load_goal(request, settings, virtual_mode)
    if request.task_code != 'PUTTY':
        raise ValueError('E_NO_GAP: PUTTY 작업만 지원합니다.')
    start, end = list(request.start_point), list(request.end_point)
    if (len(start) != 3 or len(end) != 3 or not all(map(math.isfinite, start+end)) or
            end[2] <= start[2] or any(abs(start[i]-end[i]) > 1e-6 for i in (0, 1))):
        raise ValueError('E_NO_GAP: 현재 도포는 BASE X/Y 고정 +Z 방향의 세로 홈만 지원합니다.')
    cfg = settings['force']['putty_scrape']
    target = list(cfg['target'])
    if (cfg.get('ref', 0) != 0 or list(cfg['direction']) != [1, 0, 0, 0, 0, 0] or
            len(target) != 6 or not all(map(math.isfinite, target)) or target[0] <= 0 or
            any(abs(v) > 1e-9 for v in target[1:]) or cfg.get('mod', 1) not in (0, 1)):
        raise ValueError('E_FORCE: 현재 벽 도포는 BASE +X 단일 축 힘 제어 설정만 지원합니다.')
    options = ['--start', *map(str, start), '--end', *map(str, end),
               '--contact-search', '--press-force', str(target[0]),
               '--force-mode', 'absolute' if cfg.get('mod', 1) == 0 else 'relative',
               '--force-stiffness', *map(str, cfg['stiffness']),
               '--force-tolerance', str(cfg['tol']), '--force-wait-timeout', str(cfg['wait_sec'])]
    mapping = {'floor_force': 'floor-force', 'wall_force': 'wall-force',
               'search_distance': 'search-distance', 'search_speed': 'search-speed',
               'stable_sec': 'force-stable-time', 'loss_sec': 'force-loss-timeout',
               'transfer_speed': 'transfer-speed', 'apply_speed': 'apply-speed',
               'acceleration': 'acceleration', 'transfer_z': 'transfer-z',
               'blade_angle': 'blade-angle', 'blade_end_angle': 'blade-end-angle',
               'approach_distance': 'approach-distance', 'samples': 'samples'}
    for key, option in mapping.items():
        if key in cfg:
            options.extend(['--'+option, str(cfg[key])])
    if 'edge_offset' in cfg:
        options.extend(['--edge-offset', *map(str, cfg['edge_offset'])])
    goal = SimpleNamespace(task='apply', virtual_mode=virtual_mode,
                           tool_held_confirmed=False, arguments=options)
    try:
        worker_command(goal)
    except (ValueError, TypeError) as exc:
        raise ValueError(f'E_FORCE: {exc}') from exc
    return goal


def process_error_code(message):
    text = message.lower()
    if 'e_no_gap' in text:
        return 'E_NO_GAP'
    if '특이점' in text or 'singular' in text:
        return 'E_SINGULAR'
    if '정리 확인 실패' in text or '정지 확인 실패' in text:
        return 'E_TIMEOUT'
    if '힘' in text or 'force' in text:
        return 'E_FORCE'
    if '파지' in text or 'grip' in text or '집기' in text:
        return 'E_GRIP'
    return 'E_TIMEOUT'


def process_load_goal(request, settings, virtual_mode=False):
    """명시적 판 작업 요청. Goal의 XYZ는 날 접촉 시 TCP 기준이며 벽 도포 Goal과 구분한다."""
    from types import SimpleNamespace
    cfg = settings['force']['putty_load']
    start, end = list(request.start_point), list(request.end_point)
    if (len(start) != 3 or len(end) != 3 or not all(map(math.isfinite, start+end)) or
            any(abs(start[i]-end[i]) > 1e-6 for i in (1, 2))):
        raise ValueError('E_NO_GAP: 판 묻히기는 BASE +X 직선 TCP 시작·끝점이 필요합니다.')
    if not cfg.get('tool_held_confirmed', False):
        raise ValueError('E_GRIP: 팀 요청 전 헤라 파지 수동 확인이 필요합니다.')
    if cfg.get('ref') != 0 or cfg.get('mod') != 0 or cfg.get('direction') != [0,0,1,0,0,0]:
        raise ValueError('E_FORCE: 판 작업은 BASE -Z 절대 힘 모드를 사용합니다.')
    options = ['--contact-pose', *map(str, start+list(cfg['contact_abc'])),
               '--distance', str(end[0]-start[0]), '--force-stiffness', *map(str,cfg['stiffness'])]
    names = ('approach_height', 'search_distance', 'search_speed', 'contact_force', 'hold_force',
             'force_limit', 'force_tolerance', 'force_wait_timeout', 'force_stable_time',
             'force_loss_timeout', 'scrape_speed', 'lift', 'lift_speed', 'acceleration')
    for name in names:
        value = cfg.get(name)
        if value is None:
            raise ValueError(f'E_FORCE: putty_load.{name} 설정이 필요합니다.')
        options.extend(['--'+name.replace('_','-'), str(value)])
    if cfg.get('transfer_z') is not None or cfg.get('transfer_speed') is not None:
        if cfg.get('transfer_z') is None or cfg.get('transfer_speed') is None:
            raise ValueError('자동 이송 설정은 높이/속도가 모두 필요합니다.')
        options.extend(['--transfer-z', str(cfg['transfer_z']), '--transfer-speed', str(cfg['transfer_speed'])])
    for name in ('joint_speed', 'joint_acceleration'):
        if cfg.get(name) is not None:
            options.extend(['--'+name.replace('_','-'), str(cfg[name])])
    goal = SimpleNamespace(task='load', virtual_mode=virtual_mode, tool_held_confirmed=True, arguments=options)
    worker_command(goal)
    return goal
