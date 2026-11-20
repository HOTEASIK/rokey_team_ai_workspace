"""퍼티 액션 인자를 기존 노드 설정으로 변환한다. 로봇 통신이나 이동은 수행하지 않는다."""

import math
import sys

from .action_worker import MODULES
from utils.wall import wall_normal

# 퍼티 도포는 BASE +X 벽(법선 −X)만 지원. 벽 4점으로 계산한 법선이 −X 에서 이 각도 넘게 벗어나면 거부
WALL_TILT_MAX_DEG = 5.0


# 임의 실행 파일/ROS 옵션 주입 없이, 작업별 설정만 허용한다.
COMMON = {'--transfer-speed': 1, '--transfer-z': 1}
OPTIONS = {
    'initialize': {},
    'pickup': {**COMMON, '--clearance': 1, '--settle': 1, '--pick-speed': 1,
               '--joint-speed': 1, '--joint-acceleration': 1},
    'scoop': {**COMMON, '--taught-pose': 6, '--lift': 1, '--entry-height': 1,
              '--scoop-angle': 1, '--entry-angle': 1,
              '--scoop-speed': 1, '--acceleration': 1, '--edge-offset': 3,
              '--scoop-pass': 1, '--wall-scrape-height': 1,
              '--wall-contact-force': 1, '--wall-stall-mm': 1, '--wall-stall-sec': 1,
              '--scoop-pass-step': 1, '--wall-search-margin': 1, '--wall-search-speed': 1,
              '--wall-push-force': 1, '--wall-contact-confirm': 1, '--scrape-stiffness': 6},
    'apply': {**COMMON, '--start': 3, '--end': 3, '--stroke-height': 1,
              '--blade-angle': 1, '--blade-end-angle': 1, '--samples': 1,
              '--approach-distance': 1, '--contact-search': 0,
              '--floor-force': 1, '--wall-force': 1, '--search-distance': 1,
              '--search-speed': 1, '--press-force': 1, '--force-tolerance': 1,
              '--force-wait-timeout': 1, '--force-stable-time': 1,
              '--force-loss-timeout': 1, '--apply-speed': 1,
              '--acceleration': 1, '--edge-offset': 3, '--plane': 1,
              '--normal-sign': 1, '--force-mode': 1, '--force-stiffness': 6,
              # 2026-11-18 인수인계본의 벽 접근·누르기 옵션. 팀 Process 경로가 YAML에서 이 옵션들을
              # 만들어 넘기므로 허용 목록에도 있어야 요청이 거부되지 않는다.
              '--wall-approach-z': 1, '--approach-clearance-z': 1, '--wall-backoff': 1,
              '--floor-lift': 1, '--floor-below-gap': 1, '--contact-confirm': 1, '--press-method': 1,
              '--press-stiffness': 1, '--press-tolerance': 1, '--force-limit': 1,
              '--joint-speed': 1, '--joint-acceleration': 1},
    'return': {**COMMON, '--place-speed': 1, '--settle': 1,
               '--joint-speed': 1, '--joint-acceleration': 1,
               '--contact-force': 1, '--search-speed': 1, '--contact-confirm': 1, '--seat-tolerance': 1},
}
# pickup에는 transfer-z CLI 옵션이 없다.
OPTIONS['pickup'].pop('--transfer-z')
CHOICES = {'--plane': ('xy', 'yz', 'xz'),
           '--normal-sign': ('positive', 'negative'), '--force-mode': ('absolute', 'relative'),
           '--press-method': ('direct', 'hybrid', 'position', 'force')}


def worker_command(goal):
    """Action Goal을 검증하고 shell=False용 argv를 돌려준다."""
    if goal.task not in MODULES:
        raise ValueError('지원 작업: initialize, pickup, scoop, apply, return')
    # 반납은 2026-11-19부터 실물도 허용한다. 실물은 바닥 접촉을 힘으로 감지해 내려놓는다(return_spatula).
    if goal.task == 'scoop' and not goal.virtual_mode and not goal.tool_held_confirmed:
        raise ValueError('실물 퍼티 뜨기는 tool_held_confirmed=true 파지 확인이 필요합니다.')
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


# 전체 공정(HMI)의 퍼티 단계 → 워커 작업. 순서: 집기 1회 → 틈마다 (뜨기 → 도포) → 반납 1회 (hmi/process_runner.py)
PROCESS_TASKS = {'PUTTY_PICKUP': 'pickup', 'PUTTY_SCOOP': 'scoop', 'PUTTY': 'apply', 'PUTTY_RETURN': 'return'}


def _args(cfg, names):
    """robot.yaml 값 → ['--joint-speed', '10.0', ...] (리스트 값은 펼침: taught_pose 등)"""
    out = []
    for name in names:
        if cfg.get(name) is None:
            raise ValueError(f'E_FORCE: robot.yaml {name} 설정이 필요합니다.')
        value = cfg[name]
        out += ['--' + name.replace('_', '-'),
                *([str(v) for v in value] if isinstance(value, (list, tuple)) else [str(value)])]
    return out


def _optional(cfg, names):
    """robot.yaml 에 있는 것만 (없으면 각 스크립트 기본값)"""
    return _args(cfg, [n for n in names if cfg.get(n) is not None])


# robot.yaml 에 있으면 넘기는 선택 값 (작업별)
OPTIONAL = {
    'PUTTY_PICKUP': ('pick_speed', 'clearance', 'settle'),
    'PUTTY_RETURN': ('contact_force', 'search_speed', 'contact_confirm', 'seat_tolerance'),
    'PUTTY_SCOOP': ('scoop_speed', 'entry_height', 'taught_pose', 'wall_contact_force', 'wall_stall_mm',
                    'wall_stall_sec', 'scoop_pass_step', 'wall_search_margin', 'wall_search_speed',
                    'wall_push_force', 'wall_contact_confirm', 'scrape_stiffness'),
}


def process_putty_goal(request, settings, virtual_mode=False, held=False, scoop_pass=None):
    """팀 Process Goal → 퍼티 워커 goal. held: 이 서버에서 헤라 집기가 성공했고 아직 반납 전인지
    (실물 뜨기의 파지 확인 = '집기 성공이면 잡혀 있다'로 본다, putty_full_cycle.launch.py 와 같은 규칙).
    scoop_pass: 이번 뜨기 회차 (putty_task_server 가 집기 후 뜨기마다 셈). None 이면 robot.yaml scoop_pass."""
    from types import SimpleNamespace
    code = request.task_code
    if code in ('PUTTY_PICKUP', 'PUTTY_RETURN'):
        cfg = settings['putty']['pickup' if code == 'PUTTY_PICKUP' else 'return']
        names = ('joint_speed', 'joint_acceleration') + (('transfer_speed',) if code == 'PUTTY_PICKUP' else ())
        goal = SimpleNamespace(task=PROCESS_TASKS[code], virtual_mode=virtual_mode, tool_held_confirmed=False,
                               arguments=_args(cfg, names) + _optional(cfg, OPTIONAL[code]))
        worker_command(goal)
        return goal
    if code == 'PUTTY_SCOOP':
        cfg = dict(settings['force']['putty_scoop'])
        if scoop_pass is not None:
            cfg['scoop_pass'] = int(scoop_pass)
        goal = SimpleNamespace(task='scoop', virtual_mode=virtual_mode, tool_held_confirmed=held,
                               arguments=_args(cfg, ('scoop_angle', 'scoop_pass', 'wall_scrape_height',
                                                     'transfer_speed'))
                               + _optional(cfg, OPTIONAL['PUTTY_SCOOP'])
                               + ['--transfer-z', str(settings['putty']['holder_transfer_z'])])
        try:
            worker_command(goal)
        except ValueError as exc:
            raise ValueError(f'E_GRIP: {exc} (먼저 /process/putty_pickup 성공 필요)' if '파지' in str(exc)
                             else str(exc)) from exc
        return goal
    if code != 'PUTTY':
        raise ValueError('E_NO_GAP: PUTTY 작업만 지원합니다.')
    start, end = list(request.start_point), list(request.end_point)
    if (len(start) != 3 or len(end) != 3 or not all(map(math.isfinite, start+end)) or
            end[2] <= start[2] or any(abs(start[i]-end[i]) > 1e-6 for i in (0, 1))):
        raise ValueError('E_NO_GAP: 현재 도포는 BASE X/Y 고정 +Z 방향의 세로 홈만 지원합니다.')
    normal = wall_normal(getattr(request, 'wall', []))     # 4점이 없으면(None) 기존처럼 +X 벽으로 봄
    if normal is not None and -normal[0] < math.cos(math.radians(WALL_TILT_MAX_DEG)):
        raise ValueError(f'E_NO_WALL: 현재 도포는 BASE +X 벽만 지원합니다 '
                         f'(벽 법선 {[round(v, 3) for v in normal]}, 허용 {WALL_TILT_MAX_DEG:g}°).')
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
               'floor_lift': 'floor-lift', 'floor_below_gap': 'floor-below-gap', 'contact_confirm': 'contact-confirm',
               'press_method': 'press-method', 'press_stiffness': 'press-stiffness',
               'press_tolerance': 'press-tolerance',
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
    if 'e_no_wall' in text:
        return 'E_NO_WALL'
    for code in ('E_GRIP', 'E_FORCE'):     # 변환 단계에서 붙인 접두어 우선
        if text.startswith(code.lower()):
            return code
    if '특이점' in text or 'singular' in text:
        return 'E_SINGULAR'
    if '정리 확인 실패' in text or '정지 확인 실패' in text:
        return 'E_TIMEOUT'
    if '힘' in text or 'force' in text:
        return 'E_FORCE'
    if '파지' in text or 'grip' in text or '집기' in text:
        return 'E_GRIP'
    return 'E_TIMEOUT'
