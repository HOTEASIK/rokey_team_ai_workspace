# 퍼티 데모 코드 상세 해설

2026-11-19 코드 기준입니다. 큰 흐름은 HTML 페이지 「퍼티 데모 코드 지도」에 있고, 이 문서는 **함수 단위로 무엇을 받고, 무엇을 검사하고, 어떤 두산 요청을 보내는지**를 적습니다.
줄 번호(`파일:줄`)는 이 날짜 빌드 기준이라 코드가 바뀌면 달라질 수 있습니다. 설정값 표는 [README.md](README.md), 액션 사용법은 [ACTIONS.md](ACTIONS.md)를 봅니다.

## 목차

0. [공통 구조](#0-공통-구조)
1. [초기화 — initialize_device.py](#1-초기화--initialize_devicepy)
2. [헤라 집기 — pickup_spatula.py](#2-헤라-집기--pickup_spatulapy)
3. [퍼티 뜨기 — scoop_putty.py](#3-퍼티-뜨기--scoop_puttypy)
4. [벽 도포 — apply_putty_plane.py · putty_motion.py · contact_search.py](#4-벽-도포)
5. [헤라 반납 — return_spatula.py](#5-헤라-반납--return_spatulapy)
6. [액션 서버와 전체 공정 launch](#6-액션-서버와-전체-공정-launch)
7. [알람 코드와 대응 위치](#7-알람-코드와-대응-위치)

---

## 0. 공통 구조

### 0.1 한 단계가 실행되는 경로

```text
ros2 run rokey1 <노드> --execute-motion ...           ← 직접 실행
        또는
putty_task_client <task> -- <옵션>                      ← 액션 실행
  → PuttyTask Goal → putty_task_server.execute()
  → action_protocol.worker_command(goal)                옵션 허용 목록 검사 → argv 생성
  → action_process.WorkerProcess(argv)                  자식 프로세스 실행 (scoop 확인 시 stdin에 'y')
  → python -m putty_demo.action_worker <task> ...       해당 노드의 main() 호출
        ↓
<노드>.main() → 인자 검증 → run_robot(args) → dsr_msgs2 서비스 → /dsr01/dsr_controller2
```

노드끼리는 메시지를 주고받지 않습니다. 각 노드는 시작할 때 `get_current_posx` / `get_current_posj`로 **로봇의 현재 상태를 직접 읽고** 그 자리에서 경로를 계산합니다.

### 0.2 서비스 호출 패턴 `call()`

모든 노드의 `run_robot` 안에 같은 모양의 내부 함수가 있습니다.

```python
def call(client, request, step, timeout=5.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():            → TimeoutError  (다음 명령을 보내지 않음)
    if result is None or not result.success: → RuntimeError
    return result
```

- 동기 이동(`sync_type=0`)은 이동이 끝나야 응답하므로 `timeout=120.0`으로 부릅니다.
- `DSR_ROBOT2` 래퍼 대신 서비스를 직접 부르는 이유: 래퍼는 시간 제한 없이 기다려서 취소나 알람 검사를 끼워 넣을 수 없습니다. 예외는 초기화 노드의 실물 DO 출력(`DSR_ROBOT2.set_digital_output`)뿐입니다.
- 서비스 준비는 팀 양식대로 `ServiceWaiter(node).wait(client, ...)`로 합니다.

### 0.3 자주 쓰는 두산 요청 필드

| 서비스 | 필드 | 이 프로젝트에서 쓰는 값 |
|---|---|---|
| `MoveLine` (movel/amovel) | `pos` | BASE TCP XYZABC (mm, deg, ZYZ) |
| | `vel`, `acc` | `[선속도, 10.0]`, `[가속도, 20.0]` (두 번째 값은 회전 속도·가속도) |
| | `ref`, `mode` | 0 = DR_BASE, 0 = 절대 좌표 |
| | `sync_type` | 0 = 동기(끝나면 응답), 1 = 비동기(바로 응답, 접촉 탐색용) |
| `MoveJoint` (movej) | `pos`, `vel`, `acc` | 관절각 deg, deg/s, deg/s² |
| | `mode`, `sync_type` | 0 = 절대 관절각, 0 = 동기 |
| `Ikin` | `pos`, `sol_space`, `ref` | 목표 TCP, 현재 팔 형태(`get_current_solution_space`, 보통 2), 0 |
| `Fkin` | `pos`, `ref` | 관절각 → TCP (이동 없음) |
| `MoveStop` | `stop_mode` | 3 = DR_HOLD |
| `TaskComplianceCtrl` | `stx`, `ref`, `time` | 6축 강성 N/m·Nm/rad, 0, 0.3 s |
| `SetDesiredForce` | `fd`, `dir`, `mod`, `time` | `[±F,0,0,0,0,0]`, `[1,0,0,0,0,0]`(BASE X만), 1 = 상대(REL), 0.3 s |
| `CheckMotion` | 응답 `status` | 0 = IDLE, 1 = 계산 중, 2 = 이동 중 |

### 0.4 좌표·기하 함수 — `putty_geometry.py`

| 함수 | 하는 일 |
|---|---|
| `zyz_axes(abc)` `:35` | 두산 ZYZ 오일러각 → Tool X·Y·Z 축 벡터 3개(BASE 기준). 축 j의 i 성분은 `axes[j][i]` |
| `axes_to_zyz(axes)` `:52` | 축 벡터 3개 → ZYZ 오일러각 |
| `rotate_base_y(vector, deg)` `:45` | 벡터를 BASE Y축으로 회전. 뜨기의 "헤라 기울이기"에 사용 |
| `orientation_error(a, b)` `:67` | 두 자세 사이 회전 각도(deg). 회전행렬로 계산하므로 ZYZ 표현이 달라도 같은 자세면 0 |
| `cross`, `format_values` | 외적, 로그 출력용 문자열 |

**TCP와 날 끝의 관계.** 헤라 날 끝(폭 가운데) = `TCP + Σ axes[j]·offset[j]`입니다. 반대로 날 끝을 고정점으로 두고 자세를 바꾸면 `TCP = 날 끝 − Σ axes[j]·offset[j]`로 TCP를 다시 구합니다. 뜨기와 도포가 모두 이 방식을 씁니다.

### 0.5 움직이기 전 검사 함수 — `pickup_spatula.py`

여러 단계가 import해서 씁니다.

**`check_joint_target(current, target)` `:205`** — 목표 관절이 위험하면 `RuntimeError`.
1. 관절각을 −180~180°로 접습니다(`wrap`). 220°와 −140°는 같은 손목 자세입니다.
2. `|J3| < 41°`면 팔꿈치 특이점, `|J5| < 10°`면 손목 특이점으로 거부합니다.
3. 현재 J5와 목표 J5의 부호가 다르면 손목이 반대로 꺾이는 해로 거부합니다.
4. `wrist_radius(target) < 180 mm`면 어깨 특이점으로 거부합니다(아래).

**`wrist_radius(joints)` (`putty_motion.py`)** — 손목 중심(J5 축)이 베이스 회전축에서 떨어진 수평 거리.
`|411·sin J2 + 368·sin(J2+J3)|` (m0609 위팔 411 mm, 아래팔 368 mm). 홈 자세(J2 0°, J3 90°)에서 368 mm입니다.
이 값이 작으면 J1이 정해지지 않는 어깨 특이점입니다. 뜨기 45°의 사선 시작점이 약 132 mm여서 3205 → 1206이 났습니다.

**`turn_joint_target(current, target, direction, max_turn=180)` `:182`** — J1에 360°를 더하거나 빼도 같은 자세라는 점을 이용해, 위에서 볼 때 원하는 방향으로 도는 J1을 고릅니다.
- `turn = (sign·(target_J1 − current_J1)) mod 360`. `sign`은 ccw면 +1, cw면 −1입니다.
- `turn > max_turn`이면 거의 한 바퀴를 도는 계산 오류로 보고 거부합니다.
- 결과 `J1 = current_J1 + sign·turn`이고, `|J1| > 360°`면 거부합니다.

**`move_to_joints(call, clients, step, joints, speed, acc, info, direction='ccw')` `:264`** — 관절값으로 movej.
1. `GetCurrentPosj`로 현재 관절을 읽습니다.
2. `direction`이 있으면 `turn_joint_target`, `None`이면 받은 값을 그대로 씁니다.
3. `check_joint_target`으로 검사합니다.
4. `MoveJoint(pos, vel, acc, mode=0, sync_type=0)`을 보냅니다.
5. 다시 관절을 읽어 모든 축이 목표 ±1.0°(`JOINT_TOLERANCE_DEG`) 안인지 확인합니다.

**`move_joint_to_pose(..., pose, ..., read_pose, direction)` `:242`** — TCP 목표로 movej.
`GetCurrentSolutionSpace` → `Ikin(pose, sol_space, ref=0)` → `move_to_joints` → `read_pose()`로 TCP가 2 mm / 2° 안인지 확인합니다. 팔 형태(sol_space)를 현재 값으로 고정해야 팔꿈치·손목이 뒤집힌 해가 나오지 않습니다.

**`preflight_poses(call, clients, [(이름, pose), ...])` `:221`** — 경유점 목록을 ikin으로 미리 풀어 `check_joint_target`을 통과하는지 봅니다. 하나라도 실패하면 첫 이동 전에 멈춥니다. 직선 경로의 **중간**은 검사하지 않고 경유점만 봅니다.

**`check_pillar_clearance(points, pillar, clearance=100)` `:158`** — 연속한 두 점을 잇는 선분마다 `segment_distance_xy`로 기둥 중심까지의 최소 XY 거리를 구하고, 100 mm보다 가까우면 `ValueError`. movel 구간에만 의미가 있습니다. movej 경로는 곡선이라 계산할 수 없습니다.

### 0.6 접촉 탐색 — `contact_search.seek_axis` `:405`

```python
seek_axis(io, axis, direction, threshold, distance, speed, confirm=1)
```

`io`는 `read_pose`, `read_force`, `move_async`, `stop_and_wait`를 가진 묶음(SimpleNamespace)입니다. 노드마다 자기 서비스로 만들어 넘깁니다.

1. 시작 TCP와 시작 외력(baseline)을 읽습니다.
2. `axis` 방향으로 `direction·distance`만큼 떨어진 목표로 `amovel`(비동기)을 보냅니다.
3. 0.05 s마다 외력을 읽어 `|F[axis] − baseline[axis]| ≥ threshold`가 `confirm`번 연속이면 `stop_and_wait()` 후 현재 TCP를 돌려줍니다.
4. 실제 이동량이 `distance − 0.1` mm에 닿도록 접촉이 없으면 `RuntimeError`, 시간(`distance/speed·3 + 5` s)이 넘으면 `TimeoutError`.
5. `finally`에서 정지하지 못한 채 끝나면 정지 요청을 한 번 더 보냅니다.

호출 전에 반드시 컴플라이언스를 켭니다. 헤라가 길이 방향으로 단단해서, 위치제어로 닿으면 힘이 순간적으로 치솟아 충돌 감지(7060)가 먼저 걸립니다.

---

## 1. 초기화 — `initialize_device.py`

**목적.** 어떤 자세에서든 홈 관절 `HOME_J = (0, 0, 90, 0, 90, 0)`으로 가고 빈 RG2를 닫습니다.

### 호출 순서

```text
main()                                  :25   rclpy 노드 생성, 예외를 종료 코드로 변환(성공 0, 실패 1, Ctrl+C 130)
└─ initialize(node)                     :129
   ├─ ROS 파라미터 execute_motion 확인     false면 RuntimeError (움직이지 않음)
   ├─ [가상] GetRobotSystem == 1 확인, 닫기는 onrobot SetCommand('c')
   ├─ [실물] DR_init 설정 → import DSR_ROBOT2, 닫기는 DSR_ROBOT2.set_digital_output
   └─ run_initialization(read_joints, move_home, set_output, info, virtual_close)   :116
      ├─ return_home(...)               :95
      │  ├─ checked_joint_positions()   :68   GetCurrentPosj 응답 형식 검사
      │  ├─ at_home()                   :76   6축 모두 ±0.5° 안이면 "이미 홈" → 이동 생략
      │  └─ move_joint(node, client, HOME_J)   :82   MoveJoint vel 10, acc 10, mode 0, sync 0
      │     → 다시 읽어 at_home 확인, 아니면 RuntimeError
      └─ close_gripper(set_output)      :107  DO (1,0)(2,0)(1,1)(2,0) 순서, 하나라도 실패하면 RuntimeError
```

### 읽을 때 주의

- 이 노드만 **ROS 파라미터**(`-p execute_motion:=true`)를 받고, 나머지 노드는 CLI 옵션(`--execute-motion`)을 받습니다. 그래서 `worker_command`도 initialize만 다르게 조립합니다.
- `return_home`은 **어디서든 홈으로 바로 movej**합니다. 로봇이 보관대나 용기 쪽(J1 > 90°)에 있으면 기둥 방향을 곡선으로 지나갈 수 있습니다. 2026-11-19 bringup 직후 이 movej 중에 외력 96 N 충돌(9015)이 기록됐습니다. 고칠 대상입니다.

---

## 2. 헤라 집기 — `pickup_spatula.py`

**목적.** 홈에서 출발해 로봇 뒤쪽 보관대의 헤라를 집고 이송 높이까지 들어 올립니다.

### 호출 순서

```text
main()                                   :72   CLI 파싱(--execute-motion 필수), 속도·여유값이 양수인지 검사
└─ run_robot(args)                       :352
   ├─ 클라이언트 생성 + create_joint_clients()   :327  joints/solution/ikin/joint/fkin 묶음
   ├─ ServiceWaiter.wait(...)
   ├─ ensure_tcp(read_tcp, set_tcp, info) :310  현재 TCP가 GripperDA_v1이 아니면 적용 후 다시 확인
   ├─ GetCurrentPosj → 홈 ±0.5°가 아니면 RuntimeError("홈에서 시작해야 합니다")
   ├─ home_pose = read_pose()
   └─ pickup(home_pose, clearance, settle, move, move_joints, set_output, wait, ...)   :110
```

`pickup()`은 **순서만 정하는 순수 함수**입니다. 실제 이동은 `run_robot`이 넘겨 준 콜백이 합니다.

| 콜백 | 실제 동작 |
|---|---|
| `move(step, pose, speed)` | `MoveLine` 동기 → `read_pose`로 2 mm / 2° 도착 확인 |
| `move_joints(step, joints)` | `move_to_joints(..., direction='ccw')` → 도착 TCP를 읽어 **돌려줌** |
| `set_output(step, ch, val)` | `SetCtrlBoxDigitalOutput` (실물) |
| `gripper_command('o'/'c')` | onrobot `SetCommand` (`--virtual-gripper`일 때) |

덕분에 테스트에서는 콜백을 기록용 람다로 바꿔 경로만 검사합니다(`test/test_pickup_pose.py`).

### `pickup()` 안에서 일어나는 일

1. `clear_z = max(PICK_Z + clearance, home_z, 230)`을 구하고 `check_holder_clearance(clear_z)`로 보관대 깊이 80 + 여유 20 mm 이상인지 확인합니다.
2. **홈 위로 상승**: 홈 XY 그대로 Z만 `clear_z`로 (movel).
3. **경유 관절로 이동**: `move_joints(HOLDER_DETOUR_J)`. 현장에서 홈 → 이 자세 movej가 기둥에 닿지 않는 걸 확인한 값입니다. ikin 없이 관절값 그대로 가고, 반시계로 J1 +121.7°입니다. 도착 TCP `reached`를 받습니다.
4. **이송 높이로 상승**: `(reached XY, clear_z, PICKUP_ABC)`. 경유지 Z 약 205를 230으로 올립니다.
5. `check_pillar_clearance((lift, approach))`: 보관대 위로 가는 직선이 기둥에서 100 mm 이상인지 확인합니다(실제 약 166 mm).
6. **헤라 바로 위로 직선 이동**: `(PICK XY, clear_z, PICKUP_ABC)` (movel).
7. 열기(`OPEN` = (1,0)(2,0)(2,1)) → `settle` 대기 → **집기 높이로 하강**(`PICK_POSE`, 5 mm/s) → 닫기(`CLOSE` = (1,0)(2,0)(1,1)(2,0)) → 대기 → **들어 올리기**(6번 자세로).

### 설계 메모

- `PICK_POSE`는 헤라가 보관대 바닥에 **완전히 앉은** 높이에서 잡는 TCP입니다. 그래서 매번 같은 깊이를 잡고, 뜨기의 날 끝 오프셋이 일정하게 유지됩니다.
- 이송 높이를 245에서 230으로 낮춘 이유: 보관대 위에서 높을수록 팔이 펴져 J3가 작아집니다(245 → 40.4°, 230 → 44.0°).

---

## 3. 퍼티 뜨기 — `scoop_putty.py`

**목적.** 헤라를 기울여 용기 −X 구석까지 사선으로 썰고 들어간 뒤, 벽으로 밀어 퍼티를 모으고 긁어 올립니다.

### 기하 함수 (로봇 없이 계산만)

| 함수 | 입력 → 출력 | 내용 |
|---|---|---|
| `validate_taught(args)` `:102` | args → 예외 | 각도 45~60°, 속도·높이 양수, 회차 1~4 정수, 긁기 높이 0~30 mm 및 `lift` 이하 |
| `tilt_rotation(args)` `:138` | → deg | `90 − scoop_angle`. 측정 자세는 수직(바닥 기준 90°)이라 그만큼 BASE Y로 돌립니다 |
| `container_walls(args)` `:143` | → (구석 점, −X 벽 X, 바닥 Z) | 측정 자세 TCP + R·`SCOOP_EDGE_OFFSET`(0, 0, 147.7) = 날 끝 = 바닥 구석. 벽 X는 날 끝 − 두께/2 |
| `pass_shift(args)` `:128` | → mm | `(scoop_pass − 1) × 20` |
| `scoop_end_edge(args)` `:152` | → 날 끝 XYZ | 구석 점 + (`pass_shift`, 0, 0). 여유 없음(b안) |
| `taught_target(args)` `:163` | → TCP XYZABC | 측정 자세 축들을 `rotate_base_y(axis, tilt)`로 돌리고, `TCP = 끝점 날 끝 − R'·오프셋` |
| `check_scoop_clearance(args)` `:173` | → mm | 헤라 폭 ≤ 용기 Y 폭, 그리고 사선이 +X 벽 위치를 지날 때 바닥 위 높이가 용기 깊이(40) 이상인지 |
| `taught_moves(current, args)` `:190` | → (이름, pose, 속도) × 5 | 아래 1~4단계 경유점 |
| `wall_scrape_poses(args)` `:211` | → (벽, 긁은 점, 상승점) | 벽 = 끝점 TCP − (`pass_shift`, 0, 0), 긁은 점 = 벽 + Z 긁기 높이, 상승점 Z = 끝점 + `lift` |

`taught_moves`가 만드는 경유점은 다음과 같습니다. 모두 movel입니다.

```text
height = max(현재 Z, 사선 시작 Z, 상승점 Z, transfer_z 230)
1 현재 자세로 이송 높이 확보          (현재 XY, height, 현재 ABC)          transfer_speed
2 그 자리에서 헤라를 N도로 기울이기    (현재 XY, height, 기울인 ABC)         transfer_speed
3 기울인 자세로 사선 시작점 위 이송    (시작점 XY, height, 기울인 ABC)       transfer_speed
4 기울인 자세로 사선 시작점 하강      시작점 = 끝점 + (h/tanθ, 0, h), h=entry_height 120
5 각도 유지하며 −X·−Z 사선 진입      끝점(taught_target)                  scoop_speed 3
```

### 실행 `run_robot(args)` `:231`

1. 클라이언트를 만들고 `GetRobotSystem`(0 = 실물, 1 = 가상), TCP를 확인합니다. 실물이면 외력·정지·`check_motion`·컴플라이언스·힘제어 서비스를 추가로 기다립니다.
2. 설정을 로그로 남깁니다(각도, 오프셋, 회차, 끝점 날 끝·TCP, 벽 긁기 설정).
3. 실물이면 `confirm_grasp()` `:220`: `input()`으로 y를 받습니다. 액션으로 실행하면 서버가 stdin에 `y`를 넣어 줍니다.
4. `taught_moves` + `wall_scrape_poses`의 3점까지 합쳐 `preflight_poses`로 **전부 관절 검사**를 합니다.
5. 1~5번 경유점을 `move()`(movel 동기 + 2 mm / 2° 확인)로 실행합니다.
6. **[실물] 벽 긁기**
   - `start_compliance()`: `SCRAPE_STIFFNESS`(X·Z 1000 N/m) 적용 후 0.3 s 대기. 응답 전에 `compliance_active=True`로 표시해서, 실패해도 정리 대상이 됩니다.
   - `seek_axis(io, axis=0, direction=−1, 5 N, pass_shift+10 mm, 2 mm/s, confirm=2)` → 벽(또는 모인 퍼티) 접촉점 `contact`. 계획 벽 X와의 차이를 로그로 남깁니다.
   - 긁기 높이 > 0이면:
     - `start_push_force()`: `press_force_request(3, mode=1, sign=−1)`, 즉 fd = −3 N, BASE X만, 상대 모드.
     - `move(..., (contact XY, contact Z + 25), x_free=True)`: 힘제어 중이라 X는 도착 판정에서 빼고, Y·Z는 3 mm까지 허용합니다(벽 마찰로 Z가 덜 올라갈 수 있음).
     - `release_push_force()`
   - `release_compliance()`: 현재 실제 위치로 movel해서 눌린 스프링을 푼 뒤 `ReleaseComplianceCtrl`.
   - 현재 XY에서 상승점 Z까지 movel.
7. **[가상]** 계획 벽 위치까지 −X → 긁기 높이까지 +Z → 상승점. 위치 이동만 합니다. 시뮬레이터 외력값은 믿을 수 없기 때문입니다.
8. `finally`: 힘이나 컴플라이언스가 켜진 채 실패했으면 정지 → 힘 해제 → 컴플라이언스 해제를 시도하고, 각각의 실패는 로그만 남깁니다. 자동 후퇴는 하지 않습니다.

### 설계 메모

- 기본 각도 55°. 45°는 사선 시작점에서 손목 중심이 베이스 축에서 약 132 mm라 어깨 특이점(3205)과 1206이 났습니다. 50°(약 174 mm)는 검사 기준 180 mm에 걸립니다.
- 오프셋 147.7은 수직·기울인 두 접촉 자세로 역산했고, 실물에서 120·130보다 잘 맞았습니다.
- 긁기 높이를 용기 깊이(40)보다 낮게 둔 건, 벽이 끝나는 순간 −X로 누르던 헤라가 튀어 나가지 않게 하려는 것입니다.

---

## 4. 벽 도포

도포는 세 모듈로 나뉩니다. 모두 같은 프로세스 안의 Python 모듈이고 별도 노드가 아닙니다.

| 모듈 | 역할 |
|---|---|
| `apply_putty_plane.py` | 경로 **기하**: 날 끝 위치와 헤라 각도 → TCP 목록 |
| `putty_motion.py` | **실행**: 서비스, Tool/TCP, 특이점 감시, 힘제어, 도포 루프, 정리 |
| `contact_search.py` | **접촉 판정**: 바닥 탐색, 벽 누르기, 힘 감시 |

### 4.1 진입 — `apply_putty_plane.main()` `:79`

```text
make_plane_parser().parse_args()          :115  공통 옵션(putty_motion.make_parser) + 평면·탐색·힘 옵션
                                                singularity_guard=True, edge_offset=WALL_EDGE_OFFSET 기본
resolve_plane_args(args)                  :189  start 없으면 START_CONTACT, end 없으면 start + (0,0,stroke_height)
validate(args)                            :203  좌표·각도(0<θ<90)·강성 형식
validate_search(args)                     contact_search  탐색 시: YZ 벽 +X 접근만, 힘 기준은 자동 선택 안 함(floor/wall force 필수),
                                                press-method 목록, 강성 100~20000, 허용오차 < 목표힘 등
print_settings(args)                      :96   실제로 쓰일 경로·힘 설정을 로그로
run_robot(args, application_moves)        :180  → putty_motion.run_robot(args, motion_builder, allow_real=True,
                                                     continuous=True, contact_probe=seek_floor_then_wall, press_force)
```

### 4.2 경로 기하

**`surface_frame(args, blade_angle)` `:235`** — 벽 법선 `normal`(이 설정에서 BASE −X, 로봇 쪽)과 진행 방향 `travel`(+Z)로 헤라 축을 만듭니다.
- 퍼티 면 `face = −cos θ·normal + sin θ·travel`: 벽을 향합니다.
- 손잡이 `handle = sin θ·normal + cos θ·travel`: 벽 밖 자유 공간을 향합니다.
- 이 두 축과 외적을 `BLADE_REFERENCE_ABC`(뒤집어 잡은 자세)의 축 순서에 맞춰 Tool 축 3개를 돌려줍니다.
- θ는 **벽 기준** 각도입니다. 0°는 벽에 붙어 선 상태, 90°는 바닥과 평행입니다.

**`stroke_poses(args)` `:259`** — 진행률 u = 0→1을 `SAMPLES`(21)개로 나눠, 날 끝 위치와 각도(25° → 40°)를 함께 선형 보간합니다. 각 점에서 `surface_frame`으로 축을 다시 구하고 `TCP = 날 끝 − R·WALL_EDGE_OFFSET`(0, 0, 120)으로 계산합니다. 각도가 바뀌면 오프셋의 BASE 성분도 바뀌므로 점마다 다시 계산합니다.

**`behind_robot_exit(current, height, args)` `:280`** — 출발점이 로봇 뒤쪽(X < 0, 뜨기 직후)일 때만 끼워 넣는 경로입니다. 기둥 검사 후 다음을 만듭니다.
- `기둥 피해 경유지로 직선 이동` (경유지 XY, height, **현재 ABC**)
- `홈 위치로 관절 이동(뜬 자세 유지)` (홈 XY, height, 현재 ABC)

퍼티가 올라간 헤라 자세는 바꾸지 않습니다.

**`single_stroke_moves(current, args)` `:289`** — 전체 경유점 (이름, pose, 속도):

```text
면 접근 전 이송 높이 확보          (현재 XY, height = max(현재 Z, transfer_z, 접근 Z))
[뒤쪽 출발이면] 기둥 피해 경유지로 직선 이동 → 홈 위치로 관절 이동(뜬 자세 유지)
벽 앞으로 관절 이동하며 자세 정렬   (접근 XY, align_z = WALL_APPROACH_Z 200 근처, 도포 시작 ABC)   ← movej
면 바깥 접근점                    시작 TCP + normal·approach_distance(50) + Z 여유(10)
면에 수직 접근                    시작 TCP + normal·wall_backoff(10) + Z 여유  ← 여기서 접촉 탐색 시작
평면 각도 변화 도포 2/21 … 21/21   stroke_poses의 2~21번 점, apply_speed
면 법선 방향 후퇴                 끝 TCP + normal·approach_distance
```

`args.approach_offset`에 "면에 수직 접근이 계획 접촉점에서 얼마나 떨어져 있는지"를 저장해 두고, 실행부가 이 값으로 보정합니다.

### 4.3 실행 — `putty_motion.run_robot(args, motion_builder, ...)` `:145`

**준비**

1. 클라이언트를 만듭니다. 탐색 시에는 외력·정지·상태·컴플라이언스·힘·강성, 특이점 가드 시에는 `set_singularity_handling`과 `set_singular_handling_force`가 추가됩니다.
2. `GetRobotSystem`으로 실물/가상을 확인합니다.
3. `ensure_tool_and_tcp()`: Tool이 `Tool Weight`가 아니거나 TCP가 `GripperDA_v1`이 아니면 **수동 모드 → 선택 → 자동 모드**로 바꾸고 다시 읽어 확인합니다. 실패하면 움직이지 않습니다.
4. 특이점 가드(`singularity_guard=True`):
   - `start_singularity_watch()`: 별도 executor 스레드가 `/dsr01/error`와 `joint_states`를 구독합니다.
   - `wait_ready()`: 알람 publisher가 보일 때까지 기다립니다.
   - 팀 YAML의 회피 모드를 `utils.singularity.apply`로 적용합니다.
   - `SetSingularHandlingForce(mode=0)`: 특이점 내 힘제어 에러를 무시하지 않게 합니다.
5. `moves = list(motion_builder(read_pose(), args))`: 현재 TCP 기준으로 경로를 **전부** 계산합니다. 여기서 실패하면 첫 이동도 보내지 않습니다.
6. `continuous_commands(moves)` `:65`: 이름에 "평면 각도 변화 도포"가 들어간 연속 점들을 직전 점과 함께 `SplineStroke(label, poses, speed)` 하나로 묶습니다(2~100점).
7. `preflight_joints(commands)`: 모든 점(스플라인 점 포함)을 ikin으로 풀어 어깨 특이점(180 mm), J3 ≥ 41°, J5 ≥ 10°를 확인합니다. 통과하면 J3가 가장 작은 점을 로그로 남깁니다.

**경로 실행 루프** — `shift`는 계획 경로와 실제 접촉 위치의 차이(XYZ)이고, 처음에는 0입니다.

- 일반 경유점:
  - `HOME_TRANSFER_STEP`, `WALL_TRANSFER_STEP`이면 `move_joint_to_pose(..., direction='cw')`(ikin + 시계 방향 movej)
  - 나머지는 `move_line`(movel 동기 + 2 mm / 2°)
- `면 바깥 접근점` 도착 후: 허공의 Fx를 `free_fx`로 저장합니다. 나중에 "이미 벽에 닿았는지" 비교하는 기준입니다.
- `면에 수직 접근` 도착 후 접촉 단계 (`press_method='direct'` 기준):
  1. **컴플라이언스 시작** (`WallForceHold.start`): 탐색용 강성 X = press_stiffness 1000, Z = 1000. 0.3 s 대기.
  2. **`seek_floor_then_wall(io, args)`** `contact_search:443`
     - 바닥 −Z 탐색: `seek_axis(axis=2, dir=−1, floor_force 5 N, search_distance 30, 2 mm/s, confirm)`
     - `floor_lift`(0.5 mm)만큼 +Z로 들기: 바닥 마찰이 Fx 변화로 잡혀 벽으로 오판하는 걸 막습니다.
     - direct·position·hybrid 방식은 여기서 바닥 점을 돌려줍니다(저힘 벽 탐색 생략).
  3. 접촉점 기준으로 `shift`를 계산합니다(`approach_offset` 보정).
  4. `SetStiffnessx`로 Z 강성을 원래 값으로 되돌립니다(도포 경로 추종). X는 누르기 강성을 유지합니다.
  5. **`direct_press(io, args, wall_x, free_fx)`** `contact_search:234`
     - 지금 Fx가 `free_fx`보다 `wall_force` 이상 크면 이미 벽에 닿은 것으로 보고 그 자리를 벽으로 씁니다.
     - 아니면 측정 벽 X + 1 mm(`DIRECT_PRESS_MM`)까지 `amovel` 합니다. 거리가 `search_distance`보다 멀면 거부합니다.
     - 이동 중 외력 변화가 `wall_force` 이상 `confirm`번이거나, X가 1 s 동안 0.3 mm 미만으로 정체(힘 ≥ 1.5 N)되면 정지하고 그 X를 벽으로 씁니다.
     - 힘이 상한(`force_limit`)을 넘으면 정지 후 예외.
     - 반환: (실제 TCP, 벽 X, 기준 Fx `offset`)
  6. X 강성을 `HYBRID_STIFFNESS_X`(200 N/m)로 낮추고 `set_press_force(3)`: +X 3 N, 상대 모드.
  7. 누른 힘이 목표의 70%가 될 때까지 최대 3 s 기다립니다. 미달이어도 경고만 하고 시작합니다.
  8. 현재 실제 X를 기준 X로 다시 잡습니다(힘을 거는 동안 헤라가 벽 쪽으로 더 들어가기 때문).
  9. `PressForceMonitor(3 N, 허용오차, ..., 상한, offset, warn_only=True)`: 상한 초과만 중단하고, 힘 부족·과다는 경고만 합니다.
  10. `shift = (기준 X − 계획 X, 실제 Y − 계획 Y, 실제 Z − 계획 Z)`
- `SplineStroke`(상향 도포):
  - 힘이 켜져 있으면 컨트롤러가 `MoveSplineTask`를 거부합니다(1202). 그래서 **점마다 `amovel`** 을 보내고 0.05 s마다 다음을 확인합니다.
    - `force_monitor.update(Fx)`: 상한 초과면 중단
    - X 편차가 `X_FOLLOW_LIMIT_MM`(60)을 넘으면 중단
    - 로봇 상태가 1(STANDBY)·2(MOVING)가 아니면 중단
    - `check_motion`으로 IDLE이고 한 번이라도 움직였으면 다음 점
    - 3 s 동안 움직이지 않으면 "명령 거부"로 보고 중단
  - 힘이 꺼져 있으면 `spline_request`로 `MoveSplineTask` 한 번.
  - 끝나면 `stroke_arrived(actual, target, force_active)` `:108`: 힘제어 중에는 X를 빼고 Y·Z 5 mm / 5°까지 허용합니다.
  - 그다음 `hold.release()`(힘 해제 → 누름 풀기 → 컴플라이언스 해제). 후퇴는 힘제어로 바뀐 실제 X 기준으로 합니다.
- `면 법선 방향 후퇴`: movel.

**정리 (`finally`)** — 실패했거나 힘이 켜져 있으면 정지 확인 후 `hold.release()`, 특이점 감시를 닫습니다. 감시가 알람을 감지하면(`SingularityDetected`) 정지·힘 해제만 하고 자동 회복 이동은 하지 않습니다.

### 4.4 설계 메모

- 시작 각도 25°(벽 기준, 바닥 기준 65°). 20°는 도포 시작부 J3 37.8°로 검사에 걸렸고, 25°는 최소 52.0°입니다.
- `START_CONTACT` (819.41, −15.52, 27.20)는 20° 자세에서 실측한 값이고, 120 mm 모델 대비 22.5 mm 보정이 들어 있습니다. 실행 중 바닥 탐색과 벽 누르기가 실제 위치를 다시 찾으므로 몇 mm 오차는 흡수됩니다.
- 뜨기(147.7)와 도포(120)의 날 끝 오프셋이 다릅니다. 도포는 실측 보정으로 맞춰 둔 상태라 그대로 뒀습니다.

---

## 5. 헤라 반납 — `return_spatula.py`

**목적.** 도포 끝 자세에서 헤라를 보관대에 내려놓고 홈으로 돌아와 그리퍼를 닫습니다.

### 경로 함수 `return_waypoints(end_pose, home_pose, transfer_z, detour_pose)` `:102`

열기 전(`before`)과 열기 후(`after`) 두 묶음을 돌려줍니다. 각 항목은 (이름, 목표, 종류)이고, 종류는 `'movel'`(TCP 목표 직선) 또는 `'joints'`(관절값 그대로 movej)입니다.

```text
before
  시험판에서 수직으로 떼고 상승    movel   (끝 XY, max(transfer_z 245.14, 끝 Z), 끝 ABC)
  홈 관절 자세로 관절 이동        joints  HOME_J                         ← 벽 앞에서 팔을 편 채 돌지 않게
  경유 관절 자세로 관절 이동      joints  HOLDER_DETOUR_J                ← 집기와 같은 관절 경로
  보관대 위로 직선 이동          movel   (PICK XY, 230, PICKUP_ABC)      ← check_pillar_clearance로 사전 검사
  바닥 탐색 시작점까지 하강       movel   (PICK XY, PICK Z + 15, PICKUP_ABC)
after
  그리퍼 수직 후퇴              movel   보관대 위
  경유지로 직선 이동             movel   detour_pose(fkin 결과)
  홈으로 관절 이동              joints  HOME_J                         ← 집기 경로 역순(시계)
```

### 실행 `run_robot(args)` `:138`

1. `GetRobotSystem`, TCP를 확인합니다. 실물이면 외력·정지·`check_motion`·컴플라이언스·DO, 가상이면 onrobot 서비스를 기다립니다.
2. `joint_pose(call, clients, HOLDER_DETOUR_J)`: `Fkin`으로 경유지 TCP를 계산합니다(이동 없음).
3. `run(before)`: `'joints'`는 `move_to_joints(..., direction=None)`, `'movel'`은 movel 동기 + 2 mm / 2°.
4. **[실물] 바닥 찾기**
   - `start_compliance()`: `PLACE_STIFFNESS` (3000, 3000, **1000**, 200, 200, 200). Z만 부드럽게.
   - `seek_axis(axis=2, dir=−1, 5 N, 30 mm, 2 mm/s, confirm=2)` → 접촉 TCP.
   - `check_seated_height(contact Z)` `:131`: 기록된 바닥(`PICK_POSE` Z)과 ±10 mm를 넘게 다르면 **놓지 않고** 예외. 보관대 모서리 같은 다른 곳에 닿은 경우입니다.
   - `unload_and_release()`: 현재 위치로 movel해서 눌림을 없애고 컴플라이언스를 해제합니다.
5. **[가상]** 바닥 높이(`PICK_POSE`)까지 movel.
6. `set_gripper('열기', OPEN, 'o')` → `settle` 대기 → `run(after)` → **`set_gripper('닫기', CLOSE, 'c')`**
7. `finally`: 컴플라이언스가 켜진 채 실패했으면 정지 → 해제. **그리퍼는 열지도 닫지도 않습니다.**

### 설계 메모

- 도포 중 파지가 밀리면 실제 바닥 높이가 달라집니다. 그래서 실물은 고정 높이로 내리지 않고 힘으로 찾습니다.
- 반납이 홈에서 끝나므로, 전체 공정 마지막 초기화는 실제로 이동하지 않고 홈 확인과 닫기만 합니다.

---

## 6. 액션 서버와 전체 공정 launch

### 6.1 `action_protocol.worker_command(goal)` `:49`

1. `goal.task`가 initialize/pickup/scoop/load/apply/return 중 하나인지 확인합니다.
2. 실물 scoop·load는 `tool_held_confirmed`가 없으면 거부합니다.
3. `arguments`를 앞에서부터 읽으며, 각 옵션이 `OPTIONS[task]`에 있고 중복이 아니며 값 개수와 숫자 형식이 맞는지 확인합니다. 하나라도 틀리면 Goal을 거절합니다.
4. initialize는 `--ros-args -p execute_motion:=true -p virtual_gripper:=…`, 나머지는 `--execute-motion` + 옵션으로 argv를 만듭니다.
   - 가상 pickup에는 `--virtual-gripper`를 붙입니다.
   - 실물 apply에는 `--contact-search`를 붙이고, 실제 파서로 한 번 파싱해 미리 검증합니다.
5. `[python, -u, -m, putty_demo.action_worker, task, ...]`를 돌려줍니다.

새 CLI 옵션을 만들면 **여기 `OPTIONS`에도 넣어야** 액션으로 쓸 수 있습니다.

### 6.2 `putty_task_server`

- **`__init__`** `:93`: 파일 잠금(`fcntl.flock`)으로 같은 PC·도메인에 서버가 하나만 뜨게 합니다. 두 번째 서버는 "이미 실행 중"으로 종료합니다.
- **`execute(handle)`** `:296`
  1. `worker_command` → `WorkerProcess(command, grasp_confirmed)`. 확인된 Goal이면 자식 stdin에 `y`를 씁니다.
  2. 자식 출력 줄을 모아 약 0.2 s마다 Feedback을 보냅니다(stage, 경과 시간, TCP, 외력, 마지막 로그 줄).
  3. 종료 코드가 0이면 success, 아니면 실패 메시지와 함께 정리합니다.
- **`cleanup_worker(worker, task)`** `:224`: 취소·실패 시 순서대로 실행합니다. 하나라도 실패하면 aborted로 끝내고 새 Goal을 막습니다.
  1. 작업 프로세스 SIGSTOP
  2. `MoveStop(DR_HOLD)` 후 정지 확인
  3. 프로세스 종료
  4. 정지 재확인
  5. apply·load·return·scoop이면 `ReleaseForce` → `ReleaseComplianceCtrl`
  - 정지 확인이 아직 `get_robot_state == STANDBY` 기준이라, 컴플라이언스 중에는 일찍 통과할 수 있습니다.

### 6.3 `launch/putty_full_cycle.launch.py`

- **`build_steps(context)`**: launch 인자를 읽어 (단계 이름, ExecuteProcess) 목록을 만듭니다.
  - 순서: initialize → pickup → [`confirm_grasp:=true`이면 /dev/tty y 확인] → scoop(`--tool-held-confirmed`) → apply(실물은 floor·wall force 포함) → return → initialize
  - 각 단계는 `ros2 run rokey1 putty_task_client <task> [--virtual-mode] -- <옵션>`입니다.
- **`chain(steps)`**: 단계마다 `OnProcessExit` 핸들러를 등록합니다. 종료 코드가 0이 아니면 `Shutdown`, 마지막 단계면 완료 후 `Shutdown`, 아니면 다음 단계를 실행합니다.
- **`stop_when_server_exits`**: launch가 띄운 서버가 먼저 끝나면(중복 실행 잠금 등) 공정 전체를 끝냅니다. 클라이언트가 예전에 떠 있던 다른 서버로 Goal을 보내지 않게 하려는 것입니다.
- 테스트(`test/test_putty_full_cycle_launch.py`)는 `build_steps`만 호출해 단계 순서와 옵션을 확인합니다(ROS 실행 없음).

---

## 7. 알람 코드와 대응 위치

| 코드 | 의미 | 이 프로젝트에서 나온 상황 | 대응 코드 |
|---|---|---|---|
| 3205 | 특이점 영역 진입·이탈 | 뜨기 45° 사선 시작점(어깨), 도포 높은 위치·벽 앞(팔꿈치) | `check_joint_target`, `preflight_joints`의 J3·어깨 검사, 도포 `SingularityWatch` |
| 1206 | 목표 자세 도달 불가 (NOT REACHABLE) | 3205 직후 뜨기 사선 끝점 | 위와 같음. 사전 검사로 출발 전에 막음 |
| 1202 | 컴플라이언스 중 `MoveSplineTask` 거부 | 힘제어 도포 | 점마다 `amovel`로 대체 (`putty_motion` 도포 루프) |
| 7060 | 안전 충돌 감지 | 단단한 헤라로 위치제어 접촉 | 닿기 전 컴플라이언스(바닥·벽 탐색, 반납, 뜨기 벽 긁기) |
| 3501 | 접근 중 벽 접촉 | 헤라 파지 깊이 편차 | `WALL_BACKOFF_MM` 10, `direct_press`의 "이미 닿음" 판정 |
| 9015 | 외부 힘 초과(충돌) | bringup 직후 홈 movej 중 96 N | 초기화 경로 개선 필요(1장 주의) |
