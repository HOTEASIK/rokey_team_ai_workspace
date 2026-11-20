# 퍼티 작업 노드 (putty_demo)

M0609 + RG2로 헤라를 집어 퍼티를 뜨고, 벽에 힘제어로 바른 뒤 헤라를 반납하는 ROS 2 노드 모음입니다.
ROS 2 Jazzy, 네임스페이스 `dsr01`, TCP `GripperDA_v1`, 툴 무게 `Tool Weight`를 사용합니다.

> [!NOTE]
> 이 문서는 2026-11-19 코드 기준입니다. 소스 폴더는 `src/rokey1_ws`지만 ROS 패키지 이름은 **`rokey1`**입니다.
> 액션 서버·취소 절차는 [ACTIONS.md](ACTIONS.md), 처음 읽는 팀원용 흐름 설명은 [actions_skill.md](actions_skill.md)를 봅니다.
> 단계별 함수 동작(무엇을 받고 무엇을 검사하고 어떤 두산 요청을 보내는지)은 [CODE_WALKTHROUGH.md](CODE_WALKTHROUGH.md)를 봅니다.

## 공정

```text
초기화(홈) → 헤라 집기 → 퍼티 뜨기 → 벽에 도포 → 헤라 반납 → 초기화(홈)
```

| 작업 | 실행 파일 | 하는 일 | 실물/가상 |
|---|---|---|---|
| `initialize` | [initialize_device.py](initialize_device.py) | 홈 관절 자세로 이동, 그리퍼 닫기 | 둘 다 |
| `pickup` | [pickup_spatula.py](pickup_spatula.py) | 로봇 뒤쪽 보관대에서 헤라 집기 | 둘 다 |
| `scoop` | [scoop_putty.py](scoop_putty.py) | 헤라를 기울여 용기에서 퍼티 뜨기 | 둘 다 (실물은 파지 확인 필요) |
| `apply` | [apply_putty_plane.py](apply_putty_plane.py) | 벽 앞으로 이동, 바닥·벽 찾기, +X 힘제어로 상향 도포 | 둘 다 |
| `return` | [return_spatula.py](return_spatula.py) | 보관대에 헤라 반납 (실물은 바닥을 힘으로 감지) | 둘 다 |
| `load` | [load_putty.py](load_putty.py) | 판에 퍼티 묻히기 (현재 공정 흐름에는 없음) | 둘 다 (실물은 파지 확인 필요) |

## 파일 구조

```text
putty_demo/
├─ 작업 노드 (ros2 run으로 직접 실행)
│  ├─ initialize_device.py   홈 복귀 + 그리퍼 닫기
│  ├─ pickup_spatula.py      헤라 집기. movej 공용 함수(move_joint_to_pose, 관절 검사)도 여기 있음
│  ├─ scoop_putty.py         퍼티 뜨기
│  ├─ apply_putty_plane.py   벽 도포 설정·경로 계산
│  ├─ return_spatula.py      헤라 반납
│  └─ load_putty.py          판에 퍼티 묻히기
├─ 공용 모듈 (노드가 import해서 씀, 단독 실행 안 함)
│  ├─ putty_motion.py        도포 실행부: 서비스 호출, 힘 유지, 벽 앞 movej
│  ├─ contact_search.py      접촉 탐색, 누르기(direct/hybrid/position/force), 힘 판정
│  ├─ putty_geometry.py      ZYZ 회전·좌표 계산, 헤라 방향 기준값(BLADE_REFERENCE_ABC)
│  └─ singularity_guard.py   /dsr01/error 특이점 알람 감시
└─ 액션 서버 (HMI·launch·팀 매니저용)
   ├─ putty_task_server.py   /dsr01/putty_task, /process/putty, /process/putty_load
   ├─ putty_task_client.py   터미널용 클라이언트
   ├─ action_protocol.py     Goal 옵션 허용 목록·검증, 팀 YAML → CLI 옵션 변환
   └─ action_process.py / action_worker.py   작업 노드를 자식 프로세스로 실행·중단
```

모든 노드는 `DSR_ROBOT2` 함수 대신 `dsr_msgs2` 서비스를 직접 호출합니다.
`DSR_ROBOT2`는 응답을 시간 제한 없이 기다려 이동 중 취소·시간 초과·특이점 알람 확인을 할 수 없기 때문입니다.
코드 주석에 각 서비스가 어떤 두산 API(`movel`, `movej`, `amovel`, `set_desired_force` 등)와 같은지 적혀 있습니다.

## 빠른 시작

### 준비

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/rokey_interfaces src/rokey1_ws --packages-select rokey_interfaces rokey1
source install/setup.bash
```

> [!IMPORTANT]
> 모든 터미널(bringup 포함)이 같은 통신 설정이어야 서로 보입니다.
> - `ROS_DOMAIN_ID=11` (`~/.bashrc`와 같은 값)
> - `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`는 **비어 있거나 모든 터미널에서 같은 값**이어야 합니다.
>   디스커버리 서버가 설정돼 있는데 그 서버가 꺼져 있으면 같은 PC의 노드끼리도 못 찾습니다.
>
> ```bash
> echo "$ROS_DOMAIN_ID [$ROS_DISCOVERY_SERVER]"   # 11 []
> ```

> [!NOTE]
> 기존 설치가 복사 방식이라 `--symlink-install`을 붙이면 `File exists` 오류가 납니다. 붙이지 않고 빌드합니다.
> 따라서 **코드를 고친 뒤에는 다시 빌드**해야 `ros2 run`/액션 서버에 반영됩니다.

로봇 연결(bringup)은 별도 터미널에서 실행합니다. 이 패키지는 컨트롤러를 새로 띄우지 않습니다.

```bash
# 실물 (컨트롤러와 유선 연결 필요: ping 192.168.1.100)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
# 가상
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
```

### 방법 1: 노드 직접 실행 (첫 확인용)

액션 서버는 켜지 않습니다. 한 단계씩 실행하고 로봇 상태를 확인한 뒤 다음으로 넘어갑니다.

```bash
ros2 run rokey1 initialize_device --ros-args -p execute_motion:=true -p virtual_gripper:=false
ros2 run rokey1 pickup_spatula --execute-motion --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 scoop_putty --execute-motion --scoop-angle 45        # 실물은 'y'로 파지 확인
ros2 run rokey1 apply_putty_plane --execute-motion --contact-search \
  --floor-force 5 --wall-force 5 --press-force 3 --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 return_spatula --execute-motion --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 initialize_device --ros-args -p execute_motion:=true -p virtual_gripper:=false
```

가상 모드는 집기에 `--virtual-gripper`, 초기화에 `virtual_gripper:=true`를 붙입니다.

### 방법 2: 액션 서버 + 클라이언트

```bash
ros2 run rokey1 putty_task_server                      # 켜기만 해서는 움직이지 않음
ros2 action list | grep putty                          # /dsr01/putty_task 가 보여야 함
ros2 run rokey1 putty_task_client pickup -- --joint-speed 10
```

자세한 사용법은 [ACTIONS.md](ACTIONS.md)를 봅니다. **방법 1과 2를 동시에 쓰지 않습니다.**

## 단계별 동작과 두산 API

모든 이동은 동기(이동이 끝난 뒤 응답) 호출이고, 끝나면 실제 위치가 목표 2 mm / 2° 안인지 다시 확인합니다.
예외로 `amovel`(비동기)은 이동 중 외력을 감시하는 구간에서만 씁니다.

### 초기화 (`initialize_device`)

| 순서 | 동작 | 두산 API |
|---|---|---|
| 1 | 홈 관절 `[0,0,90,0,90,0]`으로 이동 (이미 홈이면 생략) | `movej` 10 deg/s |
| 2 | 그리퍼 닫기 | 실물 `set_digital_output` 1·2번 / 가상 `/onrobot/sendCommand 'c'` |

### 헤라 집기 (`pickup_spatula`)

보관대는 로봇 뒤쪽 `PICK_POSE = [-508.39, 284.79, 121.85, 136.31, -179.38, 136.56]`입니다.
헤라가 보관대 바닥에 완전히 앉은 높이에서 잡으므로 매번 같은 깊이를 잡습니다. 이때 퍼티 면은 BASE **-X**를 봅니다.

| 순서 | 동작 | 두산 API |
|---|---|---|
| 1 | TCP 확인·적용, 홈 관절 자세인지 확인 (±0.5°) | `get_tcp`/`set_tcp`, `get_current_posj` |
| 2 | 홈 위로 Z 230까지 수직 상승 | `movel` 15 mm/s |
| 3 | **직접 찍은 경유 관절 자세** `HOLDER_DETOUR_J` (121.69, -2.45, 94.67, 0.17, 88.37, 121.89)로 이동. 위에서 볼 때 반시계(J1 +121.7°). TCP (-185.48, 292.47, 204.82) | `movej` (절대 관절각, ikin 없음) |
| 3-1 | 그 자리에서 보관대 위 이송 높이 Z 230으로 수직 상승 | `movel` 15 mm/s |
| 3-2 | 헤라 바로 위로 직선 이동 | `movel` 15 mm/s |
| 4 | 그리퍼 열기 → 수직 하강 → 닫기 → 수직 상승 | `set_digital_output`, `movel` 5 mm/s |

지그 기준 +X 150, +Y 170 mm(BASE 약 (-358, 455))에 기둥이 있습니다. movej는 TCP가 곡선으로 움직여 기둥에 부딪쳤기 때문에,
movej는 홈에서 movej로 기둥에 닿지 않음을 현장에서 확인한 경유 관절 자세까지만 하고 이후는 직선입니다. 직선 구간은 기둥 중심에서 100 mm 이상 떨어져야 출발합니다(계산 검사).

movej 전에 다음을 검사하고 하나라도 걸리면 움직이지 않습니다.
- 이송 높이가 바닥 대비 보관대 깊이 80 mm + 여유 20 mm 이상인지 (`check_holder_clearance`)
- J1 회전 방향·회전량(0~180°) (`turn_joint_target`)
- 팔꿈치 J3 ≥ 41°, 손목 |J5| ≥ 10°, 손목 뒤집힘(J5 부호 변화), 어깨 특이점(손목 중심이 베이스 축에서 180 mm 이상) (`check_joint_target`)

### 퍼티 뜨기 (`scoop_putty`)

용기는 보관대 바로 옆입니다. 기준점 `TAUGHT_SCOOP_POSE = [-553.78, 82.25, 122.70, 160.95, -179.05, 160.87]`은
헤라를 수직으로 쥔 채 날 폭 가운데가 용기 -X 쪽 끝 바닥 구석에 닿은 TCP입니다. 용기는 X 110 × Y 120 × 깊이 40 mm입니다.

| 순서 | 동작 | 두산 API |
|---|---|---|
| 0 | 경유점 + 벽 위치·긁어 올린 점·최종 상승점(계획값) 모두 관절 검사 (특이점·손목 뒤집힘) | `ikin` |
| 1 | 현재 자세로 이송 높이 확보 | `movel` 30 mm/s |
| 2 | 그 자리에서 헤라를 기울이기: 날 끝 고정, BASE Y축 +(90° − 각도) 회전 | `movel` |
| 3 | 사선 시작점 위 → 시작점 (끝점에서 +X·+Z로 `entry-height` 120 mm) | `movel` |
| 4 | **각도를 유지한 채 -X·-Z 사선 진입**. `--scoop-pass N`이면 끝점이 +X로 (N−1)×20 mm | `movel` 3 mm/s |
| 5 | [실물] 컴플라이언스(X·Z 1000 N/m) → **-X로 밀어 벽 접촉 감지** (2 mm/s, Fx 변화 5 N 2회, 최대 회차 이동량+10 mm) | `task_compliance_ctrl`, `amovel` + `get_tool_force`, `stop(DR_HOLD)` + `check_motion` |
| 6 | [실물] **-X 3 N 유지하며 +Z 25 mm 긁어 올리기** → 힘 해제 → 눌림 풀기 → 컴플라이언스 해제 | `set_desired_force`(REL), `movel`, `release_force`, `release_compliance_ctrl` |
| 5'·6' | [가상] 기록된 벽 위치까지 -X 이동 → 25 mm 상승 (외력 탐색 생략) | `movel` |
| 7 | 남은 높이 수직 상승 (끝점 기준 총 120 mm) | `movel` 3 mm/s |

여러 번 뜰 때는 회차를 줍니다: 1회차 `--scoop-pass 1`(기본, 원래 끝점), 2회차 `--scoop-pass 2`(+20 mm), … 최대 4회차(+60 mm).
`--wall-scrape-height`(기본 25 mm, 0~30)는 벽을 누르며 올라가는 높이입니다. 0이면 벽에 닿은 뒤 바로 힘을 풀고 상승합니다.
용기 벽(깊이 40 mm)이 끝나기 전에 힘을 풀어야 누르던 방향으로 튀어 나가지 않습니다.
실패·취소 시 정지 → 힘 → 컴플라이언스 순서로 해제합니다(액션 서버 취소 정리도 scoop을 힘 사용 작업으로 처리).

- 각도는 45~60°만 허용합니다(`--scoop-angle`, 기본 **55°**). 45°는 사선 시작점에서 손목 중심이 베이스 축 위(약 132 mm)로 와 어깨 특이점(3205)·도달 불가(1206)가 났고, 50°(약 174 mm)는 출발 전 검사에 걸립니다. 기울이면 손잡이는 +X·위, 날 끝은 -X, 퍼티 면은 위를 봅니다.
- 썰기 끝의 날 끝(폭 가운데)은 **측정 접촉점과 같습니다**(여유 없음, 2026-11-19 결정).
  기울이면 두께 때문에 날 아래 모서리가 계산상 바닥 아래로 들어갑니다(45°에서 약 2.1 mm). 위치제어라 충돌 감지(7060)가 날 수 있습니다.
- 사선 경로가 +X 벽 위를 충분히 높게 지나는지, 헤라 폭(100 mm 가정)이 용기 Y 폭에 들어가는지 검사합니다.

### 벽 도포 (`apply_putty_plane`)

기본 누르기 방식은 `--press-method direct`입니다.

| 순서 | 동작 | 두산 API |
|---|---|---|
| 0 | 시스템 확인, Tool `Tool Weight`·TCP `GripperDA_v1` 선택 (수동 모드 → 선택 → 자동 모드) | `set_robot_mode`, `set_tool`, `set_tcp` |
| 0 | 특이점 정책(팀 YAML의 회피 모드 + 힘제어 특이점 ERROR) 설정, 알람 감시 시작, 경로 전체 관절 검사 | `set_singularity_handling`, `set_singular_handling_force`, `ikin` |
| 1 | 제자리에서 이송 높이 확보 | `movel` |
| 2 | **벽 앞으로 이동. 위에서 볼 때 시계 방향(J1 감소)**, 같은 이동 안에서 J6가 약 180° 돌아 퍼티 면이 +X 벽을 향함 | `ikin` → `movej` |
| 3 | 면 바깥 접근점(벽에서 50 mm) → 날 끝을 벽에서 10 mm, 바닥에서 10 mm 띄운 점으로 이동 | `movel` |
| 4 | 컴플라이언스(X·Z 1000 N/m) 켜고 바닥 -Z 탐색 → 0.5 mm 들기 → Z 강성 복원 | `task_compliance_ctrl`, `amovel` + `get_tool_force` + `stop(DR_HOLD)`, `set_stiffnessx` |
| 5 | 측정된 벽 X보다 1 mm 안쪽까지 이동, 먼저 닿으면 그 자리를 벽으로 사용 | `amovel` + `get_tool_force`, `check_motion` |
| 6 | X 강성 200 N/m, BASE +X 3 N 힘제어(상대 모드). 목표의 70%까지 최대 3초 대기 | `set_stiffnessx`, `set_desired_force` |
| 7 | 헤라 각도 20° → 40°로 바꾸며 날 끝 Z 206까지 상향 도포 (MoveLine 20구간) | `amovel` × 20, `check_motion` |
| 8 | 힘 해제 → 1 mm 눌림 풀기 → 컴플라이언스 해제 → 법선 방향 후퇴 | `release_force`, `movel`, `release_compliance_ctrl` |

- 컴플라이언스 중에는 컨트롤러가 `MoveSplineTask`를 거부하므로 MoveLine 구간으로 나눠 보냅니다.
- 컴플라이언스 중에는 `get_robot_state`가 이동 중에도 STANDBY를 줄 수 있어 이동 완료는 `check_motion`으로 판정합니다.
- 도포 중 힘 부족은 경고 로그만 남기고, |Fx| 상한(기본 20 N) 초과 시에만 정지·해제합니다.
- 실패·Ctrl+C 시 정지를 확인한 뒤 힘·컴플라이언스 해제를 시도하며, 자동 후퇴·재시도는 하지 않습니다.

### 헤라 반납 (`return_spatula`)

| 순서 | 동작 | 두산 API |
|---|---|---|
| 0 | 경유 관절 자세 `HOLDER_DETOUR_J`의 TCP 계산 (이동 없음) | `fkin` |
| 1 | 이송 높이까지 수직 상승 | `movel` |
| 2 | 홈 관절 자세로 이동 (벽 앞에서 팔을 편 채 기둥 쪽으로 돌지 않게) | `movej` (관절값 그대로) |
| 2-1 | 집기와 같은 경유 관절 자세로 이동, **반시계**. 집기 방향 자세 정렬도 이때 | `movej` (관절값 그대로) |
| 2-2 | 보관대 위로 직선 이동 (기둥 중심에서 100 mm 이상인지 출발 전 계산 검사) | `movel` |
| 3 | 바닥보다 15 mm 위까지 하강 | `movel` |
| 4 | [실물] 컴플라이언스(Z 1000 N/m) → 2 mm/s 하강하며 외력 감시 → Fz 변화 5 N(2회 연속)에서 정지 | `task_compliance_ctrl`, `amovel` + `get_tool_force`, `stop(DR_HOLD)` + `check_motion` |
| 5 | [실물] 찾은 바닥이 기록 높이 ±10 mm 안인지 확인 → 눌림 풀기 → 컴플라이언스 해제 | `movel`, `release_compliance_ctrl` |
| 4' | [가상] 바닥 높이까지 하강 (시뮬레이터 외력값은 믿을 수 없어 힘 탐색 생략) | `movel` |
| 6 | 그리퍼 열기 → 수직 후퇴 → 경유지로 직선 이동 | `set_digital_output` / onrobot, `movel` |
| 7 | 홈으로 이동. 집기 때 기둥에 닿지 않은 관절 경로를 거꾸로 간다 | `movej` (관절값 그대로) |
| 8 | 홈에서 그리퍼 닫기 | `set_digital_output` / onrobot |

실패하면 정지·컴플라이언스 해제만 하고 **그리퍼는 열지 않습니다.**

## 주요 설정값

| 위치 | 이름 | 값 | 뜻 |
|---|---|---|---|
| `pickup_spatula.py` | `PICK_POSE` | 위 표 참고 | 헤라 집기·반납 TCP |
| | `HOLDER_TRANSFER_Z_MM` | 230 | 보관대 위 이송 높이 (245는 J3 40.4°로 관절 검사에 걸림) |
| | `HOLDER_DEPTH_MM` / `HOLDER_CLEARANCE_MM` | 80 / 20 | 보관대 깊이·여유 |
| | `HOLDER_DETOUR_J` | (121.69, -2.45, 94.67, 0.17, 88.37, 121.89) | movej 경유 관절 자세 (현장 측정, 집기·반납 공통) |
| | `HOLDER_PILLAR_XY` / `PILLAR_CLEARANCE_MM` | 지그 +(150, 170) / 100 | 기둥 위치·직선 구간 최소 거리 |
| | `JOINT_SPEED_DEG_S` / `JOINT_ACCELERATION_DEG_S2` | 20 / 20 | movej 속도 (CLI `--joint-speed`) |
| `scoop_putty.py` | `TAUGHT_SCOOP_POSE` | 위 표 참고 | 뜨기 기준점 |
| | `SCOOP_ANGLE_RANGE_DEG` | 45~60 | 헤라 기울기 허용 범위 |
| | `SCOOP_PASS_STEP_MM` / `SCOOP_MAX_PASS` | 20 / 4 | 회차당 끝점 +X 이동량 / 최대 회차 |
| | `WALL_CONTACT_FORCE_N` / `WALL_SEARCH_SPEED_MM_S` | 5 / 2 | 벽 접촉 감지 힘 / 탐색 속도 |
| | `WALL_PUSH_FORCE_N` / `WALL_SCRAPE_HEIGHT_MM` | 3 / 25 | 긁어 올릴 때 -X 힘 / 높이 |
| | `SCRAPE_STIFFNESS` | (1000, 3000, 1000, 200, 200, 200) | 벽 긁기 컴플라이언스 강성 |
| | `SCOOP_EDGE_OFFSET` | (0,0,147.7) | TCP → 날 끝. 두 접촉 자세로 역산, 실물 뜨기에서 확인 (120·130보다 잘 맞음) |
| `apply_putty_plane.py` | `START_CONTACT` | (819.41, -15.52, 27.20) | 날 끝이 벽에 닿는 점 (20° 시작 자세 실측) |
| | `STROKE_END_Z` | 206 | 도포 끝 날 끝 Z |
| | `BLADE_START/END_ANGLE_DEG` | 20 / 40 | 벽과 헤라 사이 각도. 위로 갈수록 키워 팔꿈치 특이점 회피 |
| | `PRESS_FORCE_N` | 3 | +X 목표 힘 |
| | `WALL_APPROACH_Z` | 200 | 벽 앞 자세 정렬 높이 (245는 J3 39°로 특이점) |
| `putty_geometry.py` | `BLADE_REFERENCE_ABC` | (-43.69, -179.38, 136.56) | 퍼티 면이 +X를 보는 TCP 자세 = 집기 자세를 Z로 180° |
| `return_spatula.py` | `PLACE_CONTACT_FORCE_N` | 5 | 바닥 감지 힘 (CLI `--contact-force`) |
| | `PLACE_STIFFNESS` Z | 1000 | 바닥 탐색 중 Z 강성 |
| `putty_motion.py` | `PREFLIGHT_J3_MIN_DEG` / `J5` | 41 / 10 | 관절 검사 기준 |
| `../config/robot.yaml` | `force.putty_scrape` | | 팀 Process 경로의 도포 설정 (목표 3 N, X 강성 200 등) |

> [!WARNING]
> 아래는 **실물로 아직 검증하지 않은** 값·가정입니다.
> - `START_CONTACT`: 헤라 방향 기준값을 바꾸면서 같은 날 끝 점의 시작 TCP가 약 +4 mm(X) 달라졌고,
>   새 보관대에서 헤라 쥐는 깊이도 달라졌을 수 있습니다. 다시 확인이 필요합니다.
> - 반납 바닥 감지 5 N / Z 강성 1000은 도포의 바닥 탐색 값을 가져온 것입니다.
> - 헤라 폭 100 mm, 두께 6 mm, 뜨기 기준점이 용기 Y 가운데라는 가정.
> - 벽 앞 movej에서 J6가 도는 방향(+180° / -180°)은 실물 ikin 결과로 정해집니다.

## 테스트

로봇 없이 가짜 서비스로 경로·검사·호출 순서를 확인합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey1_ws
python3 -m pytest -q $(ls test/test_*.py | grep -v "copyright\|flake8\|pep257")
```

2026-11-19 기준 189개 통과. 실물 접촉 성능·정지 거리·충돌은 검증하지 않습니다.

## 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `Waiting for service …` 반복 | bringup이 떠 있는지, `ROS_DOMAIN_ID`·디스커버리 서버 설정이 같은지 |
| 클라이언트 "퍼티 액션 서버가 없습니다" | 서버 터미널과 클라이언트 터미널의 통신 설정, `ros2 action list` |
| "이미 이 PC/ROS domain에서 퍼티 액션 서버가 실행 중" | `pgrep -af putty_task_server`로 남은 서버 확인 |
| `[관절 검사] … 특이점 근처` / `손목이 반대로 꺾이는 해` | 로그의 관절값 확인. 경유 자세 추가 또는 좌표 재확인 |
| `반시계/시계 J1 회전량 … 벗어났습니다` | 시작 자세가 예상과 다름 (예: 집기는 홈에서 시작해야 함) |
| 충돌 감지 7060 / 3501 | 위치제어로 바닥·벽을 밀었는지. 끝점 여유·접근 높이 확인 |
| 특이점 알람 3205 | 이송 높이·각도. `[특이점 감지]` 로그의 관절각 확인 |

## 변경 이력 요약

- **2026-11-19**
  - 11-18 인수인계본의 도포 수정 반영: Tool Weight 선택, 바닥 탐색부터 컴플라이언스, direct 누르기,
    X 강성 200 + 3 N 상대 힘, MoveLine 20구간, `check_motion` 완료 판정, 출발 전 ikin 관절 검사, 10 N 하한 제거.
  - 보관대를 로봇 뒤쪽으로 이동: 집기·반납을 반시계 movej로, 반납은 바닥 힘 감지로 변경, 실물 반납 허용.
  - 퍼티 뜨기를 새 용기 위치·-X 진입 방식으로 변경(추정 기준각 60° 제거).
  - 헤라가 반대로 쥐어지므로 도포 기준값을 180° 뒤집고, 벽 앞 이동을 시계 방향 movej로 변경.
  - 정리: `service_waiter.py`, 예전 가상 geometry 경로와 `putty_cycle.launch.py` 삭제.
- **2026-11-18 이전**: 단일 YZ 벽 상향 도포, 접촉 탐색, 특이점 감시, PuttyTask/Process 액션 서버 도입.
