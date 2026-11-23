# 퍼티 작업 Action 사용법

2026-11-19 코드 기준입니다. 노드별 동작·두산 API·설정값은 [README.md](README.md)를 봅니다.

## 구조

```text
HMI / putty_task_client / 다른 팀 공정 노드          팀 manager
        ↕ PuttyTask Action                              ↕ Process Action
/dsr01/putty_task  ─────────── putty_task_server ─────────── /process/putty, /process/putty_load
        ↓ 작업 노드를 자식 프로세스로 격리 실행 (action_process / action_worker)
initialize_device / pickup_spatula / scoop_putty / load_putty / apply_putty_plane / return_spatula
        ↕ dsr_msgs2 서비스
/dsr01/dsr_controller2 → M0609 / RG2
```

- 인터페이스는 `rokey_interfaces/action/PuttyTask`와 `rokey_interfaces/action/Process`입니다.
- 서버는 한 번에 Goal 하나만 실행하고, 실행 중 들어온 Goal은 거절합니다. 같은 PC·domain의 서버 중복 실행도 막습니다.
- **같은 로봇에 수동 CLI(`ros2 run …` 직접 실행)와 액션 서버를 동시에 쓰지 않습니다.**
- 작업 노드를 별도 프로세스로 돌리는 이유: 동기 이동 서비스 응답을 기다리는 동안에도 서버가 취소를 받아 정지 요청을 보낼 수 있게 하려는 것입니다.

## 빌드 및 시작

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/rokey_interfaces src/rokey1_ws --packages-select rokey_interfaces rokey1
source install/setup.bash
ros2 run rokey1 putty_task_server          # 또는 ros2 launch rokey1 putty_actions.launch.py
```

서버를 켜기만 해서는 움직이지 않습니다. bringup과 **같은 통신 설정**이어야 합니다.

```bash
echo "$ROS_DOMAIN_ID [$ROS_DISCOVERY_SERVER]"   # 모든 터미널에서 같은 값 (지금: 11 [])
ros2 action list | grep putty                    # /dsr01/putty_task 가 보여야 클라이언트를 실행
```

별도 터미널에서 작업을 요청합니다. 아래는 실물 요청이며, 첫 실행용으로 관절 속도를 낮췄습니다.

```bash
ros2 run rokey1 putty_task_client initialize
ros2 run rokey1 putty_task_client pickup -- --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 putty_task_client scoop --tool-held-confirmed -- --scoop-angle 45
ros2 run rokey1 putty_task_client apply -- \
  --floor-force 5 --wall-force 5 --press-force 3 --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 putty_task_client return -- --joint-speed 10 --joint-acceleration 10
ros2 run rokey1 putty_task_client initialize
```

- `--` 뒤는 작업별 CLI 옵션입니다. 서버가 [action_protocol.py](action_protocol.py)의 **허용 목록**으로 검사하고,
  목록에 없거나 형식이 틀린 옵션이 있으면 Goal을 거절합니다.
- `--execute-motion`, `--ros-args`, `--virtual-gripper`는 서버가 붙이므로 Goal 인자로 보내지 않습니다.
- 실물 `apply`는 `--contact-search`를 자동으로 붙입니다. 이때 `--floor-force`, `--wall-force`는 기본값이 없으므로 반드시 줍니다.
- 실물 `scoop`/`load`는 `--tool-held-confirmed`가 필요합니다. 사용자가 헤라 파지를 눈으로 확인했다는 뜻이며 센서 판정이 아닙니다.
- 실물 `return`도 허용합니다. 실물 반납은 바닥 접촉을 힘으로 감지해 내려놓습니다.

가상 모드는 작업명 뒤에 `--virtual-mode`를 붙입니다. 연결된 컨트롤러(`GetRobotSystem`)와 Goal의 모드가 다르면 시작하지 않습니다.

```bash
ros2 run rokey1 putty_task_client initialize --virtual-mode
ros2 run rokey1 putty_task_client pickup --virtual-mode
```

다른 ROS 노드나 CLI에서 직접 Goal을 보낼 수도 있습니다.

```bash
ros2 action send_goal /dsr01/putty_task rokey_interfaces/action/PuttyTask \
  "{task: apply, virtual_mode: false, tool_held_confirmed: false, arguments: ['--floor-force', '5', '--wall-force', '5', '--press-force', '3']}" \
  --feedback
```

중간 취소는 `putty_task_client`에서 Ctrl+C를 누르거나, 다른 ActionClient에서 GoalHandle의 `cancel_goal_async()`를 호출합니다.
일반 `ros2 action send_goal`의 Ctrl+C만으로 작업이 취소됐다고 가정하지 않습니다.

## 피드백과 결과

- Goal: `task`(initialize/pickup/scoop/load/apply/return), `virtual_mode`, `tool_held_confirmed`, `arguments`.
- Feedback: 현재 `stage`, `elapsed_sec`, BASE `tcp_pose[6]`, BASE `tool_force[6]`, 원본 `log_line`.
- `pose_valid`/`force_valid`: 최근 0.5초 안에 받은 유효한 값일 때만 true입니다. false면 배열 값을 쓰지 않습니다.
- 위치·힘은 서비스로 차례로 조회해서 동시 측정이 아닙니다. 피드백은 약 5 Hz이며 통신에 따라 늦어질 수 있습니다.
- `progress`: 중간 진행률을 알 수 없으면 -1, 성공하면 1입니다.
- Result: `success`, `message`, 작업 프로세스 `exit_code`, `stop_confirmed`.

## 이동 중 취소

1. 액션 스레드는 작업 노드의 동기 이동 대기와 분리돼 있어서 이동 중에도 취소를 받습니다.
2. 작업 프로세스를 SIGSTOP으로 멈춰 다음 명령을 못 보내게 합니다.
3. 별도 서비스 클라이언트로 MoveStop(DR_HOLD)을 보내고 STANDBY를 확인합니다.
4. 작업 프로세스에 SIGINT/SIGCONT를 보내 노드의 정리 코드가 돌게 합니다. 끝나지 않으면 프로세스만 강제 종료합니다.
5. 로봇 정지를 다시 확인하고, **힘·컴플라이언스를 쓰는 작업(`apply`, `load`, `return`)**이면 ReleaseForce/ReleaseComplianceCtrl까지 확인합니다.
6. 정리가 모두 확인돼야 canceled를 반환합니다. 하나라도 실패하면 aborted를 반환하고 새 Goal을 막습니다.

> [!WARNING]
> 3·5단계의 정지 확인은 `get_robot_state == STANDBY`를 씁니다. 컴플라이언스 중에는 이 값이 이동 중에도 STANDBY로
> 나올 수 있어서(인수인계 기록), 작업 노드 안에서는 `check_motion`으로 바꿨지만 **서버의 취소 정리는 아직 바꾸지 않았습니다.**
> MoveStop 요청 자체는 보내므로 정지는 시도하지만, "정지 확인"이 실제보다 빨리 통과할 수 있습니다.

- 취소 요청 접수나 프로세스 종료를 로봇이 실제로 멈춘 것으로 취급하지 않습니다.
- 서버를 Ctrl+C/SIGTERM으로 끌 때도 ROS 연결을 유지한 채 같은 정리 절차를 수행합니다.
- 서버 SIGKILL, PC·네트워크 장애까지는 이 절차가 보장하지 않습니다. **비상정지를 항상 준비합니다.**
- 정리에 실패하면 로봇 상태를 확인한 뒤 서버를 수동으로 재시작합니다. 자동 재시도·자동 홈 복귀는 없습니다.

## HMI와 launch

- HMI 퍼티 탭에서 [퍼티 액션 서버]를 먼저 시작합니다. 각 작업 버튼은 Goal을 보내고 피드백을 로그·단계로 보여 줍니다. [정지]는 Action Cancel을 보냅니다.
- [퍼티 초기화·집기]는 초기화 성공 Result를 확인한 뒤 pickup Goal을 보냅니다.
- 남아 있는 launch:

| launch | 하는 일 | 비고 |
|---|---|---|
| `putty_actions.launch.py` | 액션 서버만 시작 | |
| `initialize_pickup.launch.py` | 초기화 → 집기 | `virtual_gripper:=false`로 실물 |
| `return_initialize.launch.py` | 반납 → 초기화 | 지금도 `--virtual-mode`로 고정돼 있음 (실물 반납은 클라이언트로 직접 요청) |
| `putty_full_cycle.launch.py` | 초기화 → 집기 → (`confirm_grasp:=true`면 터미널 y 파지 확인) → 뜨기 → 도포 → 반납 → 초기화 | `execute_motion:=true` 필수, `virtual_mode:=true`로 가상. 단계 실패 시 즉시 종료 |

launch는 기본으로 서버를 함께 띄웁니다. 이미 HMI나 다른 터미널에 서버가 있으면 `start_action_server:=false`를 줍니다.
예전 가상 전체 공정 `putty_cycle.launch.py`는 2026-11-19에 삭제했습니다.

## 검증 범위

- 로봇 없이 가짜 서비스로 확인한 것: 중간 취소, 중복 Goal 거절, 모드 불일치 차단, 정리 실패 시 aborted와 재요청 차단,
  실제 자식 프로세스 중단, 허용 목록, 집기·뜨기·도포·반납의 호출 순서와 movej 방향.
- 실물로 확인이 필요한 것: 취소 응답 시간·정지 거리, 도포 힘 형성, 반납 바닥 감지, 새 보관대·용기 좌표.
- 도포·묻히기는 팀 YAML의 특이점 회피 모드와 힘제어 `DR_SINGULARITY_ERROR` 정책을 설정하고 별도 스레드로 특이점 알람(3205)을 감시합니다.
  감지하면 정지·힘 해제만 하고 자동 회복 이동은 하지 않습니다.

## 팀 공통 Process 연결

같은 서버가 `rokey_interfaces/action/Process`를 두 이름으로 제공합니다. 팀 매니저용이며 DB 쓰기는 매니저가 담당합니다.

| Action | task_code | 하는 일 | 설정 |
|---|---|---|---|
| `/process/putty` | `PUTTY` | 헤라 파지·퍼티 준비가 끝난 상태에서 세로 상향 도포만 실행 | `config/robot.yaml`의 `force.putty_scrape` |
| `/process/putty_load` | `PUTTY_LOAD` | 판에 퍼티 묻히기 | `force.putty_load` (힘 등 미확정 항목이 null이면 거부) |

- `/process/putty`의 start_point/end_point는 BASE 날 끝 XYZ이고, X/Y가 같고 끝 Z가 더 커야 합니다.
- YAML의 `putty_scrape` 값(목표 3 N, X 강성 200, `press_method: direct`, `floor_lift`, `contact_confirm` 등)을 CLI 옵션으로 바꿔 `apply`를 실행합니다.
  2026-11-19에 이 새 옵션들이 허용 목록에 빠져 있어 **항상 거부되던 문제**를 고쳤습니다. 예전 6 N / 10 N 하한 충돌도 없어졌습니다.
- PuttyTask와 같은 실행 잠금·취소·정리 절차를 씁니다. 완료 시 피드백 1회와 Result를 돌려줍니다.
- Process를 가상 컨트롤러로 연결하려면 서버에 `--ros-args -p process_virtual_mode:=true`를 줍니다. 기본값 false는 실물 대상입니다.
