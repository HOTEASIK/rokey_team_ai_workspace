# 퍼티 작업 Action 사용법

## 구조

```text
HMI / putty_task_client / 다른 팀 공정 노드
        ↕ PuttyTask Action: Goal / Feedback / Cancel / Result
/dsr01/putty_task (putty_task_server)
        ↓ 기존 작업 노드를 격리 실행
initialize_device / pickup_spatula / scoop_putty / apply_putty_plane / return_spatula
        ↕ dsr_msgs2 서비스
/dsr01/dsr_controller2 → M0609 / RG2
```

인터페이스는 `rokey_interfaces/action/PuttyTask`입니다. 샌딩의 기존 `ToolTask`는 변경하지 않습니다.
기존 수동 CLI도 남아 있지만 **HMI와 공정 launch는 PuttyTask를 사용**합니다.
동일 로봇에 수동 CLI와 액션/다른 제어 앱을 동시에 실행하지 않습니다.
서버는 한 Goal씩 실행하며 동시 Goal을 거절합니다. 같은 PC/domain의 중복 서버도 차단합니다.

## 빌드 및 시작

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rokey_interfaces rokey1 --symlink-install
source install/setup.bash
ros2 run rokey1 putty_task_server
```

서버 시작만으로는 움직이지 않습니다. 기존 bringup과 같은 ROS domain을 사용합니다.
서버 실행 대신 `ros2 launch rokey1 putty_actions.launch.py`도 사용할 수 있습니다.

별도 터미널에서 작업을 요청합니다. 아래 명령은 real 모드 요청입니다.

```bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 run rokey1 putty_task_client initialize
ros2 run rokey1 putty_task_client pickup -- --transfer-speed 15 --pick-speed 5
ros2 run rokey1 putty_task_client scoop --tool-held-confirmed -- --scoop-speed 3
ros2 run rokey1 putty_task_client apply -- \
  --floor-force 2.3 --wall-force 2.3 --press-force 6
```

`--` 뒤는 작업별 기존 설정 옵션입니다. 수치 기본값/경로/힘 감시 로직은 변경하지 않습니다.
`--execute-motion`, `--ros-args`, `--virtual-gripper`는 서버가 관리하므로 Goal 인자로 보내지 않습니다.
실물 scoop의 `--tool-held-confirmed`는 사용자가 헤라 파지를 확인했다는 뜻이며 센서 판정이 아닙니다.

가상 모드는 작업명 뒤에 `--virtual-mode`를 붙입니다. 실제 연결된 GetRobotSystem 응답과
Goal 모드가 다르면 작업을 시작하지 않습니다. 반납은 현재 가상 전용입니다.

```bash
ros2 run rokey1 putty_task_client return --virtual-mode
ros2 run rokey1 putty_task_client initialize --virtual-mode
```

다른 ROS 노드 또는 CLI에서도 직접 Goal을 보낼 수 있습니다.

```bash
ros2 action send_goal /dsr01/putty_task rokey_interfaces/action/PuttyTask \
  "{task: apply, virtual_mode: false, tool_held_confirmed: false, arguments: ['--floor-force', '2.3', '--wall-force', '2.3', '--press-force', '6']}" \
  --feedback
```

중간 취소는 전용 `putty_task_client`에서 Ctrl+C를 사용하거나,
다른 ActionClient에서 수락된 GoalHandle의 `cancel_goal_async()`를 사용합니다.
일반 `ros2 action send_goal` 명령의 Ctrl+C만으로 작업이 취소됐다고 가정하지 않습니다.

## 피드백과 결과

- Goal: `task`(initialize/pickup/scoop/apply/return), `virtual_mode`, `tool_held_confirmed`, `arguments`.
- Feedback: 현재 `stage`, `elapsed_sec`, BASE `tcp_pose[6]`, BASE `tool_force[6]`, 원본 `log_line`.
- `pose_valid`/`force_valid`: 최근 유효한 조회값만 true. false인 배열값은 사용하지 않습니다.
- 위치/힘은 서비스로 순차 조회되므로 동시 측정이 아닙니다. 피드백은 약 5 Hz이며 통신에 따라 지연될 수 있습니다.
- `progress`: 중간 진행률을 정확히 알 수 없으면 -1, 성공 시 1. 임의의 퍼센트를 만들지 않습니다.
- Result: `success`, `message`, 작업 프로세스 `exit_code`, `stop_confirmed`.

Action 내부 ROS 엔드포인트는 `/_action/feedback`, `/_action/status` 토픽과
`/_action/send_goal`, `/_action/get_result`, `/_action/cancel_goal` 서비스입니다.
모두 `/dsr01/putty_task` 뒤에 붙습니다. 드라이버 서비스 인터페이스는 그대로입니다.

## 이동 중 취소

1. 액션 스레드는 기존 동기 MoveLine/MoveJoint 응답 대기와 분리되어 취소를 수신합니다.
2. 작업 프로세스를 SIGSTOP으로 멈춰 다음 단계 요청을 막습니다.
3. 별도 서비스 클라이언트로 MoveStop(DR_HOLD)을 요청하고 STANDBY를 확인합니다.
4. 작업 프로세스에 SIGINT/SIGCONT를 보내 종료·finally 정리를 허용합니다. 종료되지 않으면 프로세스만 강제 종료합니다.
5. 로봇 정지를 다시 확인하고, 도포 작업이면 ReleaseForce/ReleaseComplianceCtrl도 확인합니다.
6. 정리가 확인된 경우에만 canceled를 반환합니다. 확인 실패는 aborted이며 새 Goal을 차단합니다.

프로세스 종료/취소 요청 접수 자체를 물리 로봇 정지 성공으로 취급하지 않습니다.
서버 Ctrl+C/SIGTERM에도 ROS context를 유지한 채 같은 정리 절차를 수행합니다.
서버 SIGKILL/PC·네트워크 장애까지 이 소프트웨어 절차가 보장하는 것은 아닙니다.
정리 실패 시 로봇 상태를 확인한 후 서버를 수동 재시작합니다. 자동 재시도·자동 홈 복귀는 없습니다.

## HMI와 launch

HMI의 퍼티 탭에서 [퍼티 액션 서버]를 먼저 시작합니다. 기존 초기화·집기와 새 뜨기·도포·반납은
직접 Goal을 전송하며 피드백을 로그/단계 표시로 받습니다. [정지]는 Action Cancel을 보냅니다.
초기화·집기 연속 항목은 초기화 성공 Result를 확인한 뒤 pickup Goal을 보냅니다.

기존 initialize_pickup / return_initialize / putty_cycle launch도 ActionClient를 사용합니다.
각 클라이언트가 Action Result를 확인한 뒤 종료하므로, OnProcessExit는 다음 Goal 전송 시점을 연결합니다.
기본적으로 서버를 함께 시작합니다. 이미 HMI/다른 터미널에 서버가 있으면
`start_action_server:=false`를 지정합니다. 초기화·집기/반납 launch의 서버는 작업 후에도 대기합니다.
전체 putty_cycle은 기존 가상 전용 기하 스쿱/XY 좌표를 보존했으며 현재 실물 YZ 벽 자동 공정은 아닙니다.

## 검증 범위

로봇을 움직이지 않는 테스트로 중간 취소, 중복 Goal 거절, 모드 불일치 차단,
정지/힘 해제 실패 시 aborted 및 재요청 차단, 실제 자식 프로세스 중단을 확인합니다.
실물 취소 응답시간/정지 거리와 연속 도포 성능은 별도 검증이 필요합니다.
도포 작업은 DR_AVOID와 힘 제어 DR_SINGULARITY_ERROR 정책을 설정하고 별도 스레드로
RobotError 특이점 진입 알람을 감시합니다. 동기 이동 중에도 감지하면 중단합니다.
자동 MoveJ 회복은 없습니다. 설정과 감시 방법의 제한은 README의 도포 특이점 처리 절을 확인합니다.
접촉 자세에서 확인된 특이점 영역 진입 및 6 N 형성 실패가 해결됐는지는 실물 재검증이 필요합니다.

## 팀 공통 Process 연결 (8acf6ca)

같은 서버가 `/process/putty`의 `rokey_interfaces/action/Process`도 제공합니다.
팀 매니저용이며 헤라 파지·퍼티 준비가 끝난 상태에서 세로 상향 도포만 실행합니다.
start_point/end_point는 BASE 접촉 모서리 XYZ이고 X/Y 고정, 끝 Z > 시작 Z가 필요합니다.
공통 설정은 `config/robot.yaml`에서 읽습니다. 기존 PuttyTask와 실행 잠금 및 취소/정지 절차를 공유합니다.
완료 시 putty_scrape 피드백 1회와 결과를 반환하며 DB 쓰기는 팀 manager가 담당합니다.

현재 기본 힘 6 N과 현장 함수의 최소 10 N 검사가 충돌하므로 위의 6 N 예시는
현 상태에서는 이동 전에 거부됩니다. 수치는 자동 변경하지 않았습니다.
Process 가상 연결이 필요하면 서버에 `--ros-args -p process_virtual_mode:=true`를 지정합니다.
기본값 false는 실물 컨트롤러 대상입니다. 서버/manager 시작과 실제 작업 요청은 구분합니다.
