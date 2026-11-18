# 퍼티 작업 노드

ROS 2 Jazzy / M0609 / RG2, 네임스페이스 `dsr01`, TCP `GripperDA_v1`을 사용합니다.
bringup의 real/virtual 연결에 이동 요청을 보내며 로그 전용 실행·가짜 응답 모드는 없습니다.
가상 집게 `--virtual-gripper`는 실제 가상 서비스를 호출하는 옵션으로 유지합니다.

현재 HMI/공정 launch는 `/dsr01/putty_task` Action으로 작업을 요청합니다.
이동 중 피드백·취소·실제 정지/힘 해제 확인 및 실행 예시는 [ACTIONS.md](ACTIONS.md)를 확인합니다.
아래 기존 `ros2 run initialize_device` 등은 수동 실행 호환 경로입니다.

## 도포 특이점 처리

`apply_putty_plane`은 이동 전에 다음 정책을 설정하고 응답 성공을 확인합니다.

- `motion/set_singularity_handling`: `SetSingularityHandling`, mode=0 (`DR_AVOID`, 자동 회피).
- `motion/set_singular_handling_force`: `SetSingularHandlingForce`, mode=0 (`DR_SINGULARITY_ERROR`).
  이전 로그의 mode=1(에러 무시)을 유지하지 않습니다.
- 별도 노드 `/dsr01/putty_singularity_monitor`가 `/dsr01/error`의 `RobotError`를 수신합니다.
  code=3205 영역 진입과 특이점 경고/에러를 감지하면 해당 실행을 중단합니다.
  명시적 Inside→Outside 알림만으로는 중단하지 않지만, 이미 감지한 위험을 자동 해제하지 않습니다.
- 동기 MoveLine/MoveSplineTask 서비스 응답 대기도 0.05초씩 나눠 알람을 확인합니다.
- 감지 후 정지 확인 → 활성 힘/컴플라이언스 해제 → 실패 결과 반환.
  자동 MoveJ 회복·재접근·재시도는 없습니다. Action 경로는 서버도 정리를 재확인합니다.
- 감시 publisher 없음/소실, 감시 스레드 중단 또는 정책 설정 실패 시 이동을 시작하지 않거나 중단합니다.

현재 V2 드라이버에는 `get_force_control_state` ROS 서비스가 없으므로 직접 위험도 0/1/2를
조회하는 구현은 아닙니다. 알람 부재는 현재 자세가 안전하다는 뜻이 아니며, 컨트롤러의
힘 제어 진입 시 특이점 에러 정책을 함께 사용합니다. 과거 최신 알람을 현재 상태로 오인하지 않습니다.
자동 회피는 경로를 바꿀 수 있으며, 이미 특이점인 자세에서 벗어나는 동작을 보장하지 않습니다.
이 설정만으로 6 N 형성 문제가 해결됐다고 볼 수 없습니다. 경로/힘 수치는 기존 그대로입니다.
소프트웨어 알람 감시의 지연은 통신/스케줄링에 영향을 받으며 실시간 보호 기능이 아닙니다.
정책은 컨트롤러에 적용되며 이전 값을 조회할 API가 없어 종료 시 임의 값으로 복원하지 않습니다.

## 실행 조건

- argparse 노드는 `--execute-motion`을 반드시 지정해야 합니다.
- 초기화 노드는 ROS 파라미터 `execute_motion:=true`가 필요합니다.
- 공정 launch도 `execute_motion:=true`가 없으면 오류로 종료합니다.
- 실행 요청 옵션을 생략해도 자동으로 움직이지 않습니다.
- 좌표·속도·각도·오프셋·높이 기본값은 기존 값을 유지했습니다.
- 초기화·픽업·측정 스쿱·단독 도포는 real/virtual 경로가 있습니다. 반납·기하 스쿱·전체 공정 launch는 가상 전용입니다.
- 가상 로봇 구동과 외부 카메라는 별개입니다. 이 패키지의 작업은 카메라를 사용하지 않습니다.

## 공통 환경

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rokey_interfaces rokey_team --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=10
```

기존 bringup을 별도 터미널에서 실행합니다. 코드나 launch는 컨트롤러를 새로 띄우지 않습니다.

## 초기화와 집기

집기·반납 TCP BASE XYZABC는
`[308.42,120.70,152.43,18.32,178.35,17.14]` (mm, degree)입니다.
홈 상승은 홈 자세를 유지하고, 보관 위치 위에서 집기 자세로 접근·하강·집기·상승합니다.
픽업은 XYZ 2 mm / 회전 2도 이내 도착을 확인합니다.

```bash
# 가상 집게 사용
ros2 run rokey_team initialize_device --ros-args -p execute_motion:=true -p virtual_gripper:=true
ros2 run rokey_team pickup_spatula --execute-motion --virtual-gripper

# 실물 집게: 디지털 출력 1·2번 제어, real bringup 및 장치 설정 확인 필요
ros2 run rokey_team initialize_device --ros-args -p execute_motion:=true -p virtual_gripper:=false
ros2 run rokey_team pickup_spatula --execute-motion
```

픽업 옵션: `--transfer-speed`, `--pick-speed`, `--clearance`, `--settle`.

## 퍼티 뜨기

```bash
ros2 run rokey_team scoop_putty --execute-motion --path geometry --scoop-angle 45
```

용기 바닥 중심 BASE `[723.83,-75.62,35.36]` mm, 내부 크기 120×120×50 mm 가정입니다.
용기가 가득 찬 표면 Z=85.36 mm에서 기본 3 mm 깊이까지 찍고 같은 각도로 수직 상승합니다.
목표 날 기준점은 용기 중심에서 +X로 전체 X폭의 1/4 지점입니다.
픽업 상태 퍼티 면 +X, 날 길이 +Z, TCP→날 Tool `[0,0,126]` mm 가정을 사용합니다.
새 집기 장착 상태에서 이 가정과 오프셋은 별도로 확인해야 합니다.
`--scoop-speed`, `--transfer-speed`, `--scoop-depth`, `--scoop-distance`,
`--container-center`, `--container-size`, `--edge-offset`으로 기존 옵션을 사용합니다.
`--apply-after`는 기존 16→15 도포까지 연결합니다. 별도 도포 노드를 실행한다면 생략합니다.

## 도포 케이스

접촉 탐색 실행 시 `[힘진단]` 로그가 바닥/벽 감지 후, 힘 설정 직전/응답 후,
힘 안정 대기 중, 도포 중, 힘 해제 전에 출력됩니다. 도포 중에는 기존 조회값을
재사용해 약 0.5초 이상 간격으로 출력합니다. 목표 +X 힘, 실측 BASE 힘/모멘트,
`|Fx|`, 목표−`|Fx|`, 실제 TCP XYZABC를 확인합니다. 위치/힘은 순차 조회입니다.
`계획날각`은 가까운 계획 경로점의 각도이며 실측 헤라 각도가 아닙니다.
`힘설정응답확인`은 요청 응답 확인이며 컨트롤러 활성 상태 직접 조회값이 아닙니다.
출발 전 힘 확인과 도포 중 힘 부족/초과 정지는 아래 최신 동작 절을 확인합니다.

독립 도포 실행 노드는 `apply_putty_plane`만 사용합니다.
이전 `apply_putty`, `apply_putty_j6` 실행 파일과 명령은 제거했습니다.
현재 디스크의 원본은 `src/rokey1_ws/putty_demo/apply_putty_plane.py`입니다.
공통 계산은 `putty_geometry.py`, ROS 통신은 `putty_motion.py`에 있으며 단독 실행 노드가 아닙니다.

```bash
# 현재 실물 YZ 벽: 바닥/벽 탐색 → 6 N 실측 안정 확인 → 힘 유지 상향 도포
ros2 run rokey_team apply_putty_plane --execute-motion --contact-search \
  --floor-force 2.3 --wall-force 2.3 --press-force 6

```

현재 실제 벽 좌표와 힘 제어 설정은 파일 위쪽 설정부 및 아래 최신 동작 절을 확인합니다.
가상 벽 홈과 가로 시험 케이스 및 `--case` 선택 옵션은 제거했습니다.

## 반납과 공정 launch

```bash
ros2 run rokey_team return_spatula --execute-motion --transfer-speed 30 --place-speed 10
ros2 launch rokey_team initialize_pickup.launch.py execute_motion:=true virtual_gripper:=true
ros2 launch rokey_team return_initialize.launch.py execute_motion:=true
ros2 launch rokey_team putty_cycle.launch.py execute_motion:=true
```

초기화·집기 launch는 `virtual_gripper:=false`로 실물 DO 경로를 선택할 수 있습니다.
반납·전체 공정 launch는 가상 전용입니다. 전체 공정은 초기화→집기→찍기→평면 도포→반납→홈이며
전체 공정 launch의 기존 도포 좌표와 속도는 유지했습니다.
각 단계 정상 종료 후 다음 단계로 진행하고 실패 시 후속 단계는 생략합니다.

## 공통 코드와 제한

서비스 대기는 `utils.service_waiter.ServiceWaiter`, 종료는 `utils.shutdown.safe_shutdown`입니다.
순수 경로 계산 함수는 실제 동작 생성과 단위 테스트에 사용하므로 유지합니다.
기존 가짜 상태 기반 `demo_cycle` 실행 항목은 제거했습니다.
흠 탐지·접촉력 제어·전체 충돌 검사·IK 사전 검증·구간 사이 퍼티 재공급은 미구현입니다.
타임아웃은 다음 요청을 중단하지만 로봇 이동 취소 명령을 자동으로 보내지는 않습니다.

## 실물 측정 자세 스쿱

기준 자세는 TCP BASE XYZABC [327.73,211.45,138.58,8.45,154.91,6.58]입니다.
작업자가 확인한 모서리 BASE Y축 평행과, 헤라 판 각도가 약 60°라는 추정값을 기준으로 합니다.
`--scoop-angle` 하나로 헤라 판 기울기와 +X/-Z 사선 진입각을 함께 지정합니다.
`--entry-angle`은 같은 인자의 호환 별칭이며 두 옵션을 따로 사용할 필요가 없습니다.

현재 장착에서 날 길이가 -X 쪽으로 기울고 손잡이가 위인 가정으로,
목표각 - 기준각만큼 BASE Y축 회전합니다. 60→45°는 -15° 회전입니다.
Y축을 중심으로 회전하므로 이미 Y축과 평행하다고 확인한 모서리 방향은 유지됩니다.
TCP→날 Tool 오프셋 [0,0,126] mm를 사용해 측정 당시 날 기준점이 유지되도록
TCP XYZ도 함께 보정합니다. 126 mm 오프셋과 60°는 새로 실측된 값이 아니므로
계산 결과가 실제로 정확한 45°가 됨을 보장하지 않습니다.

기본 45° 기준 보정 찍기 TCP는 약 [300.00,211.45,121.01] mm,
ABC는 약 [5.56,140.02,3.19]°입니다. 원래 TCP Z보다 약 17.57 mm 낮습니다.
진입 시작점은 보정 목표에서 X -120 / Z +120 mm이며,
찍은 뒤에는 목표 XY와 보정 ABC를 유지해 Z +120 mm 수직 상승합니다.
기존 속도·가속도·최소 이송 높이·원본 측정 자세는 변경하지 않았습니다.

```bash
ros2 run rokey_team scoop_putty --execute-motion --path taught \
  --scoop-angle 45 --entry-height 120 --lift 120
```

real에서는 보정 TCP와 추정 기준각을 먼저 출력하고 헤라 파지 확인 `y`를 받은 뒤 이동합니다.
자동 파지 감지는 아닙니다. 각 이동 후 XYZ 2 mm / 자세 2° 이내 도착을 확인합니다.
접촉력·충돌·IK는 검증하지 않습니다. `--reference-blade-angle`은 현재 측정 ABC의
기준 기울기이고 목표각이 아닙니다. 측정 자세를 다시 잡으면 이 기준도 다시 맞춰야 합니다.
`--apply-after`는 측정 모드에서 허용하지 않으며 도포/반납 실물 차단은 유지합니다.
전체 가상 런치는 기존 경로 보존을 위해 명시적으로 `--path geometry`를 사용합니다.

## 평면 도포 기울기 변화

`apply_putty_plane`은 시작 `--blade-angle` (기존 기본 45°)에서
종료 `--blade-end-angle` (30°)까지 부드럽게 기울기를 바꿉니다.
`--samples` (21개)마다 날 기준점과 회전 행렬을 계산하고 TCP XYZ/ABC를 보정합니다.
지정한 날 접촉점들은 원래 도포 선과 면 위에 유지됩니다. 샘플 사이 실제 경로·접촉력은
별도 검증이 필요합니다. 도포 목표들은 한 번의 MoveSplineTask로 실행합니다.
면 접근은 시작각, 도포 후 법선 후퇴는 종료각을 유지합니다.
동일 각도로 바르려면 두 각도를 같은 값으로 명시합니다.
전체 launch의 plane 케이스에서도 `blade_end_angle:=30`을 전달할 수 있습니다.

```bash
# YZ 벽 하단→상단: 실제 좌표를 받은 뒤 start/end를 입력
# --plane yz --start X Y Z_LOW --end X Y Z_HIGH
# --blade-angle 45 --blade-end-angle 30
```

속도 기본값은 유지합니다. 현재 평면 노드는 real/virtual 실행을 허용합니다.

## 측정 YZ 벽의 상향 도포 기본값

단일 `plane`의 기본 작업면은 YZ이며, 접촉 시작점은 BASE 날 기준
[784.04,-15.52,32.20] mm입니다. `end`를 생략하면 `--stroke-height` (180 mm)만큼
+Z로 올라간 [784.04,-15.52,212.20] mm를 끝점으로 계산합니다.
접촉점 X·Y를 유지하면서 벽 기준 기울기를 45→30°로 바꿉니다.
TCP XYZ는 기존 날 오프셋으로 계산하므로 접촉점 좌표와 다릅니다.
`--start`/`--end`를 직접 주면 직접 지정한 좌표를 사용합니다.


```bash
ros2 run rokey_team apply_putty_plane --execute-motion \
  --stroke-height 180 --blade-angle 45 --blade-end-angle 30
```

현재 real/virtual 모두 지원합니다. positive는 +X 쪽 접근점에서 -X로 접근,
negative는 -X 쪽 접근점에서 +X로 접근합니다. 이 YZ 벽은 negative가 기본입니다.
모델의 장착 방향·오프셋 가정과 실제 헤라 일치, 경로 도달 및 충돌은 검증하지 않았습니다.

## 팀 TCP 적용 변경 반영

픽업은 현재 TCP를 조회하고 `GripperDA_v1`이 아니면 `SetCurrentTcp`로 적용한 뒤
재조회합니다. 적용 실패나 재조회 불일치 시 이동을 시작하지 않습니다.
이 처리는 펜던트에 이미 등록된 TCP를 선택하며 TCP 오프셋을 새로 정의하지 않습니다.
새 집기 좌표·ABC, 속도, 회전 도착 판정, 가상 집게와 실행 요청 조건은 그대로 유지합니다.
팀 변경에 포함된 HMI 개선(현재 위치 입력·속도 조절·홈 버튼)과 homeset 노드도 가져왔으며,
HMI의 퍼티 가상 모드는 가짜 응답 대신 실제 가상 서비스 실행을 유지합니다.

## 평면 도포의 J6 단계 제거

`apply_putty_plane`은 J6 회전·J6 전용 사전 상승·관절 서비스 대기를 사용하지 않습니다.
현재 TCP를 조회하고 평면 도포 경로의 높이 확보·자세 정렬부터 바로 진행합니다.
평면 노드의 `--j6-delta`, `--j6-speed`, `--j6-acceleration` 옵션은 제거했습니다.
평면 도포 시 실행 명령에서 해당 인자를 빼야 합니다.
도포 자세 정렬과 45→30° 변화는 유지하므로 개별 관절이 고정된다는 뜻은 아닙니다.
별도 J6 케이스는 이후 제거됐고 전체 launch도 plane만 지원합니다.

## YZ 실물 벽 접근 방향 확정

측정 벽은 X가 증가하는 방향으로 접근합니다. 단일 YZ plane에서 normal-sign을
생략하면 negative (자유 공간 -X)로 설정합니다. 접근점은 접촉 TCP보다 X가 50 mm
작고, 도포 후에는 -X로 50 mm 후퇴합니다. 다른 기존 시험 케이스는 기존 positive를 유지합니다.
평면 도포만 real/virtual 실행을 허용하며 J6 별도 케이스의 실물 차단은 유지합니다.

```bash
ros2 run rokey_team apply_putty_plane --execute-motion
```

현재 측정 벽 시작점은 [784.04,-15.52,32.20]이며, 닫힌 그리퍼의 GripperDA_v1 TCP로
실제 벽에 닿아 측정한 표면점임을 확인했습니다. Tool +Z 헤라 길이는 약 120 mm입니다. 실물 경로·충돌·접촉력은 검증하지 않았으며
현재는 위치 제어이고 목표 힘(N) 제어는 없습니다.

## 연속 도포 및 최신 벽 측정 반영

이번 벽은 헤라 없이 닫힌 그리퍼의 GripperDA_v1 TCP로 실제 벽에 닿아 측정했습니다.
- 측정 시작 XYZABC: [784.04,-15.52,32.20,178.14,-129.12,176.45]
- 측정 끝 XYZABC: [787.13,-26.18,197.39,176.01,-127.98,175.61]
- 두 점의 Z 차이: 165.19 mm. 사용자 지정 벽 높이/도포 길이는 180 mm.

정확한 +Z 상향 도포를 위해 시작점의 X=784.04, Y=-15.52를 고정하고
Z=32.20→212.20 mm로 경로를 계산합니다. 측정 끝점의 XYZABC는 경로 끝점으로 직접 쓰지 않습니다.
이 ABC는 헤라가 없는 벽 측정 자세이며, 퍼티 도포 ABC로 고정하지 않습니다.
헤라 길이는 사용자 확인에 따라 Tool [0,0,120] mm로 보정합니다. BASE -Z에 120을
상수로 더하거나 빼지 않고 각 시점의 회전 행렬로 변환한 오프셋을 뺍니다.
헤라 면/폭 장착 방향의 기존 모델 가정은 여전히 적용되므로 실물 일치는 별도 확인해야 합니다.

각도는 공간 진행률 u에 대해 `45+(30-45)*u³`입니다.
중간 높이(u=0.5)에서 43.125°, 끝에서 30°이며 초반 완만·후반 빠른 변화입니다.
한 도포 구간의 시작점 포함 21개 TCP 자세를 MoveSplineTask 한 요청으로 보냅니다.
기존 점별 MoveLine 완료 대기와 중간 좌표 조회로 발생하는 정지를 제거했습니다.
접근/후퇴는 기존 MoveLine을 유지하며 두 구간 사이 후퇴는 합치지 않습니다.
스플라인 완료 후 끝점 오차를 검사하고, 실패 시 후퇴를 포함한 다음 요청을 보내지 않습니다.
컨트롤러의 스플라인 보간에 따른 실제 접촉점 편차/속도는 실물 검증 전입니다.
도포 samples 범위는 드라이버의 MAX_SPLINE_POINT=100에 맞춰 2~100입니다.

```bash
source install/setup.bash
ros2 run rokey_team apply_putty_plane --execute-motion
```

기존 이송/도포 속도 및 가속도 값은 유지했습니다. J6 회전은 없습니다.
힘 제어가 아닌 위치 제어이므로 실제 접촉력이나 벽 접촉 유지가 자동 보장되지는 않습니다.

## 벽 위치 소량 보정

사용자 요청으로 기본 접촉 경로 전체를 BASE X +5 mm / Z -5 mm 이동했습니다.
원 측정점 [784.04,-15.52,32.20]은 기록으로 유지하며, 실행 시작점은
[789.04,-15.52,27.20], 끝점은 [789.04,-15.52,207.20] mm입니다.
도포 길이 180 mm, Y, 각도 변화, 속도, Tool 오프셋은 유지했습니다.
이 보정은 위치 보정이며 힘 제어 또는 접촉 검증을 의미하지 않습니다.

## 도포 전 −Z / +X 접촉 탐색

`--contact-search` 옵션은 기본 위치까지 기존 경로로 이동한 뒤,
−Z 바닥 탐색 → 힘 변화 감지·정지 → +X 벽 탐색 → 힘 변화 감지·정지 → 도포 순서로 실행합니다.
각 탐색은 시작 시 BASE 외력을 읽고 해당 축 외력 변화의 절댓값을 N 단위로 비교합니다.
힘 기준값은 `--floor-force`와 `--wall-force`에 반드시 입력해야 하며 기본값은 없습니다.
입력 누락·오류이면 ROS 초기화나 이동 전에 종료합니다.

사용자가 지정한 축별 최대 거리 기본값은 20 mm, 탐색 속도 기본값은 2 mm/s입니다.
속도·거리는 `--search-speed`/`--search-distance`로 명시할 수 있습니다.
접촉 탐색은 유한 거리 비동기 MoveLine 중 BASE 외력을 조회하고 감지 시 MoveStop(HOLD)을
호출합니다. STANDBY가 확인된 다음에만 다음 단계를 진행합니다.
거리 안에서 미감지, 시간 초과, 센서 오류, 서비스 실패 시 정지를 시도하고 도포는 생략합니다.
두 접촉 확인 후 실제 정지 TCP와 기존 접촉 TCP의 차이만큼 남은 도포·후퇴 경로를 이동합니다.
+Z 도포 길이·각도 변화·연속 스플라인 방식·도포 속도는 유지합니다.

이것은 힘 감시 기반 접촉 탐색이며 목표 힘 유지/컴플라이언스 제어가 아닙니다.
현재는 단일 YZ 벽 +X 접근, X/Y 고정 상향 도포에서만 지원합니다.
실물 접촉 성능·안전 정지 거리·힘 센서 오차는 검증 전입니다.
힘을 적용하기 전에 정지 상태에서 현재 외력값을 조회할 수 있습니다:

```bash
ros2 service call /dsr01/dsr_controller2/aux_control/get_tool_force \
  dsr_msgs2/srv/GetToolForce '{ref: 0}'
```

실행 시 기존 명령에 `--contact-search --floor-force <확인한_N> --wall-force <확인한_N>`을
추가합니다. 예시 N 값을 임의 기본값으로 사용하지 않습니다.

## 도포 중 BASE +X 2.3 N 유지

접촉 탐색 성공 후에도 도포 중 벽을 누르도록 연결했습니다.
`--contact-search`를 사용하면 바닥·벽 탐색 성공 후 컴플라이언스와 X축 목표 힘 제어를
시작합니다. 기본 유지 힘은 사용자가 지정한 2.3 N (`--press-force`)입니다.
벽 탐색 기준 `--wall-force`와 도포 유지 목표 `--press-force`는 별개입니다.
BASE 힘 벡터 [2.3,0,0,0,0,0], 방향 마스크 [1,0,0,0,0,0], 절대 힘 모드(0)를 사용합니다.
기존 접촉력에 2.3 N을 더하는 상대 모드가 아닙니다.
컴플라이언스 강성은 드라이버의 기본값 [3000,3000,3000,200,200,200]이며
`--force-stiffness`로 지정할 수 있습니다. 좌표·도포 속도·각도 곡선은 변경하지 않았습니다.

도포 스플라인은 비동기로 실행하며 BASE 외력 데이터와 현재 자세·상태를 감시합니다.
X는 힘 제어에 맡겨 목표 X 좌표와의 오차로 완료 여부를 판정하지 않습니다.
Y/Z와 각도는 기존 오차 기준으로 확인합니다. 각도 변경에 필요한 정상 TCP X 이동을
고려하여, 현재 Y/Z에 가장 가까운 계획점의 X에서 search-distance (기본 20 mm)를
초과하는 편차가 나면 정지·해제 후 중단합니다.
완료 후 목표 힘→컴플라이언스 순서로 해제하고 실제 X 위치를 기준으로 -X 후퇴합니다.
오류·타임아웃·Ctrl+C 시에는 정지를 시도한 뒤 힘/컴플라이언스 해제를 모두 시도합니다.
해제 실패 시 후퇴를 포함한 다음 이동을 진행하지 않습니다.

기존에 성공한 접촉 탐색 명령의 floor-force/wall-force 값을 유지하고,
필요하면 `--press-force 2.3`을 명시합니다. contact-search를 생략하면 기존 위치 제어입니다.
2.3 N 감지에 성공한 사실이 2.3 N 목표 유지 성능을 보장하지는 않습니다.
컨트롤러가 목표 힘을 거부하면 중단하며 임의로 더 큰 힘을 재시도하지 않습니다.
실물 힘 유지/벽 접촉 성능은 아직 실행 검증하지 않았습니다.

## 최신 동작: 추가 접근 제거·6 N 출발 확인·도포 중 감시

순서: −Z 바닥 감지 → +X 벽 감지 → 정지 상태에서 컴플라이언스와 +X 6 N 설정
→ 실측 힘 안정 확인 → 45→30° 선형 각도 변화로 180 mm 상향 도포.
벽 감지 후 강제로 +X 몇 mm를 더 이동하던 함수와 `--extra-approach` 옵션은 삭제했습니다.
접촉 시작점의 기존 X +5 / Z −5 좌표 보정과 Tool 오프셋 기하 보정은 유지합니다.

새 실측 감시 기준은 다음과 같습니다. 물리적 유지 정확도는 실물 검증 전입니다.

- `--press-force 6`: 기존 BASE +X 절대 목표 힘. 감지 기준에 더하는 상대 모드가 아닙니다.
- `--force-tolerance 1`: |BASE Fx|가 5~7 N 범위일 때 정상 판정.
- `--force-stable-time 0.5`: 같은 실측 부호로 범위 안에서 0.5초 이상·3회 이상 확인 후 출발.
- `--force-wait-timeout 30`: 30초 내 출발 조건 미충족 시 정지·해제, 상승하지 않음.
- `--force-loss-timeout 0.5`: 도포 중 5 N 미만이 0.5초 지속되면 정지·해제.
- 7 N 초과, 안정 후 반대 부호의 5~7 N, 센서 오류, 비정상 로봇 상태도 중단.
- 힘 형성 중 벽 감지 위치에서 X 이동 한도는 기존 `--search-distance 20` mm,
  YZ 위치 변화 한도는 기존 도착 허용 오차인 2 mm입니다.

명령은 항상 BASE +X입니다. 측정은 반력 부호를 단정하지 않고 |Fx|로 크기를 확인하되
원래 부호를 로그에 남깁니다. 출발 시 실측 부호를 고정해 도포 중 반전을 감시합니다.
이 판정은 툴 무게·무게중심 설정 오차나 정확한 접촉 위치까지 검증하지 않습니다.
힘 조절은 컨트롤러가 계속 수행하며, Python은 서비스 조회 주기마다 상태를 감시합니다.
즉 이 감시는 실시간 보호 기능이 아니며 샘플 사이의 편차까지 보장하지 않습니다.
별도 소프트웨어 위치 보정이나 힘 증가는 수행하지 않습니다.

출발점은 힘 형성 후 실제 TCP로 다시 정렬합니다. 도포 중 X는 힘 제어에 맡기고,
Y/Z와 자세는 기존 스플라인으로 추종합니다. 강성 [3000,3000,3000,200,200,200]은 유지합니다.
끝점까지 힘 명령을 유지한 뒤 해제하고 실제 X에서 −X 후퇴합니다.
실패 시 정지·힘/컴플라이언스 해제를 시도하며 자동 후퇴나 재시도는 하지 않습니다.
접촉 탐색 옵션을 빼면 기존 위치 제어로 동작합니다.

## 독립 도포 노드 정리

사용자 요청으로 apply_putty.py / apply_putty_j6.py는 삭제했습니다.
좌표·각도·속도·접촉 탐색·6 N 힘 유지 설정은 유지하고 공통 함수만 내부 모듈로 옮겼습니다.
전체 launch의 도포 케이스 선택 옵션은 제거했습니다.
기존 전체 launch의 나머지 좌표·속도 설정은 바꾸지 않았으므로 현재 실물 벽 작업에는
위 단독 평면 도포 명령을 사용합니다. 독립 J6 테스트도 제거했습니다.

## 코드 역할과 통신 읽는 순서

실행 노드: initialize_device(초기화), pickup_spatula(집기), scoop_putty(찍기),
apply_putty_plane(도포), return_spatula(반납)입니다.
공통 모듈 contact_search / putty_geometry / putty_motion은 따로 실행하는 ROS 노드가 아닙니다.

평면 도포를 읽을 때는 다음 순서가 이해하기 쉽습니다.
1. apply_putty_plane.main: 옵션 검증과 실행 시작
2. surface_frame / stroke_poses / application_moves: 날 경로와 TCP 경로 계산
3. putty_motion.run_robot: 노드 생성과 ROS 서비스 통신
4. contact_search.seek_floor_then_wall: io 함수를 재사용해 접촉 탐색
5. WallForceHold 및 putty_motion의 스플라인 실행: 힘 설정과 상향 도포

모듈끼리는 일반 Python import/함수 호출입니다. ROS 토픽이나 Action을 주고받지 않습니다.
각 실행 노드는 /dsr01/dsr_controller2 아래 ROS 2 서비스를 호출합니다.
DSR 서비스 서버는 dsr_msgs2 요청을 받아 DRFL API를 호출해 로봇 컨트롤러로 전달합니다.
가상 집게는 별도 /onrobot/sendCommand 서비스(onrobot_rg_msgs/srv/SetCommand)를 사용합니다.
실물 초기화/픽업의 집게 제어는 컨트롤박스 DO 1·2번 서비스를 사용합니다.

주요 서비스 인터페이스:
- GetCurrentPosx / GetCurrentPosj: 현재 TCP XYZABC / 관절각 조회
- GetCurrentTcp / SetCurrentTcp: 등록된 TCP 선택 상태 조회 / 변경
- MoveJoint: 관절 좌표 이동 (홈)
- MoveLine: TCP 직선 이동 (접근, 탐색, 후퇴)
- MoveSplineTask: 여러 TCP 자세를 하나의 연속 도포로 실행
- GetToolForce / MoveStop / GetRobotState: 외력 조회 / 정지 요청 / 정지 확인
- TaskComplianceCtrl / SetDesiredForce: 컴플라이언스 / 목표 힘 설정
- ReleaseForce / ReleaseComplianceCtrl: 목표 힘 / 컴플라이언스 해제

call_async는 서비스 요청을 보내고 Future를 반환합니다. 이어지는 spin으로 응답을 기다립니다.
이 Python 전송 방식과 이동 요청의 sync_type(로봇 동작 완료 대기 여부)은 다른 개념입니다.
GetToolForce 조회 성공과 목표 힘 유지 성공은 같지 않습니다. 현재는 상승 전 실측 힘 안정 대기와 도포 중 감시를 수행합니다.

launch는 PuttyTask 결과를 받은 ActionClient의 종료 코드를 확인한 뒤 다음 Goal을 보냅니다.
HMI는 직접 ActionClient로 결과·피드백을 받습니다. 내부 작업 노드의 서비스 호출 구조는 유지합니다.
전체 공정은 가상 전용이며 현재 실물 벽 도포와 구분해야 합니다.
