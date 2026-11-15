# rokey1 내부 퍼티 작업 모듈

코드 위치: `src/rokey1/rokey1/putty_demo/`.
별도 ROS 패키지가 아니라 `rokey1`의 Python 하위 패키지입니다.
워크스페이스 루트에서 다음과 같이 빌드하고 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rokey1 --symlink-install
source install/setup.bash
ros2 run rokey1 pickup_spatula --dry-run --transfer-speed 30 --pick-speed 10
ros2 run rokey1 demo_cycle --dry-run
```

집기점은 BASE `[589, 76, 152]` mm이며, 반납은 아직 기존 8번 기준 드라이런입니다.
실물 실행은 초기화와 집기만 구현되어 있습니다.

아래는 기존 사용 설명입니다.

# putty_demo 초기화 노드

M0609를 홈 관절각 `[0, 0, 90, 0, 90, 0]°`로 복귀시킨다. 이미 홈에 있으면 팔 이동을 건너뛴다.
홈 도착이 확인되면 기존 `rokey1/grip_test.py`와 같은 두산 제어함 디지털 출력 순서로 RG2를 닫는다.

| 동작 | 1번 | 2번 |
|---|---|---|
| 닫기 | ON | OFF |
| 열기 | OFF | ON |

초기화에서는 `1 OFF → 2 OFF → 1 ON → 2 OFF`를 실행한다.
각 출력 명령의 성공 응답을 확인한다. 초기화 완료 로그는 출력 설정 성공을 뜻한다.
RG2의 실제 닫힘 위치는 이 노드에서 읽지 않는다.

## 빌드

압축을 풀어 기존 `~/ws_cobot_pjt/ws_dsr/src/rokey1/rokey1/putty_demo`에 반영한다.
같은 이름의 패키지를 `src` 아래에 두 개 두지 않는다.
기존에 별도로 작성한 노드가 있다면 파일과 실행 등록을 병합한다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rokey1 --symlink-install
source install/setup.bash
```

## 실행

기본 명령은 로봇에 요청을 보내지 않고 순서를 출력한다.

```bash
ros2 run rokey1 initialize_device
```

가상 응답으로 초기화 로직을 실행하려면 빌드 후 다음 명령을 사용한다.

```bash
ros2 run rokey1 initialize_device --dry-run
ros2 run rokey1 initialize_device --dry-run --scenario home_not_reached
```

ROS 2를 설치하기 전에도 압축을 푼 패키지 루트에서 Python 3으로 실행할 수 있다.

```bash
python3 putty_demo/initialize_device.py --dry-run
```

기본 시나리오는 관절 조회, 홈 이동, 홈 도착 확인, RG2 닫기 순서로 실행한다.
`home_not_reached`는 이동 성공 응답 후에도 관절각이 홈과 달라 RG2 출력을 보내기 전에 종료한다.
두 시나리오 모두 가상 응답을 사용하며 ROS 2 서비스에 연결하지 않는다.

실물 실행은 기존 실습 순서대로 로봇을 연결한 뒤, RG2가 비어 있고 홈까지의 경로가 확보된 상태에서 한다.

```bash
ros2 run rokey1 initialize_device --ros-args -p execute_motion:=true
```

서비스 대기 로그가 계속되면 같은 터미널에서 `ros2 service list -t`로 표시된 서비스가 올라왔는지 확인한다.
관절 조회와 홈 이동은 `/dsr01/dsr_controller2/` 아래의 서비스를 사용한다.
`DR_init` 또는 `DSR_ROBOT2` import가 실패하면 기존 `rokey1/grip_test`에서 쓰는 두산 Python 경로 설정을 적용한다.

## 바닥 도포 미리보기

```bash
ros2 run rokey1 apply_putty --dry-run
ros2 run rokey1 apply_putty --rviz
```

RViz에서 Fixed Frame을 `base_link`로 설정하고 MarkerArray 디스플레이에
`/putty/application_preview` 토픽을 선택한다. 초록 선은 날 윗모서리 경로,
파란 선은 RG2 TCP 경로, 주황 선은 손잡이의 근사 모델이다.
다른 BASE 프레임 이름을 쓰는 환경에서는 `--frame-id`로 지정한다.

도포는 바닥 시험판 위에서 12번 → 11번으로 진행하고
날과 시험판 사이 각도를 12° → 2°로 낮춘다.
시험판 높이 35 mm와 RG2 TCP → 날 윗모서리 중앙 Tool XYZ [0, 0, 126] mm는
미리보기 기준값이다. `--surface-z`와 `--edge-offset X Y Z`로 변경할 수 있다.

ROS 2 없이 압축을 푼 패키지 루트에서 실행할 때는
`python3 putty_demo/apply_putty.py --dry-run`을 사용한다.
이 노드는 도포 구간의 경로와 자세를 계산해 보여준다.

## 헤라 반납 경로

```bash
ros2 run rokey1 return_spatula --dry-run
```

기본값은 BASE −Y로 75 mm 옮긴 도포 경로의 끝점, BASE 이송 높이
245.14 mm, 8번 위 반납점 [724.14, 73.22, 195.14] mm다.
도포 노드에서 시험판 높이나 날 오프셋을 바꿨다면 반납 드라이런에도
같은 `--surface-z`와 `--edge-offset X Y Z`를 전달한다.
반납 경로는 헤라를 놓은 뒤 수직 후퇴까지 계산한다. 홈 복귀는 초기화가 맡는다.
실제 로봇 명령과 관절각·충돌 검증은 포함하지 않는다.

ROS 2 없이 패키지 루트에서는 `python3 -m rokey1.putty_demo.return_spatula --dry-run`을 쓴다.

## 전체 공정 드라이런

```bash
ros2 run rokey1 demo_cycle --dry-run
ros2 run rokey1 demo_cycle --dry-run --scenario grasp_failure
ros2 run rokey1 demo_cycle --dry-run --scenario release_failure
```

초기화 → 집기 → 도포 시작점으로 이송 및 도포 → 8번에 반납 및 수직 후퇴
→ 기존 초기화로 홈 복귀 및 빈 RG2 닫기 순으로 실행한다.
도포에는 BASE −Y로 75 mm 이동한 경로를 사용한다.
TCP 자세, 가상 관절 조회, 그리퍼 출력 및 헤라 보유 상태를 한 프로세스에서 공유한다.
가상 잡힘 확인 실패 시 도포를 건너뛰고, 반납 출력 실패 시 재초기화 전에 종료한다.
ROS 서비스 통신, 실제 잡힘, 관절 경로와 충돌은 검증 범위 밖이다.
ROS 2 없이 패키지 루트에서 `python3 -m rokey1.putty_demo.demo_cycle --dry-run`으로 실행할 수 있다.

## 노드별 드라이런

```bash
ros2 run rokey1 initialize_device --dry-run
ros2 run rokey1 pickup_spatula --dry-run
ros2 run rokey1 apply_putty --dry-run
ros2 run rokey1 return_spatula --dry-run
ros2 run rokey1 demo_cycle --dry-run
```

노드 파일은 import → 설정/하이퍼파라미터 → 로봇 초기화 설정(해당 노드) →
main → 함수 정의 → 실행 진입점 순서로 작성한다. ROS 2와 두산 드라이버는
사용하는 실행 모드에서 불러온다. ROS 2 없이도 압축을 푼 패키지 루트에서
`python3 -m rokey1.putty_demo.demo_cycle --dry-run`과 같이 노드 모듈을 실행할 수 있다.
