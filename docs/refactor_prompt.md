# 역할
너는 ROS2(Jazzy) + Python 협동로봇 프로젝트의 리팩터링 담당이야.
팀원들이 폴더(기능) 단위로 나눠 작업한 코드를 병합하면서, 중복 로직을 공통화하고 코드 스타일을 통일하려고 해.

# 작업 대상
- 패키지 경로: /home/rokey/rokey_team_ai_workspace/협동-1_ROS2_자동화공정/ws_cobot_pjt/ws_dsr/src/rokey1
- git 저장소 루트: /home/rokey/rokey_team_ai_workspace (현재 브랜치: develop)
- 스타일 기준 참고 경로 (읽기만 함): /home/rokey/ws_cobot_pjt/ws_dsr/src/rokey/rokey (move.py, simple_input.py)
- 이 rokey1 패키지만 수정해. doosan-robot2, onrobot-ros2, 참고 경로의 rokey 패키지는 읽기만 하고 수정하지 마.
- 코드를 붙여넣지 않으니 파일을 직접 읽고 분석해.

# 환경
- Ubuntu 24.04, ROS2 Jazzy, ament_python 패키지 (setup.py의 entry_points로 노드 등록)
- 로봇: Doosan M0609 (DR_init / DSR_ROBOT2 / DR_common2 API, namespace "dsr01")
- 그리퍼: OnRobot RG2
- 빌드: ~/ws_cobot_pjt/ws_dsr 기준 `colcon build --packages-select rokey1`

# 폴더 구조와 책임 (섞이지 않게 유지)
- rokey1/     : 그리퍼·기어 조립 작업 (move, grip_test, sun_gear)
- navigation/ : 벽·테이블 탐색 및 측정, RViz 시각화 (wall_navigation, wall_modeling, wall_rviz, nhj_internal-navigation/)
- tape/       : 테이프 위치 탐지·떼기 (tape_locate, tape_peel)
- utils/      : 전역 공통 모듈 (service_waiter, template, 이번에 추가할 공통 클래스)
→ 두 개 이상의 폴더에서 쓰이는 코드만 utils/로 올리고, 한 폴더 안에서만 반복되는 코드는 그 폴더 안의 하위 파일(예: navigation/common.py)로 분리해.

# 작업 항목

## [A] 서비스 대기 → utils/service_waiter.py의 ServiceWaiter 클래스로 통일
- ServiceWaiter는 서비스 대기 코드를 import해서 쓰려고 만든 공통 클래스야. ServiceWaiter 자체는 수정하지 마.
- 같은 대기 루프(`while not cli.wait_for_service(timeout_sec=1.0)`)를 직접 구현한 곳을 `ServiceWaiter(node).wait(...)` 호출로 바꿔:
  rokey1/move.py, rokey1/grip_test.py, rokey1/sun_gear.py(wait_for_services), navigation/wall_navigation.py(RobotIO.wait_for_services)
- 기다릴 서비스 목록은 파일마다 다르고(mode, ENABLE_FORCE, ENABLE_PERIODIC 같은 조건에 따라 달라짐) 그대로 유지해.
  "어떤 클라이언트를 기다릴지"는 각 파일에 남기고, "기다리는 방식"만 ServiceWaiter로 바꿔.

## [B] MoveStop 클라이언트 → utils/에 클래스로 신설
- `node.create_client(MoveStop, 'dsr_controller2/motion/move_stop')`와 그 호출 코드가
  tape/tape_peel.py, tape/tape_locate.py, navigation/wall_navigation.py에 반복돼.
- ServiceWaiter와 같은 스타일(생성자에서 node를 받고, docstring에 사용 예시 포함)로 utils/move_stop.py에 클래스(예: MoveStopClient)를 만들어.
  각 파일에서 정지 요청을 어떻게 보내는지(stop_mode 값, 동기/비동기 호출, 결과 처리)를 먼저 비교한 뒤, 차이를 그대로 지원하도록 설계해.
- 세 파일은 이 클래스를 import해서 쓰게 바꾸고, 정지 동작(언제, 어떤 모드로 멈추는지)은 그대로 유지해.

## [C] 종료 처리(홈 복귀 → 노드 정리 → shutdown) 통일
- 기준 패턴은 참고 경로 rokey/move.py와 같은 아래 구조야:
  ```python
  try:
      ...  # 작업
  except KeyboardInterrupt:
      node.get_logger().info("Program Stopped")
  except Exception as e:
      node.get_logger().info(f"Robot Error: {e}")   # 기존 파일이 error 레벨이면 error 유지
  finally:
      movej(homej, vel=VELOCITY, acc=ACC)
      node.destroy_node()
      rclpy.shutdown()
  ```
- try/except/finally 구조는 각 main()에 그대로 남겨. 무엇을 잡고 어떤 메시지를 남길지는 파일마다 다르기 때문이야.
- finally 안의 정리 순서만 utils/의 공통 함수로 뽑아(예: utils/shutdown.py의 `safe_shutdown(node, *cleanups)`).
  - cleanups는 파일마다 필요한 정리 동작을 순서대로 받는 콜백 목록이야 (예: 힘 제어 해제 → 홈 복귀).
  - 각 cleanup은 개별 try/except로 감싸서, 하나가 실패해도 다음 정리와 destroy_node/rclpy.shutdown은 반드시 실행되게 해.
  - 실패는 `node.get_logger().error(...)`로 남겨.
  - rclpy.ok()를 확인한 뒤 shutdown하는 기존 처리가 있으면 그 동작도 유지해.
- 파일마다 다른 추가 정리(wall_navigation의 힘 제어 해제, sun_gear의 그리퍼 처리 등)는 cleanup 콜백으로 넘겨서 순서와 동작을 똑같이 지켜.
- 홈 자세 값(homej)은 각 파일에 그대로 둬. posj(*args)와 posj([list]), int와 float 표기 차이도 바꾸지 마.

## [D] 이름·스타일 통일 (기준: /home/rokey/ws_cobot_pjt/ws_dsr/src/rokey/rokey/move.py, simple_input.py)
기준 파일의 규칙:
- 상수: 모듈 상단에 UPPER_SNAKE. 속도/가속도는 `VELOCITY, ACC = 30, 30`, 반복 횟수는 `CYCLE`, 디지털 출력은 `ON, OFF = 1, 0`
- 좌표 변수: 소문자 + 좌표 타입 접두어 (`homej`, `posj1`, `posx1`, `posx2` …)
- DSR import: main() 안에서 `DR_init.__dsr__node = node` 설정 뒤 `try: from DSR_ROBOT2 import (...)` / `from DR_common2 import posx, posj`,
  `except ImportError as e: node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")` 후 return
- 로그: `node.get_logger().info/error(...)`로 통일 (print, `log = node.get_logger().info` 별칭, 제각각인 self.log 래퍼 정리)
- 종료: [C]의 try/except/finally 패턴

적용 규칙:
- 용도가 여러 개인 속도/가속도는 `<용도>_VELOCITY`, `<용도>_ACC`로 통일해.
  예: MOVE_VEL → MOVE_VELOCITY, PROBE_VEL → PROBE_VELOCITY, SCAN_VEL → SCAN_VELOCITY, PULL_VEL → PULL_VELOCITY, JOINT_VEL → JOINT_VELOCITY.
  가속도(`*_ACC`)는 이미 기준과 같으니 그대로 둬.
- 이름만 바꿔. 값과 형태(스칼라 30, 리스트 [mm/s, deg/s])는 절대 바꾸지 마.
- 파일 안에 하드코딩된 acc=[50.0, 30.0] 같은 반복 리터럴은 같은 파일 안 상수로만 올려. 값은 그대로 둬.
- 바꾼 이름은 "기존 이름 → 새 이름" 대응표로 정리해.
- 사용자 출력이 목적인 print(예: wall_modeling의 결과 표, tape_locate의 저장 안내)는 노드가 없는 CLI 출력일 수 있어.
  그러니 바꾸기 전에 node가 있는 맥락인지 확인하고, 없으면 그대로 둬.

# 하지 말 것 (현재 상태 그대로 유지)
1. 로봇 기본 상수(ROBOT_ID='dsr01', ROBOT_MODEL='m0609')와 파일별 DR_init.__dsr__id/__dsr__model 설정 코드는 고정이야. 공통화하지 말고 각 파일에 그대로 둬.
2. 아래 이미 알려진 문제는 이번 작업에서 건드리지 마 (수정, 이동, 이름 변경 모두 금지):
   - utils/template.py의 `from rokey1.service_waiter import ServiceWaiter` import 경로
   - tape/, utils/에 __init__.py가 없는 것, setup.py의 entry_points 구성
   - `navigation/nhj_internal-navigation ` 폴더 이름(끝 공백, 하이픈)과 그 안의 파일
   - wall_navigation.py의 try/except 이중 import(`navigation.wall_modeling` / `wall_modeling`)
   → 리팩터링 결과가 이 문제들 때문에 import·빌드·실행이 안 되면, 직접 고치지 말고 "어떤 파일의 어떤 import가 어떤 이유로 막히는지" 보고만 해.
3. 로봇 이동 좌표, 속도/가속도 값, 이동 순서, 그리퍼 명령, 힘·안전 관련 로직은 절대 바꾸지 마.
4. 노드 이름, namespace, 토픽/서비스 이름과 타입, entry_point 실행 이름은 바꾸지 마.
5. 커밋하지 마. 변경 사항은 작업 트리에만 남겨둬.

# 지켜야 할 제약
- DSR_ROBOT2는 import하는 순간 DR_init 값을 읽어. 그래서 "DR_init 설정 → 노드 생성 → DSR_ROBOT2 import" 순서를 공통 모듈에서도 반드시 지켜.
  utils/ 공통 모듈은 모듈 최상단에서 DSR_ROBOT2를 import하면 안 돼. 필요한 함수나 클라이언트는 인자로 받아.
- 예외가 났을 때의 결과(어떤 정리를 어떤 순서로 하는지, 프로그램이 어떻게 끝나는지)는 기존과 같아야 해.
- PEP8, test/의 flake8/pep257 기준 통과.

# 작업 순서
1. 분석: 모든 .py를 읽고 [A]~[D] 각각에 해당하는 위치(파일:라인)와 바꿀 내용을 표로 정리해.
   [B]는 파일별 MoveStop 사용 방식 비교표를, [D]는 이름 대응표를 포함해.
   → 여기서 멈추고 나한테 계획을 보여준 뒤 승인받아.
2. 승인되면 [A] → [B] → [C] → [D] 순서로 적용해.
3. 검증
   - `colcon build --packages-select rokey1` 결과
   - 수정한 파일마다 `python3 -m py_compile <파일>` 결과
   - flake8 결과 (이번 수정으로 새로 생긴 경고만 따로 표시)
   - "하지 말 것 2"의 문제 때문에 막히는 부분이 있으면 그대로 보고
   - 실제 로봇이나 에뮬레이터 실행은 하지 말고, 사람이 테스트할 항목으로 남겨.

# 결과물 (이 순서로)
1. 분석 표 (리팩터링 전, [A]~[D])
2. 업데이트된 폴더 트리 (신설 파일 표시: utils/move_stop.py, utils/shutdown.py 등)
3. 수정·신설된 모든 파일의 전체 코드 (파일 경로와 import 포함)
4. 이름 변경 대응표 (기존 → 새 이름, 파일별)
5. 검증 결과 (실패하면 출력 그대로)
6. GitHub PR 설명글 (마크다운)
   - 변경 요약: 신설 공통 모듈과 수정 파일 목록
   - 공통 모듈 사용법: ServiceWaiter, MoveStop 클래스, safe_shutdown 예시 코드
   - 폴더별(rokey1/navigation/tape) 담당자가 실제 로봇이나 에뮬레이터에서 확인할 테스트 체크리스트
     (서비스 대기 후 정상 시작, Ctrl+C 시 홈 복귀, 예외 시 힘 제어 해제 순서, MoveStop 정지 동작 등)
   - 이번 PR에서 일부러 손대지 않은 알려진 문제 목록 ("하지 말 것 2")
   - 동작이 바뀔 수 있는 위험 지점과 그 근거
