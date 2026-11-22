## rokey1 리팩터링: 서비스 대기 / MoveStop / 종료 처리 공통화 + 이름·로그 스타일 통일

### 변경 요약
**신설 공통 모듈 (`utils/`)**
- `utils/__init__.py` — 빈 파일. `find_packages`가 utils를 설치하도록 추가 (없으면 설치 후 `from utils...` ImportError)
- `utils/move_stop.py` — `MoveStopClient`
- `utils/shutdown.py` — `run_cleanups`, `safe_shutdown`

**삭제**
- `putty_demo/service_waiter.py` (utils/service_waiter.py 와 동일한 복사본 → utils 것을 사용)

**수정 파일**
- rokey1/: `move.py`, `grip_test.py`, `sun_gear.py`, `tape_peel.py`, `tape_locate.py`
- tape/: `tape_peel.py`, `tape_locate.py`
- navigation/: `wall_navigation.py`, `wall_size.py`, `table_navigation.py`, `force_guard.py`
- putty_demo/: `initialize_device.py`, `pickup_spatula.py`

로봇 좌표, 속도/가속도 값, 이동 순서, 그리퍼 명령, 힘·안전 로직, 노드/토픽/서비스 이름은 바꾸지 않았습니다.

### 공통 모듈 사용법
```python
from utils.service_waiter import ServiceWaiter
from utils.move_stop import MoveStopClient
from utils.shutdown import safe_shutdown

import DSR_ROBOT2                                   # DR_init 설정 → 노드 생성 → import 순서 유지
stopper = MoveStopClient(node)
ServiceWaiter(node).wait(DSR_ROBOT2._ros2_movej, DSR_ROBOT2._ros2_movel, stopper.cli)

stopper.stop(DSR_ROBOT2.DR_QSTOP)                   # 응답까지 대기 (tape_*)
stopper.stop(DSR_ROBOT2.DR_QSTOP, timeout_sec=2.0)  # 최대 2초 (wall/table_navigation)

try:
    ...
except KeyboardInterrupt:
    node.get_logger().info("Program Stopped")
finally:
    # cleanup 은 콜백 또는 (실패 메시지, 콜백). 각각 따로 try/except → 실패해도 다음 정리와
    # destroy_node / rclpy.shutdown(rclpy.ok() 확인 후) 은 반드시 실행
    safe_shutdown(node, ('힘 제어 해제 실패', io.release_press),
                  ('Failed to move home', lambda: movej(homej, vel=VELOCITY, acc=ACC)))
```
`run_cleanups(node, *cleanups)` 은 shutdown 없이 정리만 한다 (wall/table_navigation 처럼 정리 후 모델링을 더 하는 경우).

### 이름 변경 (값·형태 그대로)
| 파일 | 기존 → 새 이름 |
|---|---|
| tape/·rokey1/ tape_peel | `SCAN_VEL→SCAN_VELOCITY`, `PULL_VEL→PULL_VELOCITY`, 신규 상수 `APPROACH_VELOCITY=[50.0, 30.0]`, `APPROACH_ACC=[100.0, 60.0]`, `SCAN_ACC=[50.0, 30.0]`, `PULL_ACC=[50.0, 30.0]` |
| tape/·rokey1/ tape_locate | `PROBE_VEL→PROBE_VELOCITY`, `MOVE_VEL→MOVE_VELOCITY` |
| wall_navigation | `MOVE_VEL`, `PROBE_VEL`, `SCAN_VEL`, `JOINT_VEL` → `*_VELOCITY` |
| wall_size | `MOVE_VEL`, `PROBE_VEL`, `FOLLOW_VEL` → `*_VELOCITY` |
| table_navigation | `MOVE_VEL`, `DESCEND_VEL`, `PROBE_VEL`, `DEEP_PROBE_VEL`, `SCAN_VEL`, `ROT_VEL`, `JOINT_VEL` → `*_VELOCITY` |
| force_guard | `RELIEVE_VEL → RELIEVE_VELOCITY` |
| 로그 | `log = node.get_logger().info` 별칭, `log()` 헬퍼(sun_gear), `RobotIO.log/warn`, `HybridWallExplorer.log`, `WallSizeMeasurer.log`, `TableExplorer.log` 삭제 → `node.get_logger().info/warn(...)` 직접 호출 |
| MoveStop | `self.stop_cli`/`self._MoveStop` → `self.stopper` (`MoveStopClient`) |

### 테스트 체크리스트 (실제 로봇 / 에뮬레이터)
**rokey1**
- [ ] `ros2 run rokey1 move` / `grip_test`: 서비스 대기 로그 후 정상 시작, 사이클 완료 후 홈 복귀
- [ ] move / grip_test: 동작 중 Ctrl+C → "Program Stopped" → 홈 복귀 → 정상 종료
- [ ] sun_gear: 힘 제어 단계에서 Ctrl+C/에러 → release_force → release_compliance 순서로 해제, 에러는 로그 후 비정상 종료(re-raise 유지)
- [ ] sun_gear `simulation:=true`: ENABLE_FORCE/PERIODIC 조건에 맞는 서비스만 대기

**tape**
- [ ] tape_peel: 반력 감지 시 Quick stop 후 그리퍼 닫기·당기기 / 미감지 시 후퇴
- [ ] tape_locate probe: 각 각도에서 접촉 시 Quick stop, teach 모드 저장 안내(print) 그대로 출력

**navigation**
- [ ] wall_navigation / wall_size / table_navigation: 시작 시 move_stop 포함 서비스 대기 후 시작
- [ ] 접촉·과부하 시 Quick stop (2초 타임아웃) 정상 정지
- [ ] 예외/Ctrl+C 시: 힘 제어 해제 → 데이터 저장 → (상승) → 홈 복귀 순서, 이후 모델링 실행
- [ ] wall_size: 사고 주입(`inject:=tool_side` 등) 후 홈 복귀 정책(auto/confirm)·특이점 복구·RViz 정리
- [ ] real 모드 시작 확인에서 `n` 입력 → 아무 동작 없이 종료

**putty_demo**
- [ ] initialize_device / pickup_spatula: 실행 후 정상 종료 (utils.service_waiter import 경로 변경)

### 일부러 손대지 않은 알려진 문제
- `utils/template.py`의 `from rokey1.service_waiter import ServiceWaiter` (존재하지 않는 경로 → template 은 그대로는 import 불가)
- `tape/`에 `__init__.py` 없음 → tape/ 는 설치되지 않음 (실제 설치되는 건 동일 내용의 `rokey1/tape_*.py`)
- `setup.py` entry_points (move, grip_test 만 등록)
- wall_navigation / wall_size / table_navigation 의 try/except 이중 import
- `rokey1/tape_*.py` ↔ `tape/tape_*.py` 중복 (둘 다 같은 내용으로 수정)
- 문서에 있던 `navigation/nhj_internal-navigation ` 폴더는 현재 브랜치에 없음
- 대기 방식이 다른 곳은 ServiceWaiter 로 바꾸지 않음: `force_guard` (3초 1회 확인 후 기능 끔), `pickup_spatula` (rclpy.ok() 아니면 중단), `sanding` (5초 후 종료)
- 예외: "하지 말 것 2"였던 `utils/__init__.py` 없음 문제는 공통 모듈이 동작하려면 필수라 **빈 파일 추가만** 했음 (팀 합의)

### 동작이 바뀔 수 있는 지점
1. **move.py**: 기존엔 finally 의 홈 복귀가 실패하면 예외가 전파돼 destroy_node/shutdown 이 생략됨 → 이제 에러 로그 후 정리는 계속.
2. **wall_navigation / wall_size / table_navigation**: finally 안의 `save()`·`go_home_after_stop`·`is_safe_stopped()` 가 이제 각각 보호됨 → 저장 실패해도 홈 복귀는 진행 (기존엔 저장 실패 시 홈 복귀 생략).
3. **sun_gear**: 종료 시 힘/컴플라이언스 해제 실패를 `pass` 로 무시하던 것을 error 로그로 남김.
4. **move / grip_test / tape_peel**: `rclpy.shutdown()` 전에 `rclpy.ok()` 확인 추가 (이중 shutdown 예외 방지).
5. 서비스 대기 로그 문구가 ServiceWaiter 의 `Waiting for service ... ` 로 통일 (기존 '서비스 대기 중: ...').
6. tape / navigation 의 `import DSR_ROBOT2` 를 `try/except ImportError` 로 감쌈 → 실패 시 트레이스백 대신 로그 후 return (기준 파일 패턴).
7. navigation 파일을 `python3 navigation/xxx.py` 로 **직접 실행**하면 `utils` 를 찾지 못할 수 있음 (패키지 루트가 sys.path 에 없을 때). `ros2 run` 은 문제 없음.

### 검증
- `python3 -m py_compile`: 수정/신설 16개 파일 모두 통과
- `ament_flake8`: 330 → 317건, **이번 수정으로 새로 생긴 경고 0건**
- `ament_pep257`: 신규 경고 0건 (기존 149건 그대로)
- 모듈 import 스모크 테스트 14개 OK, `safe_shutdown` 단위 확인 (cleanup 실패 후에도 다음 cleanup·shutdown 실행, 이중 호출 안전)
- `colcon build`: 생략 (요청에 따라)

🤖 Generated with [Claude Code](https://claude.com/claude-code)
