# sun_gear 트러블슈팅 기록

- 대상: `src/rokey1/rokey1/sun_gear.py` (`ros2 run rokey1 sun_gear`)
- 원본: DART 에서 만든 `m0609_pick_3.drl` 을 ROS 2 (DSR_ROBOT2) 로 옮긴 코드
- 환경: Doosan M0609 + RG2 그리퍼, 실제 로봇 (`mode:=real host:=192.168.1.100`)
- 날짜: 2026-11-12

---

## 요약

| # | 증상 | 원인 | 해결 |
|---|------|------|------|
| 1 | `set_tool failed: -1` | 컨트롤러에 없는 Tool 이름(`Tool Weight_1`) 사용 | `TOOL_NAME = 'Tool Weight'` |
| 2 | `Pick force check failed (10~15N)` | 원본 DART 의 10~15N 범위를 그대로 가져옴. -50N 으로 누르면 실측 약 45.6N | 범위를 35~60N 으로 변경 |
| 3 | 내려가면서 집지 못함 | 원본 DRL 의 '그리퍼 열기 → 닫기' 시점을 잘못 옮김 | 힘 제어 중(내려가는 동안)에 열기 → 닫기 |
| 4 | 4번 작업에서 `Pick force check failed (35~60N)` | 4번 집는 힘을 -70N 으로 바꿨는데 범위는 그대로 둠 | 범위를 목표 힘에 비례하게 계산 (`PICK_FORCE_RATIO`) |

---

## 1. `set_tool failed: -1`

### 증상
```
[ERROR] [dsr01.m0609_pick_3]: Program aborted: set_tool failed: -1
  File ".../sun_gear.py", line 327, in main
    command('set_tool', robot.set_tool, TOOL_NAME)
RuntimeError: set_tool failed: -1
```

### 원인
- `DSR_ROBOT2.set_tool()` 은 컨트롤러가 `success=False` 를 돌려주면 `-1` 을 반환한다.
- 컨트롤러에 설정된 Tool / TCP 를 조회해 보니:
  ```bash
  ros2 service call /dsr01/dsr_controller2/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool
  # → info='Tool Weight', success=True
  ros2 service call /dsr01/dsr_controller2/tcp/get_current_tcp dsr_msgs2/srv/GetCurrentTcp
  # → info='GripperDA_v1', success=True
  ```
- 코드에서는 `'Tool Weight_1'` 을 쓰고 있었는데, 이 이름은 이 티칭 펜던트에 등록되어 있지 않았다. TCP 이름은 맞았다.

### 해결
```python
TOOL_NAME = 'Tool Weight'      # 기존: 'Tool Weight_1'
TCP_NAME = 'GripperDA_v1'
```

### 참고
- `move.py`, `grip_test.py`, `simple_input.py` 에도 `"Tool Weight_1"` 이 남아 있다. 이 파일들은 반환값을 검사하지 않아서 에러 없이 **조용히 실패**하고 있었을 가능성이 크다.
- 프로그램이 `set_robot_mode MANUAL` 다음에 멈추면 로봇이 MANUAL 모드로 남는다. 다시 실행하면 AUTONOMOUS 로 바뀐다.

---

## 2. 집기 힘 검사 실패 (10~15N)

### 증상
```
[INFO]  Tool force @ -50.0N stage: [-4.10, 1.57, -45.64, 0.11, 0.88, -0.20]
[ERROR] Program aborted: Pick force check failed (10~15N): -1; stopping cycle
```

### 원인
- 힘이 변한 게 아니다. -50N 으로 누르면 실측 Fz 가 **약 -45.6N** 으로, 정상적으로 눌린 값이다.
- 검사 범위 `PICK_FORCE_RANGE` 가 원본 DART 의 **10~15N** 그대로라서, -50N 으로 누를 때는 절대 통과할 수 없었다.
- `check_force_condition` 은 방향(부호)과 상관없이 **힘의 크기**로 비교한다. 그래서 -45.6N 이 35~60N 범위에서는 통과한다.
- Tool 이름을 바꾼 것(무게 보정 차이)은 보통 몇 N 수준이라 원인이 아니다.

### 해결
```python
PICK_FORCE_RANGE = (35, 60)    # 이후 4번에서 PICK_FORCE_RATIO 로 다시 바뀜
```

### 참고
- 실측 Fx 가 약 -4.1N 이다. 옆 방향으로 힘이 조금 걸리고 있으니, 집는 위치가 살짝 어긋났는지 확인해 볼 것.
- 이 워크스페이스는 symlink-install 이 아니다. `src` 를 고칠 때마다 다시 빌드해야 반영된다.
  ```bash
  colcon build --packages-select rokey1
  ```

---

## 3. 내려가면서 집지 못함

### 증상
- 부품 위로 내려가는 동안 그리퍼가 계속 닫혀 있어서 부품을 감싸지 못했다.
- 임시로 `apply_force()` 다음에 `gripper_open()` / `gripper_close()` 를 넣어 봤지만, 이미 힘 제어가 끝나서 로봇이 멈춘 뒤에 동작해서 원본과 달랐다.

### 원인: 원본 DRL 의 실제 동작
```python
set_desired_force([0,0,-50,0,0,0], [0,0,1,0,0,0], ...)   # 내려가기 시작
if check_force_condition(axis=DR_AXIS_Z, min=10, max=15, ref=DR_TOOL):
    pass
else:
    oo()    # 그리퍼 열기 (1초)
    cc()    # 그리퍼 닫기 (1초)
wait(2.00)
release_force(); release_compliance_ctrl()
```
- 힘을 건 **바로 직후** 검사하기 때문에 아직 부품에 닿지 않아 힘은 0 에 가깝다. 그래서 검사는 **항상 실패**하고 else 로 빠진다.
- 결과적으로 원본은 "힘 검사"가 아니라 **내려가는 동안 그리퍼를 벌렸다가 닫아서 집는** 동작을 하고 있었다.
- ROS 2 로 옮길 때 이 검사를 '2초 유지한 뒤'로 옮기면서 열기/닫기 동작이 빠졌다.

### 해결
`apply_force()` 에 `grip_while_pressing` 옵션을 추가해, 힘 제어를 건 직후 열기 → 닫기를 실행하도록 했다.
```python
def apply_force(target_n, hold_sec, check=False, grip_while_pressing=False):
    ...
    robot.set_desired_force([0, 0, target_n, 0, 0, 0], [0, 0, 1, 0, 0, 0], ...)
    force_active = True
    if grip_while_pressing:
        gripper_open()     # 1초
        gripper_close()    # 1초
    robot.wait(hold_sec)   # 2초
    # (check=True 이면) 힘 검사
```
```python
# pick_place_joint (1~3번), pick_place4 (4번) 의 집기 단계
apply_force(-50.0, 2.0, check=True, grip_while_pressing=True)
```
- 순서와 시간(열기 1초, 닫기 1초, 유지 2초)이 원본 DRL 과 같아졌다.
- 다시 실행한 결과 1~3번 작업은 정상적으로 통과했다.

---

## 4. 4번 작업 힘 검사 실패 (35~60N)

### 증상
```
  File ".../sun_gear.py", line 302, in pick_place4
    apply_force(-70.0, 2.0, check=True, grip_while_pressing=True)
RuntimeError: Pick force check failed (35~60N): -1; stopping cycle
```

### 원인
- 4번 작업의 집는 힘을 -70N 으로 올렸는데, 검사 범위는 -50N 기준인 35~60N 고정값 그대로였다.

### 해결
범위를 목표 힘의 크기에 비례해서 계산하도록 바꿨다.
```python
PICK_FORCE_RATIO = (0.7, 1.2)  # 범위 = |목표 힘| x 비율

fmin = round(abs(target_n) * PICK_FORCE_RATIO[0], 1)
fmax = round(abs(target_n) * PICK_FORCE_RATIO[1], 1)
```
| 목표 힘 | 검사 범위 |
|--------|-----------|
| -50N (1~3번) | 35 ~ 60N |
| -70N (4번) | 49 ~ 84N |

### 참고
- 로봇에서 아직 다시 확인하지 않았다. 다음에 돌릴 때 `Tool force @ -70.0N stage` 로그의 Z 값(세 번째 값)을 확인할 것.
- 원본 DRL 처럼 힘 검사로 멈추지 않게 하려면 `CHECK_PICK_FORCE = False`.

---

## 남은 확인 사항

- [ ] 4번 작업 (-70N) 실측 힘을 보고 `PICK_FORCE_RATIO` 조정
- [ ] 4번 작업: 그리퍼를 연 **뒤에** Z 150 → 98.57mm 로 **내려가는** 동작이 의도한 것인지 (원본 DRL 그대로임)
- [ ] 1번 기어 집기 위치의 옆 방향 힘(Fx 약 -4N) 확인
- [ ] 다른 스크립트(`move.py`, `grip_test.py`, `simple_input.py`)의 `Tool Weight_1` 정리
- [ ] -80N 으로 5초 끼우는 단계는 힘 검사가 없으니, 처음 실행할 때는 비상정지 버튼을 가까이 둘 것

## 디버깅에 쓴 명령어

```bash
# 현재 Tool / TCP / 로봇 모드 조회
ros2 service call /dsr01/dsr_controller2/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool
ros2 service call /dsr01/dsr_controller2/tcp/get_current_tcp dsr_msgs2/srv/GetCurrentTcp
ros2 service call /dsr01/dsr_controller2/system/get_robot_mode dsr_msgs2/srv/GetRobotMode   # 0=MANUAL, 1=AUTONOMOUS

# 수정 후 다시 빌드 (symlink-install 아님)
colcon build --packages-select rokey1
ros2 run rokey1 sun_gear
```
