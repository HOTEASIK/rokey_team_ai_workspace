# M0609 ROS2 Command Center

## 01. 로봇 상태 모니터링과 작업 진행 표시
- [x] 작업 진행정보, linear/angular 속도, IDLE/AUTO/MANUAL/ERROR 모드, Conveyer/Motor/PLC 연결 상태, Status 라벨, 진행률 및 연결 LED를 제공한다.

## 02. 로봇 제어와 안전 우선 조작
- [x] START/STOP, 화면에 항상 노출되는 EMERGENCY STOP, WASD 및 방향키 조작, 속도 슬라이더, AUTO/MANUAL 전환을 제공한다.
- [x] E-STOP 작동 시 모션을 정지하고 안전 경고를 최우선으로 표시하며 RESET E-STOP 흐름을 제공한다.

## 03. 통신·센서·안전 모니터링
- [x] ROS2 topic, TCP, Serial 상태와 packet latency, message loss rate 및 latency graph를 표시한다.
- [x] Force Monitoring, 충돌 감지 상태, 거리 센서 값과 실시간 그래프·숫자 패널을 표시한다.
- [x] Emergency stop, 충돌 감지, 속도 제한 상태와 시스템 fault 로그를 붉은 경고 우선순위로 표시한다.

## 04. 실시간 작동 시뮬레이션과 디버그 이력
- [x] 오른쪽 상단에 M0609 로봇 명령 실행과 움직임을 보여주는 시뮬레이션을 제공한다.
- [x] 오른쪽 하단에 최신 ROS2·제어·센서·안전 이벤트를 연속 표시하는 실시간 로그 뷰를 제공한다.
- [x] INFO/WARN/ERROR 필터와 time-stamped 이벤트, error log, command history, sensor log를 제공한다.
- [x] 왼쪽 기능 영역과 오른쪽 시뮬레이션·로그 영역이 노트북 화면에 맞게 반응하도록 구성한다.
