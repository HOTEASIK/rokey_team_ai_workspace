# 호떡 누르미 RViz2 / M0609 모델

## 확정 치수
- 누름판 반경 45 mm (지름 90 mm)
- 총 높이 95 mm
- 손잡이 최상단 반경 13 mm (지름 26 mm)
- STL과 URDF 모두 **미터(m)** 단위, 접촉 바닥 중심이 원점 (0,0,0), 손잡이는 +Z 방향

## 사진 기반 추정 치수 (실측 필요)
- 누름판 두께 약 1.6 mm
- 손잡이 중간 반경 약 10~11 mm
- 파지 후보 위치 Z=65 mm
- 질량/관성 임시값, 실제 제품 측정 전 물리 제어에 사용하지 마세요.

## 설치 및 실행 (Ubuntu 24.04 / ROS2 Jazzy)
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cp -r /home/rlagpdbs/다운로드/hotteok_press_m0609_rviz/hotteok_press_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select hotteok_press_description --symlink-install
source install/setup.bash
ros2 launch hotteok_press_description view.launch.py
```
RViz2 Fixed Frame: `hotteok_press_link`.

## M0609 결합 주의
이 패키지는 **누르미 단독 RViz 모델**입니다. 실제 M0609/2지 그리퍼 URDF는 포함되지 않았습니다.
로봇 URDF의 실제 파지 프레임과 `handle_grasp_frame`을 고정 조인트로 연결해야 합니다.
그리퍼의 실제 링크 이름, 손잡이 파지 높이, 그리퍼 축 정렬은 현장 측정이 필요합니다.
실제 로봇 동작, 접촉력, 파지력은 이 파일로 검증할 수 없습니다.
