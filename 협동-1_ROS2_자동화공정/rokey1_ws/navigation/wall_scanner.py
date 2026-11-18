#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import time

import rclpy
import DR_init

from datetime import datetime


# =====================================================
# 1. 로봇 설정
# =====================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

READY_J = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# =====================================================
# 2. 벽의 네 모서리 좌표 (Base 좌표계, mm)
# =====================================================

BOTTOM_LEFT = [787.14, 225.47, 32.93]
TOP_LEFT = [793.27, 221.34, 204.32]

TOP_RIGHT = [796.20, -254.90, 210.33]
BOTTOM_RIGHT = [786.95, -253.26, 34.65]


# =====================================================
# 3. 탐색 설정
# =====================================================

# 수직 방향 탐색 간격
VERTICAL_STEP = 35.0

# 수평 방향 탐색 간격
HORIZONTAL_STEP = 5.0

# 벽 앞에서 처음 접근할 때 띄울 거리
APPROACH_DISTANCE = 350.0

# 접촉 상태에서 벽으로부터 유지할 위치 오프셋
# 실제 접촉은 힘 제어가 담당
CONTACT_OFFSET = 1.0

# TCP 이동 속도 (mm/s)
SCAN_VEL = 10.0
SCAN_ACC = 20.0

# 목표 접촉력 (N)
TARGET_FORCE = 5.0

# 이 힘보다 작으면 접촉이 끊어진 것으로 판단
GAP_FORCE_THRESHOLD = 2.0

# 힘 제어 강성
# X 방향은 상대적으로 부드럽게 설정
STIFFNESS = [1000.0, 3000.0, 3000.0,
             200.0, 200.0, 200.0]

# 힘 제어 방향: Base X축
FORCE_DIRECTION = [1, 0, 0, 0, 0, 0]

# 탐색 결과 저장 위치
RESULT_FILE = os.path.expanduser(
    "~/ws_cobot_pjt/ws_dsr/gap_results.json"
)


# =====================================================
# 4. 좌표 계산 함수
# =====================================================

def interpolate(p1, p2, ratio):
    return [
        p1[i] + (p2[i] - p1[i]) * ratio
        for i in range(3)
    ]


def distance(p1, p2):
    return math.sqrt(
        sum((p1[i] - p2[i]) ** 2 for i in range(3))
    )


def wall_point(u, v):
    """
    벽의 네 모서리를 이용한 좌표 보간.

    u : 좌우 위치 (0=왼쪽, 1=오른쪽)
    v : 상하 위치 (0=위쪽, 1=아래쪽)

    벽이 완벽한 평면이 아니어도
    네 모서리를 기준으로 X,Y,Z를 계산한다.
    """

    left = interpolate(TOP_LEFT, BOTTOM_LEFT, v)
    right = interpolate(TOP_RIGHT, BOTTOM_RIGHT, v)

    return interpolate(left, right, u)


# =====================================================
# 5. 탐색 경로 생성
# =====================================================

def make_scan_path():

    left_height = distance(TOP_LEFT, BOTTOM_LEFT)
    right_height = distance(TOP_RIGHT, BOTTOM_RIGHT)

    wall_height = max(left_height, right_height)

    row_count = math.ceil(
        wall_height / VERTICAL_STEP
    ) + 1

    path = []

    for row in range(row_count):

        v = row / (row_count - 1)

        # 현재 높이에서 벽의 좌우 폭 계산
        left_point = wall_point(0.0, v)
        right_point = wall_point(1.0, v)

        width = distance(left_point, right_point)

        column_count = math.ceil(
            width / HORIZONTAL_STEP
        ) + 1

        # 지그재그 방향
        if row % 2 == 0:
            u_values = [
                i / (column_count - 1)
                for i in range(column_count)
            ]
        else:
            u_values = [
                1.0 - i / (column_count - 1)
                for i in range(column_count)
            ]

        for u in u_values:

            point = wall_point(u, v)

            path.append({
                "row": row,
                "u": u,
                "v": v,
                "point": point
            })

    return path, row_count


# =====================================================
# 6. API 반환값 확인
# =====================================================

def check_api_result(result, function_name):
    """
    DSR_ROBOT2 API는 성공 시 0,
    실패 시 -1을 반환하는 함수가 있다.
    bool을 반환하는 경우도 고려한다.
    """

    if isinstance(result, bool):
        success = result
    else:
        success = (result == 0)

    if not success:
        raise RuntimeError(
            f"{function_name} 실행 실패: {result}"
        )


def check_contact(check_force_condition, DR_AXIS_X, DR_BASE):
    """
    check_force_condition:
    조건 만족 시 0, 불만족 시 -1 반환을 사용한다.
    """

    result = check_force_condition(
        DR_AXIS_X,
        min=GAP_FORCE_THRESHOLD,
        ref=DR_BASE
    )

    if isinstance(result, bool):
        return result

    return result == 0


# =====================================================
# 7. 결과 JSON 저장
# =====================================================

def save_results(gap_points, total_points):

    result = {
        "robot": ROBOT_MODEL,
        "timestamp": datetime.now().isoformat(),

        "wall_corners": {
            "top_left": TOP_LEFT,
            "top_right": TOP_RIGHT,
            "bottom_left": BOTTOM_LEFT,
            "bottom_right": BOTTOM_RIGHT
        },

        "scan_settings": {
            "horizontal_step_mm": HORIZONTAL_STEP,
            "vertical_step_mm": VERTICAL_STEP,
            "target_force_N": TARGET_FORCE,
            "gap_force_threshold_N": GAP_FORCE_THRESHOLD
        },

        "total_scan_points": total_points,
        "gap_count": len(gap_points),

        "gaps": gap_points
    }

    with open(RESULT_FILE, "w", encoding="utf-8") as file:
        json.dump(
            result,
            file,
            ensure_ascii=False,
            indent=4
        )

    print(f"\n탐색 결과 저장 완료: {RESULT_FILE}")


# =====================================================
# 8. 벽면 탐색
# =====================================================

def scan_wall(
    path,
    movel,
    posx,
    check_force_condition,
    DR_AXIS_X,
    DR_BASE
):

    gap_points = []

    print("\n===== 벽면 탐색 시작 =====")

    total = len(path)

    for index, item in enumerate(path):

        point = item["point"]

        # 실제 벽 좌표보다 1mm 앞을 목표로 설정
        target = posx(
            point[0] - CONTACT_OFFSET,
            point[1],
            point[2],
            *ORIENTATION
        )

        print(
            f"[{index + 1}/{total}] "
            f"행={item['row']} "
            f"좌표={point}",
            flush=True
        )

        # TCP 이동
        result = movel(
            target,
            vel=SCAN_VEL,
            acc=SCAN_ACC,
            ref=DR_BASE
        )

        check_api_result(result, "movel")

        # 이동 후 힘 측정 서비스 응답 대기
        time.sleep(0.05)

        contact = check_contact(
            check_force_condition,
            DR_AXIS_X,
            DR_BASE
        )

        if not contact:

            gap = {
                "row": item["row"],
                "position": {
                    "x": round(point[0], 3),
                    "y": round(point[1], 3),
                    "z": round(point[2], 3)
                },
                "force_status": "CONTACT_LOST"
            }

            gap_points.append(gap)

            print(
                f"  [GAP 후보] "
                f"({point[0]:.2f}, "
                f"{point[1]:.2f}, "
                f"{point[2]:.2f})",
                flush=True
            )

    print("\n===== 벽면 탐색 완료 =====")

    return gap_points


# =====================================================
# 9. 메인 함수
# =====================================================

def main(args=None):

    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = rclpy.create_node(
        "wall_scanner",
        namespace=ROBOT_ID
    )

    DR_init.__dsr__node = node

    compliance_enabled = False
    force_enabled = False

    try:

        from DSR_ROBOT2 import (
            movej,
            movel,
            get_current_posx,
            set_robot_mode,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            release_force,
            release_compliance_ctrl,
            set_ref_coord,

            ROBOT_MODE_AUTONOMOUS,
            DR_AXIS_X,
            DR_BASE,
            DR_FC_MOD_ABS
        )

        from DR_common2 import posx

        global ORIENTATION

        print("\n===================================")
        print(" DOOSAN M0609 WALL SCANNER")
        print(" REAL ROBOT MODE")
        print("===================================")

        # -------------------------------------------------
        # 1. 탐색 경로 계산
        # -------------------------------------------------

        path, row_count = make_scan_path()

        print(f"\n탐색 줄 수: {row_count}")
        print(f"총 탐색 지점 수: {len(path)}")

        print(f"수평 간격: {HORIZONTAL_STEP} mm")
        print(f"수직 간격: {VERTICAL_STEP} mm")

        # -------------------------------------------------
        # 2. 사용자 실행 확인
        # -------------------------------------------------

        print("\n실제 로봇을 움직입니다.")
        print("로봇 주변에 사람이나 장애물이 없는지 확인하세요.")

        answer = input(
            "\n실행하려면 MOVE를 입력하세요: "
        )

        if answer.strip() != "MOVE":
            print("탐색을 취소했습니다.")
            return

        # -------------------------------------------------
        # 3. 로봇 자동 모드
        # -------------------------------------------------

        mode_result = set_robot_mode(
            ROBOT_MODE_AUTONOMOUS
        )

        check_api_result(
            mode_result,
            "set_robot_mode"
        )

        print("[INFO] 자동 모드 설정 완료")

        # -------------------------------------------------
        # 4. READY_J 이동
        # -------------------------------------------------

        print("\n[1] READY_J 이동")

        result = movej(
            READY_J,
            vel=20,
            acc=20
        )

        check_api_result(result, "movej READY_J")

        # -------------------------------------------------
        # 5. 현재 TCP 자세 읽기
        # -------------------------------------------------

        current_pose = get_current_posx()

        # 일반적인 반환 형식:
        # ([x, y, z, r, p, yaw], solution_space)
        if (
            isinstance(current_pose, (list, tuple))
            and len(current_pose) == 2
            and isinstance(current_pose[0], (list, tuple))
        ):
            current_pose = current_pose[0]

        if current_pose is None or len(current_pose) < 6:
            raise RuntimeError(
                "현재 TCP 자세를 읽지 못했습니다."
            )

        ORIENTATION = list(current_pose[3:6])

        print(
            f"[INFO] 현재 TCP 자세: {ORIENTATION}"
        )

        # -------------------------------------------------
        # 6. 벽의 왼쪽 위 접근 위치로 이동
        # -------------------------------------------------

        print("\n[2] 벽 왼쪽 위 접근")

        approach_point = wall_point(0.0, 0.0)

        approach_target = posx(
            approach_point[0] - APPROACH_DISTANCE,
            approach_point[1],
            approach_point[2],
            *ORIENTATION
        )

        result = movel(
            approach_target,
            vel=SCAN_VEL,
            acc=SCAN_ACC,
            ref=DR_BASE
        )

        check_api_result(
            result,
            "벽 접근 이동"
        )

        # -------------------------------------------------
        # 7. Base 좌표계 설정
        # -------------------------------------------------

        set_ref_coord(DR_BASE)

        print("\n[3] X축 힘 제어 활성화")

        # -------------------------------------------------
        # 8. 컴플라이언스 제어
        # -------------------------------------------------

        result = task_compliance_ctrl(
            stx=STIFFNESS
        )

        check_api_result(
            result,
            "task_compliance_ctrl"
        )

        compliance_enabled = True

        # ROS2 서비스 응답과 실제 모드 전환 사이의
        # 짧은 지연을 고려
        time.sleep(0.5)

        # -------------------------------------------------
        # 9. 벽 방향으로 접촉력 설정
        # -------------------------------------------------

        result = set_desired_force(
            fd=[
                TARGET_FORCE,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ],
            dir=FORCE_DIRECTION,
            time=0.3,
            mod=DR_FC_MOD_ABS
        )

        check_api_result(
            result,
            "set_desired_force"
        )

        force_enabled = True

        time.sleep(0.5)

        # -------------------------------------------------
        # 10. 벽면 지그재그 탐색
        # -------------------------------------------------

        gap_points = scan_wall(
            path,
            movel,
            posx,
            check_force_condition,
            DR_AXIS_X,
            DR_BASE
        )

        # -------------------------------------------------
        # 11. 힘 제어 해제
        # -------------------------------------------------

        print("\n[4] 힘 제어 해제")

        release_force()
        force_enabled = False

        time.sleep(0.3)

        release_compliance_ctrl()
        compliance_enabled = False

        # -------------------------------------------------
        # 12. 결과 저장
        # -------------------------------------------------

        save_results(
            gap_points,
            len(path)
        )

        # -------------------------------------------------
        # 13. HOME_J 복귀
        # -------------------------------------------------

        print("\n[5] HOME_J 복귀")

        result = movej(
            HOME_J,
            vel=20,
            acc=20
        )

        check_api_result(
            result,
            "movej HOME_J"
        )

        print("\n===== 모든 작업 완료 =====")

        print(f"발견한 틈새 후보: {len(gap_points)}개")

    except KeyboardInterrupt:

        print("\n사용자가 탐색을 중단했습니다.")

    except Exception as e:

        print(f"\n[ERROR] {e}")

    finally:

        # 오류가 발생해도 힘 제어는 해제
        if force_enabled:
            try:
                release_force()
                print("[INFO] 힘 제어 해제")
            except Exception as e:
                print(f"[WARNING] 힘 제어 해제 실패: {e}")

        if compliance_enabled:
            try:
                release_compliance_ctrl()
                print("[INFO] 컴플라이언스 해제")
            except Exception as e:
                print(f"[WARNING] 컴플라이언스 해제 실패: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
