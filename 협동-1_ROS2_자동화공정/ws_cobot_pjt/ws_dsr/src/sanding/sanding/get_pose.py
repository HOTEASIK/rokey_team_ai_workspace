"""두산 M0609 현재 위치 토픽 퍼블리셔 (베이스 좌표계 기준)

로봇을 움직이지 않고 현재 위치를 읽어서 주기적으로 토픽으로 퍼블리시합니다.
  - /dsr01/current_posx (std_msgs/Float64MultiArray)
      [x, y, z (mm), rx, ry, rz (deg, ZYZ 오일러)]  -- DR_BASE 기준
  - /dsr01/current_posj (std_msgs/Float64MultiArray)
      [j1 ~ j6 (deg)]

사용법:
  ros2 run sanding get_pose
  ros2 topic echo /dsr01/current_posx
  ros2 topic echo /dsr01/current_posj
"""
import time

import rclpy
import DR_init
from std_msgs.msg import Float64MultiArray

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
PERIOD = 0.5  # 퍼블리시 주기 (초)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sanding_get_pose", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    try:
        from DSR_ROBOT2 import get_current_posx, get_current_posj, DR_BASE
    except ImportError as e:
        logger.error(f"Error importing DSR_ROBOT2 : {e}")
        return

    # namespace가 dsr01이므로 /dsr01/current_posx, /dsr01/current_posj 로 퍼블리시됨
    posx_pub = node.create_publisher(Float64MultiArray, "current_posx", 10)
    posj_pub = node.create_publisher(Float64MultiArray, "current_posj", 10)

    logger.info(
        f"Publishing /{ROBOT_ID}/current_posx, /{ROBOT_ID}/current_posj every {PERIOD}s"
    )

    # DSR_ROBOT2 함수가 내부에서 같은 노드로 spin하므로, 타이머 콜백 대신 단순 루프 사용
    try:
        while rclpy.ok():
            posx, _sol = get_current_posx(ref=DR_BASE)
            posj = get_current_posj()

            posx_pub.publish(Float64MultiArray(data=[float(v) for v in posx]))
            posj_pub.publish(Float64MultiArray(data=[float(v) for v in posj]))

            time.sleep(PERIOD)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Robot Error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
