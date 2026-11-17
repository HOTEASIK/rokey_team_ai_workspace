"""집기와 반납이 동일한 측정 XYZABC를 사용하는지 검사한다."""

from putty_demo.pickup_spatula import (
    PICK_POSE, PICKUP_ABC, RECORDED_HOME_POSE, pickup,
)
from putty_demo.return_spatula import return_waypoints


def test_pickup_uses_taught_pose_after_home_lift():
    moves = []
    pickup(RECORDED_HOME_POSE, 50, 0,
           lambda label, pose, speed: moves.append((label, pose)),
           lambda *args: None, lambda seconds: None)
    assert moves[0][1][3:] == RECORDED_HOME_POSE[3:]
    assert moves[1][1][:2] == PICK_POSE[:2]
    assert moves[1][1][3:] == PICKUP_ABC
    assert moves[2][1] == PICK_POSE
    assert moves[-1][1] == moves[1][1]


def test_return_uses_pickup_orientation_instead_of_home_orientation():
    before, after = return_waypoints((500, 0, 300, 0, 90, 0), RECORDED_HOME_POSE, 245.14)
    assert before[-1][1] == PICK_POSE
    assert before[-2][1][3:] == PICKUP_ABC
    assert after[-1][1][:2] == PICK_POSE[:2]
    assert after[-1][1][3:] == PICKUP_ABC
