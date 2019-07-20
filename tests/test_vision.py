from sneaky_lib.sneaky_vision import *

def test_distance():
    assert getDistanceToTarget(
        15.0, -5.0, 64.0, 200, 300) == pytest.approx(53, 5)
    assert getDistanceToTarget(
        15.0, 0.0, 64.0, 150, 300) == 0
    assert getDistanceToTarget(
        15.0, 5.0, 64.0, 100, 300) == pytest.approx(53, 5)


def test_yawAngle():
    assert getAngleToTarget(80.0, 150.0, 300.0) == 0
    assert getAngleToTarget(80.0, 300.0, 300.0) == 40.0
    assert getAngleToTarget(80.0, 225.0, 300.0) == 20.0
    assert getAngleToTarget(80.0, 0.0, 300.0) == -40.0

# TODO: Writes test for distance from yaw method

