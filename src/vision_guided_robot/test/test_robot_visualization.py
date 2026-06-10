import pytest

from vision_guided_robot.robot_visualization import FootprintConfig, footprint_corners


def test_footprint_corners_form_closed_rectangle():
    corners = footprint_corners(FootprintConfig(length_m=0.4, width_m=0.2))

    assert len(corners) == 5
    assert corners[0] == corners[-1]
    assert corners[0][0] == pytest.approx(0.2)
    assert corners[0][1] == pytest.approx(0.1)
    assert corners[2][0] == pytest.approx(-0.2)
    assert corners[2][1] == pytest.approx(-0.1)
