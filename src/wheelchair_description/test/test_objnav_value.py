import math

import pytest

from objnav_value import ValueMap

HFOV = math.radians(90)


def test_view_scores_cells_ahead_only():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.3)
    assert vm.at(2.0, 0.0) == pytest.approx(0.3)
    assert math.isnan(vm.at(-2.0, 0.0))  # behind the camera
    assert math.isnan(vm.at(4.5, 0.0))  # beyond max_range
    assert math.isnan(vm.at(2.0, 3.0))  # outside the field of view


def test_views_blend_on_axis():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.2)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.4)
    assert vm.at(2.0, 0.0) == pytest.approx(0.3, abs=1e-3)


def test_yaw_turns_the_view():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, math.pi / 2, HFOV, 0.5)
    assert vm.at(0.0, 2.0) == pytest.approx(0.5)
    assert math.isnan(vm.at(2.0, 0.0))


def test_outside_the_map_is_unseen():
    assert math.isnan(ValueMap(size=10.0, res=0.1).at(50.0, 0.0))
