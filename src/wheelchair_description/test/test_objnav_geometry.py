import math

import numpy as np
import pytest

from objnav_geometry import (Grid, Hit, Hits, box_mask, mask_centre, object_depth,
                             pixel_to_optical, quat_to_matrix, standoff)


def test_grid_cell_floors_with_negative_origin():
    g = Grid(np.zeros((10, 10), np.int8), 0.1, -0.5, -0.5)
    assert g.cell(-0.45, -0.45) == (0, 0)
    assert g.cell(-0.01, 0.0) == (5, 4)


def test_grid_value_outside_is_unknown_with_negative_origin():
    g = Grid(np.zeros((10, 10), np.int8), 0.1, -0.5, -0.5)
    assert g.value(-0.55, 0.0) == -1  # int() would truncate -0.5 to cell 0 and report free
    assert g.value(0.0, 0.0) == 0


def test_object_depth_uses_near_quartile():
    depth = np.full((10, 10), 3.0)
    depth[:5] = 1.0
    assert object_depth(depth, np.ones((10, 10), bool)) == pytest.approx(1.0)


def test_object_depth_rejects_far_and_invalid():
    mask = np.ones((10, 10), bool)
    assert object_depth(np.full((10, 10), np.nan), mask) is None
    assert object_depth(np.full((10, 10), 6.0), mask) is None
    assert object_depth(np.full((10, 10), 0.1), mask) is None
    few = np.zeros((10, 10), bool)
    few[0, :5] = True  # 5 pixels: too few to trust
    assert object_depth(np.full((10, 10), 2.0), few) is None


def test_box_mask_is_central_half_and_centre_matches():
    m = box_mask((100, 100), [20, 40, 60, 80])
    ys, xs = np.nonzero(m)
    assert xs.min() == 30 and xs.max() == 50 and ys.min() == 50 and ys.max() == 70
    assert mask_centre(m) == pytest.approx((40.0, 60.0))


def test_pixel_to_optical():
    k = [500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1]
    assert pixel_to_optical(320, 240, 2.0, k) == pytest.approx([0, 0, 2.0])
    assert pixel_to_optical(820, 240, 2.0, k) == pytest.approx([2.0, 0, 2.0])


def test_quat_to_matrix_yaw_90():
    s = math.sin(math.pi / 4)
    r = quat_to_matrix(0, 0, s, s)
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 1.0, 0])
    assert quat_to_matrix(0, 0, 0, 1) == pytest.approx(np.eye(3))


def test_standoff_faces_object_from_first_free_spot():
    g = Grid(np.full((100, 100), 100, np.int8), 0.1, 0.0, 0.0)
    g.data[48:53, 29:34] = 0  # only around (3.1, 5.0) is free
    x, y, yaw = standoff((5.0, 5.0), (1.0, 5.0), g)
    assert (x, y, yaw) == pytest.approx((3.1, 5.0, 0.0))


def test_standoff_none_when_boxed_in():
    g = Grid(np.full((100, 100), 100, np.int8), 0.1, 0.0, 0.0)
    assert standoff((5.0, 5.0), (1.0, 5.0), g) is None


def hit(x, y, view=(0.0, 0.0, 0.0), cam="camera"):
    return Hit(np.array([x, y]), view, cam, None)


def test_one_viewpoint_is_not_enough():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0))
    assert h.ready() is None


def test_second_camera_confirms():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0, cam="right_camera"))
    centre, newest = h.ready()
    assert centre == pytest.approx([3.05, 0.0])
    assert newest.cam == "right_camera"


def test_moved_or_turned_viewpoint_confirms():
    moved = Hits()
    moved.add(hit(3.0, 0.0))
    moved.add(hit(3.0, 0.1, view=(0.4, 0.0, 0.0)))
    assert moved.ready() is not None
    turned = Hits()
    turned.add(hit(3.0, 0.0))
    turned.add(hit(3.0, 0.1, view=(0.0, 0.0, 0.5)))
    assert turned.ready() is not None


def test_far_apart_hits_do_not_cluster():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(5.0, 0.0, cam="right_camera"))
    assert h.ready() is None


def test_rejected_spot_ignores_new_hits():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0, cam="right_camera"))
    centre, _ = h.ready()
    h.reject(centre)
    h.add(hit(3.0, 0.1, cam="mapping_camera"))
    h.add(hit(3.1, 0.1, view=(1.0, 0.0, 0.0)))
    assert h.ready() is None
