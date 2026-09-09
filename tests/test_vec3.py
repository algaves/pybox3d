import pytest

from pybox3d import Vec3


def test_default_construction():
    v = Vec3()
    assert (v.x, v.y, v.z) == (0.0, 0.0, 0.0)


def test_positional_and_keyword_construction():
    assert Vec3(1, 2, 3).to_tuple() == (1.0, 2.0, 3.0)
    assert Vec3(x=1, y=2, z=3).to_tuple() == (1.0, 2.0, 3.0)
    assert Vec3(y=5).to_tuple() == (0.0, 5.0, 0.0)


def test_add_sub_operators():
    a = Vec3(1, 2, 3)
    b = Vec3(4, 5, 6)
    assert (a + b).to_tuple() == pytest.approx((5, 7, 9))
    assert (b - a).to_tuple() == pytest.approx((3, 3, 3))


def test_scalar_multiply():
    v = Vec3(1, 2, 3)
    assert (v * 2).to_tuple() == pytest.approx((2, 4, 6))


def test_negate():
    v = Vec3(1, -2, 3)
    assert (-v).to_tuple() == pytest.approx((-1, 2, -3))


def test_dot_orthogonal_is_zero():
    assert Vec3(1, 0, 0).dot(Vec3(0, 1, 0)) == pytest.approx(0.0)


def test_dot_parallel():
    assert Vec3(2, 0, 0).dot(Vec3(3, 0, 0)) == pytest.approx(6.0)


def test_cross_right_hand_rule():
    x = Vec3(1, 0, 0)
    y = Vec3(0, 1, 0)
    z = x.cross(y)
    assert z.to_tuple() == pytest.approx((0, 0, 1))


def test_length_and_length_squared():
    v = Vec3(3, 4, 0)
    assert v.length() == pytest.approx(5.0)
    assert v.length_squared() == pytest.approx(25.0)


def test_normalize():
    v = Vec3(0, 5, 0)
    n = v.normalized()
    assert n.to_tuple() == pytest.approx((0, 1, 0))


def test_normalize_zero_vector_returns_zero():
    n = Vec3(0, 0, 0).normalized()
    assert n.to_tuple() == pytest.approx((0, 0, 0))


def test_equality():
    assert Vec3(1, 2, 3) == Vec3(1, 2, 3)
    assert Vec3(1, 2, 3) != Vec3(1, 2, 4)


def test_dot_and_cross_accept_plain_sequences():
    v = Vec3(1, 0, 0)
    assert v.dot((0, 1, 0)) == pytest.approx(0.0)
    assert v.cross((0, 1, 0)).to_tuple() == pytest.approx((0, 0, 1))


def test_repr_is_readable():
    assert repr(Vec3(1, 2, 3)) == "Vec3(1.0, 2.0, 3.0)"
