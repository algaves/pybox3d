import math

import pytest


@pytest.fixture
def approx():
    """Shorthand for pytest.approx with a tolerance suited to float32 math
    (libbox3d uses `float`, not `double`, throughout)."""

    def _approx(expected, rel=1e-4, abs=1e-4):
        return pytest.approx(expected, rel=rel, abs=abs)

    return _approx


@pytest.fixture
def pi():
    return math.pi
