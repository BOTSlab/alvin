

from math import isinf, isnan


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    """simple reimplementation of PEP 485"""
    # dirty sanity checking
    assert rel_tol > 0
    assert abs_tol >= 0
    assert not isnan(a)
    assert not isnan(b)

    if isinf(a) or isinf(b):
        # inf only equal to itself
        return a is b
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def allclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    """Test all in-order pairs of values in a and b are close to equal"""
    return all([isclose(x, y, rel_tol, abs_tol) for x, y in zip(a, b)])
