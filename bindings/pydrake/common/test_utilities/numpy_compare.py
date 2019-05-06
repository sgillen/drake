"""
Provides consistent utilities for comparing NumPy matrices and scalars.

Prefer comparisons in the following order:

 - Methods from this module.
 - Methods form `np.testing.*`, if the dtypes are guaranteed to be NumPy
   builtins and the API definitely won't support other scalar types.
"""

from collections import namedtuple

import numpy as np

# TODO(eric.cousineau): Make custom assert-vectorize which will output
# coordinates and stuff.


# Scalar
_AssertComparator = namedtuple('_AssertComparator', ['eq', 'ne'])
_comparators = {}
_to_float = {}


def _register_comparator(cls_a, cls_b, eq, ne=None):
    key = (cls_a, cls_b)
    assert key not in _comparators, key
    eq = np.vectorize(eq)
    if ne is not None:
        ne = np.vectorize(ne)
    _comparators[key] = _AssertComparator(eq, ne)


def _str_eq(a, b):
    a = str(a)
    assert a == b, (a, b)


def _str_ne(a, b):
    a = str(a)
    assert a != b, (a, b)


try:
    from pydrake.autodiffutils import AutoDiffXd

    def _ad_eq(a, b):
        assert a.value() == b.value, (a.value(), b.value())
        np.testing.assert_equal(a.derivatives(), b.derivatives())

    def _ad_ne(a, b):
        if a.value() != b.value():
            assert_not_equal(a.derivatives(), b.derivatives())
        else:
            assert False, (a.value(), b.value())

    _to_float[AutoDiffXd] = AutoDiffXd.value
    _register_comparator(AutoDiffXd, AutoDiffXd, _ad_eq, _ad_ne)
except ImportError:
    pass

try:
    from pydrake.symbolic import Expression, Formula

    _to_float[Expression] = Expression.Evaluate
    _register_comparator(Expression, str, _str_eq, _str_ne)
    _register_comparator(Formula, str, _str_eq, _str_ne)
except ImportError:
    pass


def _get_comparator_from_arrays(a, b):
    # Ensure all types are homogeneous.
    a_types = {type(x.item()) for x in a.flat}
    b_types = {type(x.item()) for x in b.flat}
    assert len(a_types) == 1, a_types
    assert a_types == b_types, (a_types, b_types)
    key = (a_types[0], b_types[0])
    return _comparators[key]


@np.vectorize
def to_float(x):
    """Converts scalar or array to floats."""
    x = np.asarray(x)
    if x.dtype == object:
        cls = type(x.item())
        return _to_float[cls](x)
    else:
        return np.float64(x)


def assert_equal(a, b):
    """Compare scalars or arrays directly, requiring equality."""
    a, b = map(np.asarray, (a, b))
    if a.size == 0 and b.size == 0:
        return
    if a.dtype != object and b.dtype != object:
        np.testing.assert_equal(a, b)
    else:
        _get_comparator_from_arrays(a, b).eq(a, b)


@np.vectorize
def _assert_not_equal_raw(a, b):
    assert a != b, (a, b)


def assert_not_equal(a, b):
    """Compare scalars or arrays directly, requiring inequality."""
    a, b = map(np.asarray, (a, b))
    assert not (a.size == 0 and b.size == 0)
    if a.dtype != object and b.dtype != object:
        _assert_not_equal_raw(a, b)
    else:
        _get_comparator_from_arrays(a, b).ne(a, b)
