"""
Provides consistent utilities for comparing NumPy matrices and scalars.

Prefer comparisons in the following order:

 - Methods from this module.
 - Methods form `np.testing.*`, if the dtypes are guaranteed to be NumPy
   builtins and the API definitely won't support other scalar types.
"""

from collections import namedtuple
from itertools import product

import numpy as np

# TODO(eric.cousineau): Make custom assert-vectorize which will output
# coordinates and stuff.


# Scalar
_AssertComparator = namedtuple('_AssertComparator', ['eq', 'ne'])
_comparators = {}
_to_float = {}


def _register_comparator(cls_a, cls_b, eq, ne=None):
    # N.B. This contract is fragile, and should not made public until it's
    # refined (e.g. ensuring str printing is informative, and returning boolean
    # values).
    key = (cls_a, cls_b)
    assert key not in _comparators, key
    eq = np.vectorize(eq)
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
    import pydrake.symbolic as _sym

    def _struct_eq(a, b):
        assert a.EqualTo(b), (a, b)

    def _struct_ne(a, b):
        assert not a.EqualTo(b), (a, b)

    _to_float[_sym.Expression] = _sym.Expression.Evaluate
    _register_comparator(_sym.Formula, str, _str_eq, _str_ne)
    _sym_lhs = [_sym.Variable, _sym.Expression, _sym.Polynomial, _sym.Monomial]
    _sym_rhs = _sym_lhs + [float]
    for _lhs in _sym_lhs:
        _register_comparator(_lhs, str, _str_eq, _str_ne)
    for _lhs, _rhs in product(_sym_lhs, _sym_rhs):
        _register_comparator(_lhs, _rhs, _struct_eq, _struct_ne)
except ImportError:
    pass


def _get_comparator_from_arrays(a, b):
    # Ensure all types are homogeneous.
    a_type, = {type(np.asarray(x).item()) for x in a.flat}
    b_type, = {type(np.asarray(x).item()) for x in b.flat}
    key = (a_type, b_type)
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


def _assert_not_equal_raw(a, b):
    assert a != b, (a, b)


def assert_not_equal(a, b):
    """Compare scalars or arrays directly, requiring inequality."""
    a, b = map(np.asarray, (a, b))
    assert not (a.size == 0 and b.size == 0)
    if a.dtype != object and b.dtype != object:
        assert_ne = _assert_not_equal_raw
    else:
        assert_ne = _get_comparator_from_arrays(a, b).ne
    # For this to fail, all items must have failed.
    br = np.broadcast(a, b)
    errs = []
    for ai, bi in br:
        e = None
        try:
            assert_ne(ai, bi)
        except AssertionError as e:
            # N.B. Fragile, assuming that this assertion error is actual for
            # inequaliy. For now, do not expose publicly.
            errs.append(str(e))
    assert len(errs) < br.size, errs
