from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import unittest
import numpy as np
import pydrake.math as drake_math

# Use convenience abbreviation.
AD = AutoDiffXd


class TestAutoDiffXd(unittest.TestCase):
    def test_api(self):
        a = AD(1, [1., 0])
        self.assertEquals(a.value(), 1.)
        self.assertTrue((a.derivatives() == [1., 0]).all())
        self.assertEquals(str(a), "AD{1.0, nderiv=2}")
        self.assertEquals(repr(a), "<AutoDiffXd 1.0 nderiv=2>")

    def _compare_scalar(self, actual, expected):
        self.assertAlmostEquals(actual.value(), expected.value())
        self.assertTrue(
            (actual.derivatives() == expected.derivatives()).all(),
            (actual.derivatives(), expected.derivatives()))

    def _check_array(self, actual, expected):
        expected = np.array(expected)
        self.assertEquals(actual.dtype, expected.dtype)
        self.assertEquals(actual.shape, expected.shape)
        if actual.dtype == object:
            for a, b in zip(actual.flat, expected.flat):
                self._compare_scalar(a, b)
        else:
            self.assertTrue(np.allclose(actual, expected))

    def _check_logical(self, func, a, b, expect):
        # Test overloads which have the same return values for:
        # - f(AutoDiffXd, AutoDiffXd)
        # - f(AutoDiffXd, float)
        # - f(float, AutoDiffXd)
        self.assertEquals(func(a, b), expect)
        self.assertEquals(func(a, b.value()), expect)
        self.assertEquals(func(a.value(), b), expect)

    def _check_logical_array(self, func, a, b, expect):
        # Test overloads which have the same return values for:
        # - f(AutoDiffXd, AutoDiffXd)
        # - f(AutoDiffXd, float)
        # - f(float, AutoDiffXd)
        af = np.array([ai.value() for ai in a.flat]).reshape(a.shape)
        bf = np.array([bi.value() for bi in b.flat]).reshape(b.shape)
        self._check_array(func(a, b), expect)
        self._check_array(func(a, bf), expect)
        self._check_array(func(af, b), expect)

    def test_scalar_math(self):
        a = AD(1, [1., 0])
        self._compare_scalar(a, a)
        b = AD(2, [0, 1.])
        # Arithmetic
        self._compare_scalar(a + b, AD(3, [1, 1]))
        self._compare_scalar(a + 1, AD(2, [1, 0]))
        self._compare_scalar(1 + a, AD(2, [1, 0]))
        self._compare_scalar(a - b, AD(-1, [1, -1]))
        self._compare_scalar(a - 1, AD(0, [1, 0]))
        self._compare_scalar(1 - a, AD(0, [-1, 0]))
        self._compare_scalar(a * b, AD(2, [2, 1]))
        self._compare_scalar(a * 2, AD(2, [2, 0]))
        self._compare_scalar(2 * a, AD(2, [2, 0]))
        self._compare_scalar(a / b, AD(1./2, [1./2, -1./4]))
        self._compare_scalar(a / 2, AD(0.5, [0.5, 0]))
        self._compare_scalar(2 / a, AD(2, [-2, 0]))
        # Logical
        af = a.value()
        self._check_logical(lambda x, y: x == y, a, a, True)
        self._check_logical(lambda x, y: x != y, a, a, False)
        self._check_logical(lambda x, y: x < y, a, b, True)
        self._check_logical(lambda x, y: x <= y, a, b, True)
        self._check_logical(lambda x, y: x > y, a, b, False)
        self._check_logical(lambda x, y: x >= y, a, b, False)
        # Additional math
        self._compare_scalar(a**2, AD(1, [2., 0]))
        # Test autodiff overloads.
        # See `math_overloads_test` for more comprehensive checks.
        c = AD(0, [1., 0])
        d = AD(1, [0, 1.])
        self._compare_scalar(drake_math.sin(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.cos(c), AD(1, [0, 0]))
        self._compare_scalar(drake_math.tan(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.asin(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.acos(c), AD(np.pi / 2, [-1, 0]))
        self._compare_scalar(drake_math.atan2(c, d), AD(0, [1, 0]))
        self._compare_scalar(drake_math.sinh(c), AD(0, [1, 0]))
        self._compare_scalar(drake_math.cosh(c), AD(1, [0, 0]))
        self._compare_scalar(drake_math.tanh(c), AD(0, [1, 0]))

    def test_array_creation(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        c = AD(0, [1., 0])
        d = AD(1, [0, 1.])
        av = np.array([a])
        bv = np.array([b])
        cv = np.array([c])
        dv = np.array([d])
        x = np.array([c, d])
        self.assertEquals(x.dtype, object)
        # Conversion.
        with self.assertRaises(TypeError):
            # Avoid implicit coercion, as this will imply information loss.
            xf = np.zeros(2, dtype=np.float)
            xf[:] = x
        with self.assertRaises(TypeError):
            # We could define `__float__` to allow this, but then that will
            # enable implicit coercion, which we should avoid.
            xf = x.astype(dtype=np.float)
        # Presently, does not convert.
        x = np.zeros((3, 3), dtype=AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        x = np.eye(3).astype(AD)
        self.assertFalse(isinstance(x[0, 0], AD))
        # Arithmetic
        self._check_array(av + bv, [AD(3, [1, 1])])
        self._check_array(av + 1, [AD(2, [1, 0])])
        self._check_array(1 + av, [AD(2, [1, 0])])
        self._check_array(av - bv, [AD(-1, [1, -1])])
        self._check_array(av - 1, [AD(0, [1, 0])])
        self._check_array(1 - av, [AD(0, [-1, 0])])
        self._check_array(av * bv, [AD(2, [2, 1])])
        self._check_array(av * 2, [AD(2, [2, 0])])
        self._check_array(2 * av, [AD(2, [2, 0])])
        self._check_array(av / bv, [AD(1./2, [1./2, -1./4])])
        self._check_array(av / 2, [AD(0.5, [0.5, 0])])
        self._check_array(2 / av, [AD(2, [-2, 0])])
        # Logical
        af = a.value()
        self._check_logical_array(lambda x, y: x == y, av, av, [True])
        self._check_logical_array(lambda x, y: x != y, av, av, [False])
        self._check_logical_array(lambda x, y: x < y, av, bv, [True])
        self._check_logical_array(lambda x, y: x <= y, av, bv, [True])
        self._check_logical_array(lambda x, y: x > y, av, bv, [False])
        self._check_logical_array(lambda x, y: x >= y, av, bv, [False])
        # Additional math
        self._check_array(av**2, [AD(1, [2., 0])])
        self._check_array(np.cos(cv), [AD(1, [0, 0])])
        self._check_array(np.sin(cv), [AD(0, [1, 0])])
        self._check_array(np.tan(cv), [AD(0, [1, 0])])
        self._check_array(np.arcsin(cv), [AD(0, [1, 0])])
        self._check_array(np.arccos(cv), [AD(np.pi / 2, [-1, 0])])
        self._check_array(np.arctan2(cv, dv), [AD(0, [1, 0])])
        self._check_array(np.sinh(cv), [AD(0, [1, 0])])
        self._check_array(np.cosh(cv), [AD(1, [0, 0])])
        self._check_array(np.tanh(cv), [AD(0, [1, 0])])


import sys
sys.stdout = sys.stderr
