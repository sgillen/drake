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
    def _check_scalar(self, actual, expected):
        if isinstance(actual, bool):
            self.assertTrue(isinstance(expected, bool))
            self.assertEquals(actual, expected)
        else:
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
                self._check_scalar(a, b)
        else:
            self.assertTrue(np.allclose(actual, expected))

    def test_scalar_api(self):
        a = AD(1, [1., 0])
        self.assertEquals(a.value(), 1.)
        self.assertTrue((a.derivatives() == [1., 0]).all())
        self.assertEquals(str(a), "AD{1.0, nderiv=2}")
        self.assertEquals(repr(a), "<AutoDiffXd 1.0 nderiv=2>")
        self._check_scalar(a, a)

    def test_array_api(self):
        a = AD(1, [1., 0])
        b = AD(2, [0, 1.])
        x = np.array([a, b])
        self.assertEquals(x.dtype, object)
        # Idempotent check.
        self._check_array(x, x)
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

    def _check_math(self, check):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        c_scalar = AD(0, [1., 0])
        d_scalar = AD(1, [0, 1.])
        a, b, c, d = map(check.reformat, (a_scalar, b_scalar, c_scalar, d_scalar))

        # Arithmetic
        check.check_value(a + b, AD(3, [1, 1]))
        check.check_value(a + 1, AD(2, [1, 0]))
        check.check_value(1 + a, AD(2, [1, 0]))
        check.check_value(a - b, AD(-1, [1, -1]))
        check.check_value(a - 1, AD(0, [1, 0]))
        check.check_value(1 - a, AD(0, [-1, 0]))
        check.check_value(a * b, AD(2, [2, 1]))
        check.check_value(a * 2, AD(2, [2, 0]))
        check.check_value(2 * a, AD(2, [2, 0]))
        check.check_value(a / b, AD(1./2, [1./2, -1./4]))
        check.check_value(a / 2, AD(0.5, [0.5, 0]))
        check.check_value(2 / a, AD(2, [-2, 0]))
        # Logical
        check.check_logical(lambda x, y: x == y, a, a, True)
        check.check_logical(lambda x, y: x != y, a, a, False)
        check.check_logical(lambda x, y: x < y, a, b, True)
        check.check_logical(lambda x, y: x <= y, a, b, True)
        check.check_logical(lambda x, y: x > y, a, b, False)
        check.check_logical(lambda x, y: x >= y, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        check.check_value(a**2, AD(1, [2., 0]))
        check.check_value(check.cos(c), AD(1, [0, 0]))
        check.check_value(check.sin(c), AD(0, [1, 0]))
        check.check_value(check.tan(c), AD(0, [1, 0]))
        check.check_value(check.arcsin(c), AD(0, [1, 0]))
        check.check_value(check.arccos(c), AD(np.pi / 2, [-1, 0]))
        check.check_value(check.arctan2(c, d), AD(0, [1, 0]))
        check.check_value(check.sinh(c), AD(0, [1, 0]))
        check.check_value(check.cosh(c), AD(1, [0, 0]))
        check.check_value(check.tanh(c), AD(0, [1, 0]))
        # Return value so it can be inspected.
        return a

    def test_scalar_math(self):
        a = self._check_math(ScalarMath(self._check_scalar))
        self.assertEquals(type(a), AD)

    def test_array_math(self):
        a = self._check_math(VectorizedMath(self._check_array))
        self.assertEquals(type(a), np.ndarray)
        self.assertEquals(a.shape, (2,))


class BaseMath(object):
    # Base class for defining scalar or vectorized (array) math checks.
    def __init__(self):
        # Derived classes should define extra math functions.
        pass

    def reformat(self, scalar):
        # Reformats a scalar to the given form.
        raise NotImplemented

    def check_value(self, actual, expected_scalar):
        raise NotImplemented

    def check_logical(self, actual, expected_scalar):
        raise NotImplemented


class ScalarMath(BaseMath):
    # Basic scalar element math.
    def __init__(self, check_value_impl):
        BaseMath.__init__(self)
        self._check_value_impl = check_value_impl
        # Math functions:
        self.sin = drake_math.sin
        self.cos = drake_math.cos
        self.tan = drake_math.tan
        self.arcsin = drake_math.asin
        self.arccos = drake_math.acos
        self.arctan2 = drake_math.atan2
        self.sinh = drake_math.sinh
        self.cosh = drake_math.cosh
        self.tanh = drake_math.tanh

    def reformat(self, scalar):
        return scalar

    def check_value(self, actual, expected_scalar):
        self._check_value_impl(actual, expected_scalar)

    def check_logical(self, func, a, b, expected_scalar):
        # Test overloads which have the same return values for:
        # - f(AutoDiffXd, AutoDiffXd)
        # - f(AutoDiffXd, float)
        # - f(float, AutoDiffXd)
        expected = self.reformat(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, b.value()), expected)
        self._check_value_impl(func(a.value(), b), expected)


class VectorizedMath(BaseMath):
    # Vectorized math for arrays.
    def __init__(self, check_value_impl):
        BaseMath.__init__(self)
        self._check_value_impl = check_value_impl
        # Math functions:
        self.sin = np.sin
        self.cos = np.cos
        self.tan = np.tan
        self.arcsin = np.arcsin
        self.arccos = np.arccos
        self.arctan2 = np.arctan2
        self.sinh = np.sinh
        self.cosh = np.cosh
        self.tanh = np.tanh

    def reformat(self, scalar):
        return np.array([scalar, scalar])

    def check_value(self, actual, expected_scalar):
        expected = self.reformat(expected_scalar)
        self._check_value_impl(actual, expected)

    def _array_to_float(self, a):
        return np.array([ai.value() for ai in a.flat]).reshape(a.shape)

    def check_logical(self, func, a, b, expected_scalar):
        # See above.
        af = self._array_to_float(a)
        bf = self._array_to_float(b)
        expected = self.reformat(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, bf), expected)
        self._check_value_impl(func(af, b), expected)
