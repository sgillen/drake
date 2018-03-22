from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import pydrake.autodiffutils as mut
from pydrake.autodiffutils import AutoDiffXd

import unittest
import numpy as np
import pydrake.math as drake_math

from pydrake.test.algebra_test_util import ScalarAlegbra, VectorizedAlgebra

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
            self.assertTrue((actual == expected).all())

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

    def _check_algebra(self, algebra):
        a_scalar = AD(1, [1., 0])
        b_scalar = AD(2, [0, 1.])
        c_scalar = AD(0, [1., 0])
        d_scalar = AD(1, [0, 1.])
        a, b, c, d = map(
            algebra.to_algebra, (a_scalar, b_scalar, c_scalar, d_scalar))

        # Arithmetic
        algebra.check_value(a + b, AD(3, [1, 1]))
        algebra.check_value(a + 1, AD(2, [1, 0]))
        algebra.check_value(1 + a, AD(2, [1, 0]))
        algebra.check_value(a - b, AD(-1, [1, -1]))
        algebra.check_value(a - 1, AD(0, [1, 0]))
        algebra.check_value(1 - a, AD(0, [-1, 0]))
        algebra.check_value(a * b, AD(2, [2, 1]))
        algebra.check_value(a * 2, AD(2, [2, 0]))
        algebra.check_value(2 * a, AD(2, [2, 0]))
        algebra.check_value(a / b, AD(1./2, [1./2, -1./4]))
        algebra.check_value(a / 2, AD(0.5, [0.5, 0]))
        algebra.check_value(2 / a, AD(2, [-2, 0]))
        # Logical
        algebra.check_logical(lambda x, y: x == y, a, a, True)
        algebra.check_logical(lambda x, y: x != y, a, a, False)
        algebra.check_logical(lambda x, y: x < y, a, b, True)
        algebra.check_logical(lambda x, y: x <= y, a, b, True)
        algebra.check_logical(lambda x, y: x > y, a, b, False)
        algebra.check_logical(lambda x, y: x >= y, a, b, False)
        # Additional math
        # - See `math_overloads_test` for scalar overloads.
        algebra.check_value(a**2, AD(1, [2., 0]))
        algebra.check_value(algebra.cos(c), AD(1, [0, 0]))
        algebra.check_value(algebra.sin(c), AD(0, [1, 0]))
        algebra.check_value(algebra.tan(c), AD(0, [1, 0]))
        algebra.check_value(algebra.arcsin(c), AD(0, [1, 0]))
        algebra.check_value(algebra.arccos(c), AD(np.pi / 2, [-1, 0]))
        algebra.check_value(algebra.arctan2(c, d), AD(0, [1, 0]))
        algebra.check_value(algebra.sinh(c), AD(0, [1, 0]))
        algebra.check_value(algebra.cosh(c), AD(1, [0, 0]))
        algebra.check_value(algebra.tanh(c), AD(0, [1, 0]))
        # Return value so it can be inspected.
        return a

    def test_scalar_algebra(self):
        a = self._check_algebra(ScalarAlegbra(self._check_scalar))
        self.assertEquals(type(a), AD)

    def test_array_algebra(self):
        a = self._check_algebra(VectorizedAlgebra(self._check_array))
        self.assertEquals(type(a), np.ndarray)
        self.assertEquals(a.shape, (2,))
