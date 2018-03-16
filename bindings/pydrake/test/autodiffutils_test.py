from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2
# behavior.

import unittest
import numpy as np
from pydrake.autodiffutils import AutoDiffXd


class TestAutoDiffXd(unittest.TestCase):
    def _compare_scalar(self, actual, expected):
        self.assertAlmostEquals(actual.value(), expected.value())
        self.assertTrue((actual.derivatives() == expected.derivatives()).all())

    def test_scalar_math(self):
        ad = AutoDiffXd
        a = ad(1, [1., 0])
        self._compare_scalar(a, a)
        b = ad(2, [0, 1.])
        self._compare_scalar(a + b, ad(3, [1, 1]))
        self._compare_scalar(a + 1, ad(2, [1, 0]))
        self._compare_scalar(1 + a, ad(2, [1, 0]))
        self._compare_scalar(a - b, ad(-1, [1, -1]))
        self._compare_scalar(a - 1, ad(0, [1, 0]))
        self._compare_scalar(1 - a, ad(0, [-1, 0]))
        self._compare_scalar(a * b, ad(2, [2, 1]))
        self._compare_scalar(a * 2, ad(2, [2, 0]))
        self._compare_scalar(2 * a, ad(2, [2, 0]))
        self._compare_scalar(a / b, ad(1./2, [1./2, -1./4]))
        self._compare_scalar(a / 2, ad(0.5, [0.5, 0]))
        self._compare_scalar(2 / a, ad(2, [-2, 0]))
        self._compare_scalar(a**2, ad(1, [2., 0]))
