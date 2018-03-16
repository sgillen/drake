from __future__ import print_function
# N.B. We are purposely not importing `division` to test nominal Python2 behavior.

import unittest
import numpy as np
from pydrake.autodiffutils import (AutoDiffXd)


class TestAutoDiffXd(unittest.TestCase):
    def _compare_scalar(self, actual, expected):
        self.assertAlmostEquals(actual.value(), expected.value())
        self.assertTrue((actual.derivatives() == expected.derivatives()).all())

    def test_div(self):
        x = AutoDiffXd(1, [1., 0])
        y = x/2.
        self._compare_scalar(y, AutoDiffXd(0.5, [0.5, 0]))

    def test_pow(self):
        x = AutoDiffXd(1., [1., 0., 0.])
        y = x**2
        self._compare_scalar(y, AutoDiffXd(1, [2., 0, 0]))
