import unittest

import numpy as np

import pydrake.common.test_utilities.numpy_compare as npc

# Hack into private API to register custom comparisons.
class Toy(object):
    def __init__(self, value):
        assert isinstance(value, str)
        self._str = value

    def __str__(self):
        return self._str

    def assert_eq(self, other):
        assert self._str == other._str, (self, other)

    def assert_ne(self, other):
        assert self._str != other._str, (self, other)


npc._to_float[Toy] = lambda x: float(str(x))
npc._register_comparator(Toy, Toy, Toy.assert_eq, Toy.assert_ne)
npc._register_comparator(Toy, str, npc._str_eq, npc._str_ne)


class TestNumpyCompareSimple(unittest.TestCase):
    def test_to_float(self):
        xi = np.array([1, 2, 3], np.int)
        xf = npc.to_float(xi)
        self.assertEqual(xf.dtype, float)
        np.testing.assert_equal(xi, xf)

    def test_asserts_builtin(self):
        a = 1.
        b = 0.
        # Scalar.
        npc.assert_equal(a, a)
        with self.assertRaises(AssertionError):
            npc.assert_equal(a, b)
        npc.assert_not_equal(a, b)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(a, a)
        # Array.
        A = np.array([1., 1.])
        C = np.array([1., 2.])
        npc.assert_equal(A, a)
        npc.assert_equal(C, C)
        with self.assertRaises(AssertionError):
            npc.assert_equal(A, b)
        npc.assert_not_equal(A, A + [0, 0.1])
        npc.assert_not_equal(A, b)
        with self.assertRaises(AssertionError):
            npc.assert_not_equal(C, C)

    def test_asserts_custom(self):
        pass
