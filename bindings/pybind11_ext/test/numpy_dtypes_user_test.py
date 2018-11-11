import unittest

import numpy as np

import numpy_dtypes_user as mut


class TestNumpyDtypesUser(unittest.TestCase):
    def check_symbol(self, sym, value):
        self.assertEqual(sym.str(), value)

    def test_scalar_meta(self):
        """Tests basic metadata."""
        self.assertTrue(issubclass(mut.Symbol, np.generic))
        self.assertIsInstance(np.dtype(mut.Symbol), np.dtype)

    def test_scalar_basics(self):
        """
        Tests basics for scalars.
        Important to do since we had to redo the instance registry to inherit
        from `np.generic` :(
        """
        c1 = mut.Symbol()
        c2 = mut.Symbol()
        self.assertIsNot(c1, c2)
        self.assertIs(c1, c1.self_reference())
        # TODO(eric.cousineau): Consider using `pybind11`s `ConstructorStats`
        # to do instance tracking.
        # Test functions.
        a = mut.Symbol("a")
        self.assertEqual(repr(a), "<Symbol 'a'>")
        self.assertEqual(str(a), "a")
        self.assertEqual(a.str(), "a")

    def test_algebra(self):

        def op_with_inplace(a, b, fop, fiop, value):
            # Scalar.
            self.check_symbol(fop(a, b), value)
            c = mut.Symbol(a)
            fiop(c, b)
            self.check_symbol(c, value)

            # Array.
            A = np.array([a, a])
            B = np.array([b, b])
            c1, c2 = fop(A, B)
            self.check_symbol(c1, value)
            self.check_symbol(c2, value)
            C = np.array(A)
            fiop(C, B)
            c1, c2 = C
            self.check_symbol(c1, value)
            self.check_symbol(c2, value)

        a = mut.Symbol("a")
        b = mut.Symbol("b")

        # N.B. Implicit casting is not easily testable here; see array tests.
        # Operators.
        def fop(a, b): return a + b
        def fiop(c, b): c += b
        op_with_inplace(a, b, fop, fiop, "(a) + (b)")

        def fop(a, b): return a - b
        def fiop(c, b): c -= b
        op_with_inplace(a, b, fop, fiop, "(a) - (b)")

        def fop(a, b): return a * b
        def fiop(c, b): c *= b
        op_with_inplace(a, b, fop, fiop, "(a) * (b)")

        def fop(a, b): return a / b
        def fiop(c, b): c /= b
        op_with_inplace(a, b, fop, fiop, "(a) / (b)")

        def fop(a, b): return a & b
        def fiop(c, b): c &= b
        op_with_inplace(a, b, fop, fiop, "(a) & (b)")

        def fop(a, b): return a | b
        def fiop(c, b): c |= b
        op_with_inplace(a, b, fop, fiop, "(a) | (b)")

        def op(a, b, fop, value):
            self.check_symbol(fop(a, b), value)
            A = np.array([a, a])
            B = np.array([b, b])
            c1, c2 = fop(A, B)
            self.check_symbol(c1, value)
            self.check_symbol(c2, value)

        # Logical.
        op(a, b, lambda a, b: a == b, "(a) == (b)")
        op(a, b, lambda a, b: a != b, "(a) != (b)")
        op(a, b, lambda a, b: a < b, "(a) < (b)")
        op(a, b, lambda a, b: a <= b, "(a) <= (b)")
        op(a, b, lambda a, b: a > b, "(a) > (b)")
        op(a, b, lambda a, b: a >= b, "(a) >= (b)")
