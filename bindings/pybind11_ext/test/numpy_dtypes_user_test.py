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

    def test_scalar_algebra(self):
        a = mut.Symbol("a")
        b = mut.Symbol("b")

        def op(fop, value):
            self.check_symbol(fop(a, b), value)

        def op_with_inplace(fop, fiop, value):
            self.check_symbol(fop(a, b), value)
            c = mut.Symbol(a)
            fiop(c, b)
            self.check_symbol(c, value)

        # N.B. Implicit casting is not easily testable here; see array tests.
        # Operators.
        def fiop(c, b): c += b
        op_with_inplace(lambda a, b: a + b, fiop, "(a) + (b)")
        def fiop(c, b): c -= b
        op_with_inplace(lambda a, b: a - b, fiop, "(a) - (b)")
        def fiop(c, b): c *= b
        op_with_inplace(lambda a, b: a * b, fiop, "(a) * (b)")
        def fiop(c, b): c /= b
        op_with_inplace(lambda a, b: a / b, fiop, "(a) / (b)")
        def fiop(c, b): c &= b
        op_with_inplace(lambda a, b: a & b, fiop, "(a) & (b)")
        def fiop(c, b): c |= b
        op_with_inplace(lambda a, b: a | b, fiop, "(a) | (b)")
        # Logical.
        op(lambda a, b: a == b, "(a) == (b)")
        op(lambda a, b: a != b, "(a) != (b)")
        op(lambda a, b: a < b, "(a) < (b)")
        op(lambda a, b: a <= b, "(a) <= (b)")
        op(lambda a, b: a > b, "(a) > (b)")
        op(lambda a, b: a >= b, "(a) >= (b)")
