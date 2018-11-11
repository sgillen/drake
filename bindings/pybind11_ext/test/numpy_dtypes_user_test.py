import unittest

import numpy as np

import numpy_dtypes_user as mut


class TestNumpyDtypesUser(unittest.TestCase):
    def check_symbol(self, x, value):
        self.assertIsInstance(x, mut.Symbol)
        self.assertEqual(str(x), value)

    def check_symbol_all(self, X, value):
        for x in X.flat:
            self.assertEqual(str(x), value)

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

    def test_array_creation_basics(self):
        # Uniform creation.
        A = np.array([mut.Symbol("a")])
        self.assertEqual(A.dtype, mut.Symbol)
        self.assertEqual(A[0].str(), "a")

    def test_array_cast_explicit(self):
        # Check round-trip for semantically trivial casts.
        A = np.array([mut.Symbol("a")])
        for dtype in (mut.Symbol, np.object):
            B = A.astype(dtype)
            self.assertEqual(B.dtype, dtype)
            C = B.astype(mut.Symbol)
            self.assertEqual(C.dtype, mut.Symbol)
        # Check registered explicit casts.
        # - From.
        from_float = np.array([1.]).astype(mut.Symbol)
        self.check_symbol(from_float[0], "float(1)")
        from_str = np.array([mut.StrValueExplicit("abc")]).astype(mut.Symbol)
        self.check_symbol(from_str[0], "abc")
        from_length = np.array([mut.LengthValueImplicit(1)]).astype(mut.Symbol)
        self.check_symbol(from_length[0], "length(1)")
        # - To.
        # N.B. `np.int` may not be the same as `np.int32`; C++ uses `np.int32`.
        to_int = A.astype(np.int32)
        self.assertEqual(to_int[0], 1)
        to_str = A.astype(mut.StrValueExplicit)
        self.assertEqual(to_str[0].value(), "a")
        to_length = A.astype(mut.LengthValueImplicit)
        self.assertEqual(to_length[0].value(), 1)

    def test_array_cast_implicit(self):
        # Implicit casts by assignment.
        A = np.array([mut.Symbol()])
        A[0] = mut.LengthValueImplicit(1)
        A[0] = mut.StrValueExplicit("a")

    def test_array_creation_mixed(self):
        # Mixed creation -> object.
        O = np.array([mut.Symbol(), 1.])
        self.assertEqual(O.dtype, np.object)
        # - Explicit Cast.
        A = O.astype(mut.Symbol)
        self.assertEqual(A.dtype, mut.Symbol)
        self.check_symbol(A[0], "")
        self.check_symbol(A[1], "float(1)")

        # Mixed creation, but explicit.
        with self.assertRaises(TypeError):
            # Requires implicit conversion double. No dice.
            A = np.array([mut.Symbol(), 1.], dtype=mut.Symbol)

    def test_array_ufunc(self):
        # - Symbol
        a = mut.Symbol("a")
        b = mut.Symbol("b")
        C = mut.custom_binary_ufunc([a, a], [b, b])
        self.assertEqual(C.shape, (2,))
        self.check_symbol_all(C, "custom(a, b)")
        # - LengthValueImplicit
        al = mut.LengthValueImplicit(1)
        bl = mut.LengthValueImplicit(2)
        Cl = mut.custom_binary_ufunc([al, al], [bl, bl])
        self.assertEqual(Cl.shape, (2,))
        for c in Cl:
            self.assertEqual(c.value(), 3)

    def test_array_creation_constants(self):
        # Zeros: More so an `empty` array.
        Z = np.full((2,), mut.Symbol())
        self.assertEqual(Z.dtype, mut.Symbol)
        self.check_symbol_all(Z, "")

        # Zeros: For making an "empty" array, but using float conversion.
        Zf = np.zeros((2,)).astype(mut.Symbol)
        self.check_symbol_all(Zf, "float(0)")

        # Ones: Uses float conversion.
        I = np.ones((2,)).astype(mut.Symbol)
        self.check_symbol_all(I, "float(1)")

        # WARNING: The following are all BAD. AVOID THEM (as of NumPy v1.15.2).
        # BAD Memory: `np.empty` works with uninitialized memory.
        E = np.empty((2,), dtype=mut.Symbol)
        self.assertEqual(E.dtype, mut.Symbol)
        # BAD Memory: `np.zeros` works by using `memzero`.
        Z = np.zeros((2,), dtype=mut.Symbol)
        self.assertEqual(Z.dtype, mut.Symbol)
        # BAD Semantics: This requires that `np.long` be added as an implicit
        # conversion.
        with self.assertRaises(ValueError):
            I = np.ones((2,), dtype=mut.Symbol)

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
