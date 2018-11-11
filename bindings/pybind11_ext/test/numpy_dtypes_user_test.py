import unittest

import numpy as np

import numpy_dtypes_user as mut


class TestNumpyDtypesUser(unittest.TestCase):
    def check_symbol(self, x, value, message=None):
        self.assertIsInstance(x, mut.Symbol)
        self.assertEqual(str(x), value, message)

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
        # Check idempotent round-trip casts.
        A = np.array([mut.Symbol("a")])
        for dtype in (mut.Symbol, np.object, mut.StrValueExplicit):
            B = A.astype(dtype)
            self.assertEqual(B.dtype, dtype)
            C = B.astype(mut.Symbol)
            self.assertEqual(C.dtype, mut.Symbol)
            self.check_symbol(C[0], "a")
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
        # By assignment.
        a = mut.Symbol("a")
        A = np.array([a])

        def reset():
            A[:] = a

        b_length = mut.LengthValueImplicit(1)
        # - Implicitly convertible types.
        # A[0] = b_length
        # self.check_symbol(A[0], "length(1)")
        A[:] = b_length
        self.check_symbol(A[0], "length(1)")
        # - Permitted as in place operation.
        reset()
        A += mut.LengthValueImplicit(1)
        self.check_symbol(A[0], "(a) + (length(1))")
        # Explicit: Scalar assignment not permitted.
        b_str = mut.StrValueExplicit("b")
        with self.assertRaises(TypeError):
            A[0] = b_str
        # N.B. For some reason, NumPy considers this explicit coercion...
        A[:] = b_str
        self.check_symbol(A[0], "b")
        # - Permitted as in place operation.
        reset()
        A += mut.StrValueExplicit("b")
        self.check_symbol(A[0], "(a) + (b)")
        reset()

    def test_array_creation_mixed(self):
        # Mixed creation with implicitly convertible types.
        with self.assertRaises(TypeError):
            # No type specified, NumPy gets confused.
            O = np.array([mut.Symbol(), mut.LengthValueImplicit(1)])
        A = np.array([
            mut.Symbol(), mut.LengthValueImplicit(1)], dtype=mut.Symbol)
        self.check_symbol(A[0], "")
        self.check_symbol(A[1], "length(1)")

        # Mixed creation without implicit casts, yields dtype=object.
        O = np.array([mut.Symbol(), 1.])
        self.assertEqual(O.dtype, np.object)
        # - Explicit Cast.
        A = O.astype(mut.Symbol)
        self.assertEqual(A.dtype, mut.Symbol)
        self.check_symbol(A[0], "")
        self.check_symbol(A[1], "float(1)")

        # Mixed creation with explicitly convertible types - does not work.
        with self.assertRaises(TypeError):
            A = np.array([
                mut.Symbol(), mut.StrValueExplicit("a")], dtype=mut.Symbol)

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

    def test_array_creation_constants_bad(self):
        """
        WARNING: The following are all BAD. AVOID THEM (as of NumPy v1.15.2).
        """
        # BAD Memory: `np.empty` works with uninitialized memory.
        # Printing will most likely cause a segfault.
        E = np.empty((2,), dtype=mut.Symbol)
        self.assertEqual(E.dtype, mut.Symbol)
        # BAD Memory: `np.zeros` works by using `memzero`.
        # Printing will most likely cause a segfault.
        Z = np.zeros((2,), dtype=mut.Symbol)
        self.assertEqual(Z.dtype, mut.Symbol)
        # BAD Semantics: This requires that `np.long` be added as an implicit
        # conversion.
        # Could add implicit conversion, but that may wreak havoc.
        with self.assertRaises(ValueError):
            I = np.ones((2,), dtype=mut.Symbol)

    def check_binary(self, a, b, fop, value):
        self.check_symbol(fop(a, b), value)
        A = np.array([a, a])
        B = np.array([b, b])
        c1, c2 = fop(A, B)
        self.check_symbol(c1, value)
        self.check_symbol(c2, value)

    def check_binary_with_inplace(
            self, a, b, fop, fiop, value, inplace_same=True):
        """
        Args:
            a: Left-hand operand.
            b: Right-hand operand.
            fop: Binary operator function function (x, y) -> z.
            fiop: Binary operator inplace function (x, y). Must return `x`.
            value: Expected value.
            inplace_same:
                For the scalar case, expects that `a += b` will not implicitly
                create a new instance (per Python's math rules). If False, a new
                instance must be created.
        """
        # Scalar.
        self.check_symbol(fop(a, b), value)
        c = mut.Symbol(a)
        d = fiop(c, b)
        if inplace_same:
            self.assertIs(c, d)
        else:
            self.assertIsNot(c, d)
        self.check_symbol(d, value)

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

    def test_algebra_closed(self):
        """Tests scalar and array algebra with implicit conversions."""
        a = mut.Symbol("a")
        b = mut.Symbol("b")

        # Operators.
        def fop(x, y): return x + y
        def fiop(x, y): x += y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) + (b)")

        def fop(x, y): return x - y
        def fiop(x, y): x -= y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) - (b)")

        def fop(x, y): return x * y
        def fiop(x, y): x *= y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) * (b)")

        def fop(x, y): return x / y
        def fiop(x, y): x /= y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) / (b)")

        def fop(x, y): return x & y
        def fiop(x, y): x &= y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) & (b)")

        def fop(x, y): return x | y
        def fiop(x, y): x |= y; return x
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) | (b)")

        # Logical.
        def fop(x, y): return x == y
        self.check_binary(a, b, fop, "(a) == (b)")

        def fop(x, y): return x != y
        self.check_binary(a, b, fop, "(a) != (b)")

        def fop(x, y): return x < y
        self.check_binary(a, b, fop, "(a) < (b)")

        def fop(x, y): return x <= y
        self.check_binary(a, b, fop, "(a) <= (b)")

        def fop(x, y): return x > y
        self.check_binary(a, b, fop, "(a) > (b)")

        def fop(x, y): return x >= y
        self.check_binary(a, b, fop, "(a) >= (b)")

    def test_algebra_implicit_casting(self):
        # N.B. Only tested on a single operator, `__add__` and `__iadd__`.
        a = mut.Symbol("a")

        def fop(x, y): return x + y
        def fiop(x, y): x += y; return x

        # N.B. Implicitly convertible types will enable true in-place
        # operations. Explicitly convertible types requires a new value.
        b_length = mut.LengthValueImplicit(1)
        self.check_binary_with_inplace(
            a, b_length, fop, fiop, "(a) + (length(1))", inplace_same=True)

        b_str = mut.StrValueExplicit("b")
        self.check_binary_with_inplace(
            a, b_str, fop, fiop, "(a) + (b)", inplace_same=False)
