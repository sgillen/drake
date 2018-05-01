import pydrake.systems.scalar_conversion as mut

import copy
import unittest

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import LeafSystem_, SystemScalarConverter


@mut.define_convertible_system("DefaultConverted")
def DefaultConverted(T):
    """Defines a simple templated class which defines a default
    converter."""

    class DefaultConvertedInstantiation(LeafSystem_[T]):
        # N.B. Do not define `__init__`, as this will be overridden.
        # Define `_construct` and `_construct_copy` instead.
        # You must define `converter` as an argument to ensure that it
        # propagates properly.
        def _construct(self, value, converter=None):
            LeafSystem_[T].__init__(self, converter)
            self.value = value
            self.copied_from = None

        def _construct_copy(self, other, converter=None):
            LeafSystem_[T].__init__(self, converter)
            self.value = other.value
            self.copied_from = other

    return DefaultConvertedInstantiation


class TestScalarConversion(unittest.TestCase):
    def test_converter_attributes(self):
        covnersion_scalars = (
            float, AutoDiffXd, Expression,
        )
        self.assertEqual(
            SystemScalarConverter.SupportedScalars,
            covnersion_scalars)
        conversion_pairs = (
            (AutoDiffXd, float),
            (Expression, float),
            (float, AutoDiffXd),
            (Expression, AutoDiffXd),
            (float, Expression),
            (AutoDiffXd, Expression),
        )
        self.assertEqual(
            SystemScalarConverter.SupportedConversionPairs,
            conversion_pairs)

    def test_convertible_system(self):
        # Test paramters.
        param_list = [(T,) for T in SystemScalarConverter.SupportedScalars]
        self.assertEqual(DefaultConverted.param_list, param_list)

        # Test private converter.
        converter = DefaultConverted._converter
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            self.assertTrue(converter.IsConvertible[T, U]())

        # Test calls that we have available for scalar conversion.
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            system_U = DefaultConverted[U](100)
            self.assertIs(system_U.copied_from, None)
            if T == AutoDiffXd:
                method = LeafSystem_[U].ToAutoDiffXd
            elif T == Expression:
                method = LeafSystem_[U].ToSymbolic
            else:
                continue
            system_T = method(system_U)
            self.assertIsInstance(system_T, DefaultConverted[T])
            self.assertEqual(system_T.value, 100)
            self.assertIs(system_T.copied_from, system_U)

    def test_define_convertible_system_api(self):
        """Tests more advanced API of `define_convertible_system`, both
        positive and negative tests."""

        def generic_instantiation_func(T):

            class GenericInstantiation(LeafSystem_[T]):
                def _construct(self, converter=None):
                    LeafSystem_[T].__init__(self, converter)

                def _construct_copy(self, other, converter=None):
                    LeafSystem_[T].__init__(self, converter)

            return GenericInstantiation

        # Non-symbolic
        # - Implicit conversion pairs.
        T_list = [float, AutoDiffXd]
        T_pairs_full = [
            (float, AutoDiffXd),
            (AutoDiffXd, float),
        ]
        A = mut.define_convertible_system("A", T_list=T_list)(
            generic_instantiation_func)
        self.assertEqual(A._T_list, T_list)
        self.assertEqual(A._T_pairs, T_pairs_full)

        # - Explicit conversion pairs.
        T_pairs = [
            (float, AutoDiffXd),
        ]
        B = mut.define_convertible_system("B", T_list=T_list, T_pairs=T_pairs)(
            generic_instantiation_func)
        self.assertEqual(B._T_list, T_list)
        self.assertEqual(B._T_pairs, T_pairs)

        # Negative tests.
        # - Not a supported scalar.
        T_list_bad = [int, float]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system("C", T_list=T_list_bad)
        # - Not in original param list.
        T_pairs_bad = [
            (float, Expression),
        ]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system(
                "C", T_list=T_list, T_pairs=T_pairs_bad)
        # - Unsupported conversion.
        T_pairs_unsupported = [
            (float, float),
        ]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system("C", T_pairs=T_pairs_unsupported)

    def test_bad_class_definitions(self):
        """Tests bad class definitions."""
        bad_init = "Convertible systems should not define"

        # No `__init__`.
        @mut.define_convertible_system("NoInit")
        def NoInit(T):

            class NoInitInstantiation(LeafSystem_[T]):
                def __init__(self):
                    pass

            return NoInitInstantiation

        with self.assertRaises(RuntimeError) as cm:
            NoInit[float]
        self.assertIn(bad_init, str(cm.exception))

        # No `_construct_copy`.
        @mut.define_convertible_system("NoConstructCopy")
        def NoConstructCopy(T):

            class NoConstructCopyInstantiation(LeafSystem_[T]):
                def _construct(self):
                    pass

            return NoConstructCopyInstantiation

        with self.assertRaises(RuntimeError) as cm:
            NoConstructCopy[float]
        self.assertIn(bad_init, str(cm.exception))

        # Does not inherit from `LeafSystem_[T]`.
        @mut.define_convertible_system("BadParenting")
        def BadParenting(T):

            class BadParentingInstantiation(object):
                def __init__(self):
                    pass

                def _construct(self):
                    pass

            return BadParentingInstantiation

        with self.assertRaises(RuntimeError) as cm:
            BadParenting[float]
        # N.B. Since we check the class before the instantiation has completed,
        # we will not have the templated name.
        self.assertIn("BadParentingInstantiation", str(cm.exception))
        self.assertIn("LeafSystem_[T]", str(cm.exception))
        self.assertIn("T=float", str(cm.exception))
