import pydrake.systems.scalar_conversion as mut

import copy
import unittest

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import (
    LeafSystem_,
    SystemScalarConverter,
)
from pydrake.util.cpp_template import is_instantiation_of, TemplateClass


conversion_pairs = [
    # Synchronize with `ConversionPairs` `framework_py_systems.cc`.
    (AutoDiffXd, float),
    (Expression, float),
    (float, AutoDiffXd),
    (Expression, AutoDiffXd),
    (float, Expression),
    (AutoDiffXd, Expression),
]


@TemplateClass.define("ExplicitlyConverted", param_list=LeafSystem_.param_list)
def ExplicitlyConverted(ExplicitlyConverted, param):
    """Defines a simple templated class which does not define a default
    converter."""
    T, = param
    LeafSystem = LeafSystem_[T]
    scalar_helper = mut.ScalarHelper(ExplicitlyConverted)

    class ExplicitlyConvertedInstantiation(LeafSystem):
        # @note You can place a scalar type for traceability.
        scalar_type = T

        def __init__(self, *args, **kwargs):
            other = scalar_helper.check_if_copying(args, kwargs)
            if other:
                # Copy constructor; record input type.
                assert is_instantiation_of(type(other), ExplicitlyConverted)
                LeafSystem.__init__(self)
                self.value = other.value
                self.from_scalar_type = type(other).scalar_type
            else:
                self._construct(*args, **kwargs)

        def _construct(self, value, converter=None):
            if converter is None:
                LeafSystem.__init__(self)
            else:
                LeafSystem.__init__(self, converter)
            self.value = value
            self.from_scalar_type = None

    return ExplicitlyConvertedInstantiation


@TemplateClass.define("DefaultConverted", param_list=LeafSystem_.param_list)
def DefaultConverted(DefaultConverted, param):
    """Defines a simple templated class which defines a default converter."""
    T, = param
    LeafSystem = LeafSystem_[T]
    # N.B. Due to evaluation order on `add_instantiations`, do NOT create
    # converter here. Wait until first class instance is constructed.
    scalar_helper = mut.ScalarHelper(DefaultConverted)

    class DefaultConvertedInstantiation(LeafSystem):
        def __init__(self, *args, **kwargs):
            other = scalar_helper.check_if_copying(args, kwargs)
            if other:
                LeafSystem.__init__(self, scalar_helper.make_converter())
                self.value = other.value
            else:
                self._construct(*args, **kwargs)

        def _construct(self, value):
            LeafSystem.__init__(self, scalar_helper.make_converter())
            self.value = value

    return DefaultConvertedInstantiation


class TestScalarConversion(unittest.TestCase):
    def test_explicit_type_converter(self):
        # Tests bare basics.
        converter = SystemScalarConverter()
        value = 10

        for T, U in conversion_pairs:
            self.assertFalse(converter.IsConvertible[T, U]())
            system_U = ExplicitlyConverted[U](value)
            self.assertEqual(system_U.value, value)
            self.assertTrue(system_U.from_scalar_type is None)
            # Make a `dict` so we can mutate the values via closure and then
            # check their values.
            closure_mutables = dict(
                param=None,
                system_in=None,
                system_out=None)
            closure_mutables_none = copy.copy(closure_mutables)

            def conversion(system_in):
                # Trivial and meaningless conversion.
                self.assertIsInstance(system_in, ExplicitlyConverted[U])
                # Ensure we're seeing the same system we provided.
                self.assertIs(system_in, system_U)
                system_out = ExplicitlyConverted[T](system_in)
                closure_mutables.update(
                    param=(T, U),
                    system_in=system_in,
                    system_out=system_out)
                return system_out

            converter.Add[T, U](conversion)
            # Check conversion.
            system_T = converter.Convert[T, U](system_U)
            self.assertIsInstance(system_T, ExplicitlyConverted[T])
            self.assertEqual(system_T.value, system_U.value)
            self.assertEqual(system_T.from_scalar_type, U)
            # Check closure.
            self.assertEqual(closure_mutables["param"], (T, U))
            self.assertIs(closure_mutables["system_in"], system_U)
            self.assertIs(closure_mutables["system_out"], system_T)

            # Check conversion (if it's an exposed method) to ensure that we
            # can pass the converter.
            method = None
            if T == AutoDiffXd:
                method = LeafSystem_[U].ToAutoDiffXd
            elif T == Expression:
                method = LeafSystem_[U].ToSymbolic
            if method:
                # Reset closure.
                closure_mutables = copy.copy(closure_mutables_none)
                self.assertIs(closure_mutables["param"], None)
                system_U = ExplicitlyConverted[U](100, converter=converter)
                # N.B. With this structure, `system_T` is not actually
                # convertible since it has no converter.
                system_T = method(system_U)
                self.assertEqual(system_T.value, 100)
                self.assertEqual(system_T.from_scalar_type, U)
                self.assertEqual(closure_mutables["param"], (T, U))
                self.assertIs(closure_mutables["system_in"], system_U)
                self.assertIs(closure_mutables["system_out"], system_T)

    def test_default_type_converter(self):
        # Test calls that we have available for scalar conversion.
        for T, U in conversion_pairs:
            print(T, U)
            system_U = DefaultConverted[U](100)
            if T == AutoDiffXd:
                method = LeafSystem_[U].ToAutoDiffXd
            elif T == Expression:
                method = LeafSystem_[U].ToSymbolic
            else:
                continue
            system_T = method(system_U)
            self.assertIsInstance(system_T, DefaultConverted[T])
            self.assertEqual(system_T.value, 100)
