# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector, BasicVector_,
    Parameters, Parameters_,
    Value,
    VectorBase,
    )
from pydrake.systems.test.test_util import (
    make_unknown_abstract_value,
    MoveOnlyType,
)


def pass_through(x):
    return x


def int_list(x):
    return [int(xi) for xi in x]


class TestValue(unittest.TestCase):
    def assertArrayEqual(self, lhs, rhs):
        # TODO(eric.cousineau): Place in `pydrake.test.unittest_mixins`.
        lhs, rhs = np.array(lhs), np.array(rhs)
        if lhs.dtype == Expression or rhs.dtype == Expression:
            lhs, rhs = lhs.astype(Expression), rhs.astype(Expression)
            self.assertTrue(Expression.equal_to(lhs, rhs).all())
        else:
            self.assertTrue(np.allclose(lhs, rhs))

    def test_basic_vector(self):
        map(self._check_basic_vector, (float, AutoDiffXd, Expression))

    def _check_basic_vector(self, T):
        # Test constructing vectors of sizes [0, 1, 2], and ensure that we can
        # construct from both lists and `np.array` objects with no ambiguity.
        for n in [0, 1, 2]:
            for wrap in [pass_through, int_list, np.array]:
                # Ensure that we can get vectors templated on double by
                # reference.
                expected_init = wrap(map(float, range(n)))
                expected_add = wrap([x + 1 for x in expected_init])
                expected_set = wrap([x + 10 for x in expected_init])

                value_data = BasicVector_[T](expected_init)
                value = value_data.get_mutable_value()
                self.assertArrayEqual(value, expected_init)

                # Add value directly.
                value[:] += 1
                self.assertArrayEqual(value, expected_add)
                self.assertArrayEqual(value_data.get_value(), expected_add)
                self.assertArrayEqual(
                    value_data.get_mutable_value(), expected_add)

                # Set value from `BasicVector`.
                value_data.SetFromVector(expected_set)
                self.assertArrayEqual(value, expected_set)
                self.assertArrayEqual(value_data.get_value(), expected_set)
                self.assertArrayEqual(
                    value_data.get_mutable_value(), expected_set)
                # Ensure we can construct from size.
                value_data = BasicVector_[T](n)
                self.assertEqual(value_data.size(), n)
                # Ensure we can clone.
                value_copies = [
                    value_data.Clone(),
                    copy.copy(value_data),
                    copy.deepcopy(value_data),
                ]
                for value_copy in value_copies:
                    self.assertTrue(value_copy is not value_data)
                    self.assertEqual(value_data.size(), n)

    def test_basic_vector_set_get(self):
        value = BasicVector(np.arange(3., 5.))
        self.assertEqual(value.GetAtIndex(1), 4.)
        value.SetAtIndex(1, 5.)
        self.assertEqual(value.GetAtIndex(1), 5.)

    def test_abstract_value_copyable(self):
        expected = "Hello world"
        value = Value[str](expected)
        self.assertTrue(isinstance(value, AbstractValue))
        self.assertEqual(value.get_value(), expected)
        expected_new = "New value"
        value.set_value(expected_new)
        self.assertEqual(value.get_value(), expected_new)
        # Test docstring.
        self.assertFalse("unique_ptr" in value.set_value.__doc__)

    def test_abstract_value_move_only(self):
        obj = MoveOnlyType(10)
        # This *always* clones `obj`.
        self.assertEqual(
            str(Value[MoveOnlyType]),
            "<class 'pydrake.systems.framework.Value[MoveOnlyType]'>")
        value = Value[MoveOnlyType](obj)
        self.assertTrue(value.get_value() is not obj)
        self.assertEqual(value.get_value().x(), 10)
        # Set value.
        value.get_mutable_value().set_x(20)
        self.assertEqual(value.get_value().x(), 20)
        # Test custom emplace constructor.
        emplace_value = Value[MoveOnlyType](30)
        self.assertEqual(emplace_value.get_value().x(), 30)
        # Test docstring.
        self.assertTrue("unique_ptr" in value.set_value.__doc__)

    def test_abstract_value_py_object(self):
        expected = {"x": 10}
        value = Value[object](expected)
        # Value is by reference, *not* by copy.
        self.assertTrue(value.get_value() is expected)
        # Update mutable version.
        value.get_mutable_value()["y"] = 30
        self.assertEqual(value.get_value(), expected)
        # Cloning the value should perform a deep copy of the Python object.
        value_clone = copy.deepcopy(value)
        self.assertEqual(value_clone.get_value(), expected)
        self.assertTrue(value_clone.get_value() is not expected)
        # Using `set_value` on the original value changes object reference.
        expected_new = {"a": 20}
        value.set_value(expected_new)
        self.assertEqual(value.get_value(), expected_new)
        self.assertTrue(value.get_value() is not expected)

    def test_abstract_value_make(self):
        value = AbstractValue.Make("Hello world")
        self.assertTrue(isinstance(value, Value[str]))
        value = AbstractValue.Make(MoveOnlyType(10))
        self.assertTrue(isinstance(value, Value[MoveOnlyType]))
        value = AbstractValue.Make({"x": 10})
        self.assertTrue(isinstance(value, Value[object]))

    def test_abstract_value_unknown(self):
        value = make_unknown_abstract_value()
        self.assertTrue(isinstance(value, AbstractValue))
        with self.assertRaises(RuntimeError) as cm:
            value.get_value()
        self.assertTrue(all(
            s in cm.exception.message for s in [
                "AbstractValue",
                "UnknownType",
                "get_value",
                "AddValueInstantiation",
            ]), cm.exception.message)

    def test_parameters_api(self):
        map(self._check_parameters_api, (float, AutoDiffXd, Expression))

    def _check_parameters_api(self, T):
        Parameters = Parameters_[T]
        BasicVector = BasicVector_[T]

        def compare(actual, expected):
            self.assertEqual(type(actual), type(expected))
            if isinstance(actual, BasicVector):
                self.assertArrayEqual(actual.get_value(), expected.get_value())
            else:
                assert isinstance(actual, Value[str])
                # Strings getting converted to numpy arrays is no bueno. Do
                # scalar comparison.
                self.assertEqual(actual.get_value(), expected.get_value())

        model_numeric = BasicVector([0.])
        model_abstract = AbstractValue.Make("Hello")

        params = Parameters(
            numeric=[model_numeric.Clone()], abstract=[model_abstract.Clone()])
        self.assertEqual(params.num_numeric_parameters(), 1)
        self.assertEqual(params.num_abstract_parameters(), 1)
        # Numeric.
        compare(params.get_numeric_parameter(index=0), model_numeric)
        compare(params.get_mutable_numeric_parameter(index=0), model_numeric)
        # WARNING: This will invalidate old references!
        params.set_numeric_parameters(params.get_numeric_parameters().Clone())
        # Abstract.
        compare(params.get_abstract_parameter(index=0), model_abstract)
        compare(params.get_mutable_abstract_parameter(index=0), model_abstract)
        # WARNING: This will invalidate old references!
        params.set_abstract_parameters(
            params.get_abstract_parameters().Clone())
        # WARNING: This may invalidate old references!
        params.SetFrom(copy.deepcopy(params))

        # Test alternative constructors.
        ctor_test = [
            Parameters(),
            Parameters(numeric=[model_numeric.Clone()]),
            Parameters(abstract=[model_abstract.Clone()]),
            Parameters(
                numeric=[model_numeric.Clone()],
                abstract=[model_abstract.Clone()]),
            Parameters(vec=model_numeric.Clone()),
            Parameters(value=model_abstract.Clone()),
            ]
