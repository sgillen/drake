#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.analysis import (
    Simulator,
    )
from pydrake.systems.framework import (
    BasicVector,
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    VectorSystem,
    )
from pydrake.systems.primitives import (
    ZeroOrderHold,
    )

from pydrake.systems.test.test_util import (
    call_overrides,
    )


class CustomAdder(LeafSystem):
    # Reimplements `Adder`.
    def __init__(self, num_inputs, size):
        LeafSystem.__init__(self)
        for i in xrange(num_inputs):
            self._DeclareInputPort(PortDataType.kVectorValued, size)
        self._DeclareVectorOutputPort(BasicVector(size), self._calc_sum)

    def _calc_sum(self, context, sum_data):
        # @note This will NOT work if the scalar type is AutoDiff or symbolic,
        # since they are not stored densely.
        sum = sum_data.get_mutable_value()
        sum[:] = 0
        for i in xrange(context.get_num_input_ports()):
            input_vector = self.EvalVectorInput(context, i)
            sum += input_vector.get_value()

class CustomVectorSystem(VectorSystem):
    def __init__(self, is_discrete):
        # VectorSystem only supports pure Continuous or pure Discrete.
        # Dimensions:
        #   1 Input, 2 States, 3 Outputs.
        VectorSystem.__init__(self, 1, 3)
        self._is_discrete = is_discrete
        if self._is_discrete:
            self._DeclareDiscreteState(2)
        else:
            self._DeclareContinuousState(2)

        self.has_called = []

    def _DoCalcVectorOutput(self, context, input, state, output):
        # output = [input; state]
        output[:] = np.vstack(input, state)
        self.has_called.append("output")

    def _DoCalcVectorTimeDerivatives(self, context, input, state, derivatives):
        # xdot = x + [1; 1]*u
        derivatives[:] = state + input
        self.has_called.append("continuous")

    def _DoCalcVectorDiscreteVariableUpdates(self, context, input, state, next_state):
        # x[n+1] = x[n] + 2*[1; 1]*u
        next_state = state + 2*input
        self.has_called.append("discrete")


class TestCustom(unittest.TestCase):
    def _create_adder_system(self):
        system = CustomAdder(2, 3)
        return system

    def _fix_adder_inputs(self, context):
        self.assertEquals(context.get_num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([1, 2, 3]))
        context.FixInputPort(1, BasicVector([4, 5, 6]))

    def test_adder_execution(self):
        system = self._create_adder_system()
        context = system.CreateDefaultContext()
        self._fix_adder_inputs(context)
        output = system.AllocateOutput(context)
        self.assertEquals(output.get_num_ports(), 1)
        system.CalcOutput(context, output)
        value = output.get_vector_data(0).get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_adder_simulation(self):
        builder = DiagramBuilder()
        adder = builder.AddSystem(self._create_adder_system())
        adder.set_name("custom_adder")
        # Add ZOH so we can easily extract state.
        zoh = builder.AddSystem(ZeroOrderHold(0.1, 3))
        zoh.set_name("zoh")

        builder.ExportInput(adder.get_input_port(0))
        builder.ExportInput(adder.get_input_port(1))
        builder.Connect(adder.get_output_port(0), zoh.get_input_port(0))
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        self._fix_adder_inputs(context)

        simulator = Simulator(diagram, context)
        simulator.Initialize()
        simulator.StepTo(1)
        # Ensure that we have the outputs we want.
        state = diagram.GetMutableSubsystemState(zoh, context)
        value = state.get_discrete_state().get_data()[0].get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))

    def test_leaf_system_overrides(self):

        class TrivialSystem(LeafSystem):
            def __init__(self):
                LeafSystem.__init__(self)
                self.called_publish = False
                # Ensure we have desired overloads.
                self._DeclarePeriodicPublish(0.1)
                self._DeclarePeriodicPublish(0.1, 0)
                self._DeclarePeriodicPublish(period=0.1, offset=0)

            def _DoPublish(self, context, events):
                # Call base `DoPublish` to ensure that we do not get
                # recursion.
                LeafSystem._DoPublish(self, context, events)
                self.called_publish = True

        system = TrivialSystem()
        self.assertFalse(system.called_publish)
        call_leaf_system_overrides(system)
        self.assertTrue(system.called_publish)

    def test_vector_system_overrides(self):
        dt = 1.
        for is_discrete in [False, True]:
            system = CustomVectorSystem(is_discrete)
            context = system.CreateDefaultContext()
            input_vector = [1.]
            context.FixInputPort(0, BasicVector(input_vector))
            output = call_vector_system_overrides(
                system, context, is_discrete, dt)

            # Check call order.
            update_type = is_discrete and "discrete" or "continuous"
            assert system.has_called == [update_type, "output"]

            # Check values.
            state = context.GetState()
            state_vector = (is_discrete
                and state.get_discrete_state()
                or state.get_continuous_state())

            # Check output, `y = x + c*u`.
            coeff = is_discrete and 2 or 1
            y_expected = output.Get


if __name__ == '__main__':
    unittest.main()
