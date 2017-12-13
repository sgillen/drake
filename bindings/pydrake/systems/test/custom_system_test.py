#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import unittest
import numpy as np

from pydrake.systems.framework import (
    LeafSystem,
    PortDataType,
    )


class CustomAdder(LeafSystem):
    # Reimplements `Adder`.
    def __init__(self, num_inputs, size):
        LeafSystem.__init__(self)
        for i in xrange(num_inputs):
            self.DeclareInputPort(PortDataType.kVectorValued, size)
        self.DeclareVectorOutputPort(BasicVector(size), self._calc_sum)

    def _calc_sum(context, sum):
        value = sum.get_mutable_value()
        value[:] = 0
        for i in xrange(context.get_num_input_ports()):
            input_vector = self.EvalVectorInput(context, i)
            value = input_vector.get_value()


class TestCustomSystem(unittest.TestCase):
    def test_custom_adder(self):
        system = CustomAdder(2, 3)
        self._test_execution(system)
        self._test_simulation(system)
        # Ensure that, even though `simulator` has been destroyed,
        # we can still access `system`.
        self._test_execution(self, system)

    def _setup_inputs(self, context):
        context.FixInputPort(0, BasicVector([1, 2, 3]))
        context.FixInputPort(1, BasicVector([4, 5, 6]))

    def _test_execution(system):
        context = system.CreateDefaultContext()
        self._setup_inputs(context)
        assert context.get_num_input_ports() == 2
        output = system.AllocateOutput(context)
        system.CalcOutput(context, output)
        value = output.get_value()
        self.assertTrue(np.allclose([5, 7, 9], value))

    def _test_simulation(self, system):
        pass

    def test_lifetime(self):
        simulator = Simulator(CustomAdder(2, 3))
        simulator.Initialize()
        context = simulator.get_mutable_context()
        self._setup_inputs(context)
        simulator.StepTo(1)
        # Ensure that we have the outputs we want.
        context.get_output()
        self.assertTrue(False)


if __name__ == '__main__':
    unittest.main()
