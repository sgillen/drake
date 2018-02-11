#!/usr/bin/env python

from __future__ import print_function

import unittest
import math
import numpy as np

from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake.math import BarycentricMesh
from pydrake.systems.controllers import (
    DynamicProgrammingOptions, FittedValueIteration)


class TestControllers(unittest.TestCase):
    def test_fitted_value_iteration_pendulum(self):
        plant = PendulumPlant()
        simulator = Simulator(plant)

        def quadratic_regulator_cost(context):
            print("got here")
#            x = context.get_continuous_state_vector().CopyToVector()
#            u = plant.EvalVectorInput(context,0)
#            print(type(x.dot(x) + u.dot(u)))
            return 0

        state_mesh = [set(np.linspace(0,2*math.pi,51)),
                      set(np.linspace(-10,10,51))]
        input_limit = 2
        input_mesh = [set(np.linspace(-input_limit, input_limit, 9))]
        timestep = 0.01

        options = DynamicProgrammingOptions()

        policy, value_function = FittedValueIteration(simulator,
                                             quadratic_regulator_cost,
                             state_mesh, input_mesh, timestep, options)

if __name__ == '__main__':
    unittest.main()
