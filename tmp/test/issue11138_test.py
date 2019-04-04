"""
Adapted from:
https://github.com/RussTedrake/underactuated/blob/5fedce4/src/van_der_pol/particles.py
"""

import unittest

import numpy as np

from pydrake.all import (
    AddRandomInputs,
    BasicVector,
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    RandomDistribution,
)


class VanDerPolParticles(LeafSystem):
    def __init__(self, num_particles, mu=1.0):
        LeafSystem.__init__(self)
        self.DeclareInputPort("noise",
                              PortDataType.kVectorValued,
                              num_particles,
                              RandomDistribution.kGaussian)
        self.DeclareContinuousState(num_particles, num_particles, 0)
        self.DeclareVectorOutputPort(BasicVector(2*num_particles),
                                     self.CopyStateOut)
        self.num_particles = num_particles
        self.mu = mu

    # TODO(russt):  SetRandomState to  [-0.1144;2.0578] + 0.01*randn(...)

    def DoCalcTimeDerivatives(self, context, derivatives):
        # TODO(russt):  Update this to get_position/velocity once those are
        # bound.
        x = context.get_continuous_state_vector().CopyToVector()
        q = x[:self.num_particles]
        qdot = x[self.num_particles:]
        w = self.EvalVectorInput(context, 0).CopyToVector()
        qddot = -self.mu * (q * q - 1) * qdot - q + .5 * w
        derivatives.get_mutable_vector().SetFromVector(np.concatenate((qdot,
                                                                       qddot)))

    # y(t) = x(t)
    def CopyStateOut(self, context, output):
        x = context.get_continuous_state_vector().get_value()
        y = output.get_mutable_value()
        y[:] = x


class TestBaddies(unittest.TestCase):
    def test_bad(self):
        num_particles = 5000
        builder = DiagramBuilder()
        sys = builder.AddSystem(VanDerPolParticles(num_particles))
        AddRandomInputs(.1, builder)
        diagram = builder.Build()


if __name__ == "__main__":
    unittest.main()
