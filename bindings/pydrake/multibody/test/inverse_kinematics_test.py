from pydrake.multibody import inverse_kinematics as ik

import math
import unittest

import numpy as np

from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant 
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)
from pydrake.multibody.benchmarks.acrobot import (
    MakeAcrobotPlant,
)
import pydrake.solvers.mathematicalprogram as mp

from pydrake.common import FindResourceOrThrow
class TestInverseKinematics(unittest.TestCase):
    def setUp(self):
        file_name = FindResourceOrThrow(
            "drake/bindings/pydrake/multibody/test/two_bodies.sdf")
        self.plant = MultibodyPlant(time_step=0.01)
        model_instance = AddModelFromSdfFile(
            file_name=file_name, plant=self.plant, scene_graph=None)
        self.plant.Finalize()
        self.link1_frame = self.plant.GetBodyByName("body1").body_frame()
        self.link2_frame = self.plant.GetBodyByName("body2").body_frame()
        self.ik_two_bodies= ik.InverseKinematics(self.plant)
        self.prog = self.ik_two_bodies.get_mutable_prog()
        self.q = self.ik_two_bodies.q()

        def squaredNorm(x):
            return np.array([x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]])

        self.prog.AddConstraint(squaredNorm, [1], [1], self.q[:4])
        self.prog.AddConstraint(squaredNorm, [1], [1], self.q[7:11])

    def test_AddPositionConstraint(self):
        self.ik_two_bodies.AddPositionConstraint(self.link1_frame, [0, 0.1, 0.2], self.link2_frame, [-1, -1, -1], [1, 1, 1])
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)
        q_val = prog.GetSolution(self.q)
        print q_val

    def test_AddOrientationConstraint(self):
        self.ik_two_bodies.AddOrientationConstraint(self.link1_frame, self.link2_frame, 0.5 * math.pi)
        result = self.prog.Solve()
        self.assertEqual(result, mp.SolutionResult.kSolutionFound)