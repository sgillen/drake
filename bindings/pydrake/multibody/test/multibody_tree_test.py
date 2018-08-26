# -*- coding: utf-8 -*-

from pydrake.multibody.multibody_tree import (
    # Indices.
    FrameIndex,
    BodyIndex,
    JointIndex,
    JointActuatorIndex,
    ModelInstanceIndex,
    world_index,
    # Elements.
    Frame,
    BodyFrame,
    Body,
    Joint,
    # Tree.
    MultibodyTree,
)
from pydrake.multibody.multibody_tree.math import (
    SpatialVelocity,
)
from pydrake.multibody.multibody_tree.multibody_plant import (
    MultibodyPlant,
)
from pydrake.multibody.multibody_tree.parsing import (
    AddModelFromSdfFile,
)

from pydrake.common import FindResourceOrThrow

import copy
import unittest
import numpy as np


class TestMath(unittest.TestCase):
    def test_spatial_velocity(self):
        velocity = SpatialVelocity()
        # - Accessors.
        self.assertTrue(isinstance(velocity.rotational(), np.ndarray))
        self.assertTrue(isinstance(velocity.translational(), np.ndarray))
        self.assertEqual(velocity.rotational().shape, (3,))
        self.assertEqual(velocity.translational().shape, (3,))
        # - Fully-parameterized constructor.
        w = [0.1, 0.3, 0.5]
        v = [0., 1., 2.]
        velocity1 = SpatialVelocity(w=w, v=v)
        self.assertTrue(np.allclose(velocity1.rotational(), w))
        self.assertTrue(np.allclose(velocity1.translational(), v))

    def test_type_safe_indices(self):
        """Existence tests."""
        self.assertEqual(world_index(), BodyIndex(0))

    def test_multibody_plant_api_via_parsing(self):
        # TODO(eric.cousineau): Decouple this when construction can be done
        # without parsing.
        # This a subset of `multibody_plant_sdf_parser_test.cc`.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        model_instance = AddModelFromSdfFile(plant)
        self.assertIsInstance(model_instnace, ModelInstanceIndex)
        plant.Finalize()
        benchmark = MakeAcrobotPlant(AcrobotParameters(), True)
        self.assertEqual(plant.num_bodies(), benchmark.num_bodies())
        self.assertEqual(plant.num_joints(), benchmark.num_joints())
        self.assertEqual(plant.num_actuators(), benchmark.num_actuators())
        self.assertEqual(
            plant.num_model_instances(), benchmark.num_model_instances())
        self.assertEqual(plant.num_positions(), benchmark.num_positions())
        self.assertEqual(
            plant.num_positions(model_instance=model_instance),
            benchamrk.num_positions(model_instance=model_instance))
        self.assertEqual(
            plant.num_velocities(), benchmark.num_velocities())
        self.assertEqual(
            plant.num_velocities(model_instance=model_instance),
            benchmark.num_velocities(model_instance=model_instance))
        self.assertEqual(
            plant.num_multibody_states(), benchmark.num_multibody_states())
        self.assertEqual(
            plant.num_actuated_dofs(), benchmark.num_actuated_dofs())
        self.assertTrue(plant.is_finalized())
        self.assertTrue(plant.HasBodyNamed(name="Link1"))
        self.assertTrue(plant.HasJointNamed(name="ShoulderJoint"))
        self._test_joint_api(plant.GetJointByName(name="ShoulderJoint"))
        self._test_joint_actuator_api(
            plant.GetJointActuatorNamed(name="ElbowJoint"))
        
        self.assertIsInstance(
            plant.GetBodyByName(name="Link1"), Body)
        self.assertIsInstance(
            plant.get_actuation_input_port(), InputPort)
        self.assertIsInstance(
            plant.get_continuous_state_output_port(), OutputPort)
        self.assertIsInstance(
            plant.get_contact_results_output_port(), OutputPort)

    def _test_joint_api(self, joint):
        self.assertIsInstance(joint, Joint)
        self.assertIsInstance(joint.name(), str)
        self.assertIsInstance(joint.parent_body(), Body)
        self.assertIsInstance(joint.child_body(), Body)
        self.assertIsInstance(joint.frame_on_parent(), Frame)
        self.assertIsInstance(joint.frame_on_child(), Frame)
        self.assertIsInstance(joint.num_dofs(), int)
