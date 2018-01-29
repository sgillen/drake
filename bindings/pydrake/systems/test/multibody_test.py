import numpy as np
import unittest

from pyrdake.rbtree import RigidBodyTree

import pydrake.systems.multibody as mut



class TestMultibody(unittest.TestCase):
    def test_api(self):
        urdf_path = os.path.join(
            pydrake.getDrakePath(), "examples/pendulum/Pendulum.urdf")
        for is_discrete in [False, True]:
            tree = RigidBodyTree(urdf_path)
            if is_discrete:
                timestep = 0.0
                plant = mut.RigidBodyPlant(tree)
            else:
                timestep = 0.1
                plant = mut.RigidBodyPlant(tree, timestep)

            # Tested in order in which they're declared in `multibody_py.cc`,
            # when able.
            self.assertTrue(plant.get_rigid_body_tree() is tree)
            self.assertEquals(plant.get_num_bodies(), tree.get_num_bodies())
            
            self.assertEquals(
                plant.get_num_positions(), tree.get_num_positions())
            self.assertEquals(
                plant.get_num_positions(0), tree.get_num_positions())

            self.assertEquals(
                plant.get_num_velocities(), tree.get_num_velocities())
            self.assertEquals(
                plant.get_num_velocities(0), tree.get_num_velocities())

            self.assertEquals(
                plant.get_num_states(), tree.get_num_states())
            self.assertEquals(
                plant.get_num_states(0), tree.get_num_states())

            self.assertEquals(
                plant.get_num_actuators(), tree.get_num_actuators())
            self.assertEquals(
                plant.get_num_actuators(0), tree.get_num_actuators())

            self.assertEquals(
                plant.get_num_model_instances(), 1)
            self.assertEquals(plant.get_input_size(), plant.get_num_actuators())
            self.assertEquals(plant.get_output_size(), plant.get_num_states())

            context = plant.AllocateContext()
            x = plant.GetStateVector(context)
            plant.SetDefaultState(context)
            plant.set_position(context, 0, 1.)
            self.assertEquals(x[0], 1.)
            plant.set_velocity(context, 0, 2.)
            self.assertEquals(x[1], 2.)
            plant.set_state_vector(context, [3., 3.])
            self.assertTrue(np.allclose(x, [3., 3.]))
            plant.set_state_vector(context.get_mutable_state(), [4., 4.])
            self.assertTrue(np.allclose(x, [4., 4.]))

            self.assertEquals(
                plant.FindInstancePositionIndexFromWorldIndex(0, 1), 1)

            # Existence checks.
            self.assertTrue(plant.actuator_command_input_port() is not None)
            self.assertTrue(plant.model_instance_has_actuators())
            self.assertTrue(
                plant.model_instance_actuator_command_input_port() is not None)
            self.assertTrue(
                plant.state_output_port() is not None)
            self.assertTrue(
                plant.model_instance_state_output_port() is not None)
            self.assertTrue(
                plant.torque_output_port() is not None)
            self.assertTrue(
                plant.model_instance_torque_output_port() is not None)
            self.assertTrue(
                plant.kinematics_results_output_port() is not None)
            self.assertTrue(
                plant.contact_results_output_port() is not None)

            # Basic status.
            self.assertEquals(plant.is_state_discrete(), is_discrete)
            self.assertEquals(plant.get_time_step(), timestep)

if __name__ == '__main__':
    unittest.main()
