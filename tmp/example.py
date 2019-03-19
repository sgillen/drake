# See: https://stackoverflow.com/questions/55224229

import matplotlib.pyplot as plt
import numpy as np

from pydrake.all import (
    MultibodyPlant, Parser, UniformGravityFieldElement,
    LinearQuadraticRegulator, FindResourceOrThrow,
    BasicVector, plot_system_graphviz,
)

file_name = FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf")
plant = MultibodyPlant()
parser = Parser(plant=plant)
acrobot = parser.AddModelFromFile(file_name)
plant.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
plant.Finalize()

# Show figure.
plot_system_graphviz(plant, max_depth=2)
plt.savefig("/tmp/mbp_graphviz.png")

acrobot_input_port = plant.get_actuation_input_port(acrobot)
shoulder = plant.GetJointByName("ShoulderJoint")
elbow = plant.GetJointByName("ElbowJoint")

nq = plant.num_positions()
nv = plant.num_velocities()
nx = nq + nv
nu = plant.num_actuators()
assert (nx, nu) == (4, 1)

plant_context = plant.CreateDefaultContext()
plant_context.FixInputPort(
    plant.GetInputPort("applied_generalized_force").get_index(),
    BasicVector([0.] * plant.num_velocities()))
plant_context.FixInputPort(
    acrobot_input_port.get_index(),
    BasicVector([0.]))
shoulder.set_angle(context=plant_context, angle=0.0)
elbow.set_angle(context=plant_context, angle=0.0)

Q = np.identity(nx)
R = np.identity(nu)
N = np.zeros([nx, nu])
controller = LinearQuadraticRegulator(
    plant, plant_context, Q=Q, R=R, N=N,
    input_port_index=int(acrobot_input_port.get_index()))
