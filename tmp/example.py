# See: https://stackoverflow.com/questions/55224229

import numpy as np

from pydrake.all import (
    MultibodyPlant, Parser, UniformGravityFieldElement,
    LinearQuadraticRegulator, FindResourceOrThrow
)

file_name = FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf")
acrobot = MultibodyPlant()
parser = Parser(plant=acrobot)
parser.AddModelFromFile(file_name)
acrobot.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
acrobot.Finalize()

acrobot_context = acrobot.CreateDefaultContext()

shoulder = acrobot.GetJointByName("ShoulderJoint")
elbow = acrobot.GetJointByName("ElbowJoint")

shoulder.set_angle(context=acrobot_context, angle=0.0)
elbow.set_angle(context=acrobot_context, angle=0.0)

Q = np.identity(4)
R = np.identity(1)
N = np.zeros([4, 4])
controller = LinearQuadraticRegulator(acrobot, acrobot_context, Q, R)
