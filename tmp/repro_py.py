import os.path

from pydrake.all import (
    DiagramBuilder, FloatingBaseType,
    RigidBodyPlant,
    RigidBodyTree,
    RigidBodyFrame,
    Simulator,
    ConstantVectorSource,
    DrakeVisualizer,
    SignalLogger,
    AddModelInstanceFromUrdfFile)
from pydrake.lcm import DrakeLcm

import time
import numpy as np

builder = DiagramBuilder()

tree = RigidBodyTree()
world_frame = RigidBodyFrame("world_frame", tree.world(), [0, 0, 0], [0, 0, 0])
AddModelInstanceFromUrdfFile("tmp/hamr_scaled.urdf",
                             FloatingBaseType.kFixed, world_frame, tree)

plant = RigidBodyPlant(tree, 0.01)

hamr = builder.AddSystem(plant)

# create Visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(
    DrakeVisualizer(tree=tree, lcm=lcm, enable_playback=True))

# no torque input
no_torques = ConstantVectorSource(np.zeros(tree.get_num_actuators()))
hamr_input = builder.AddSystem(no_torques)

# connect everything
builder.Connect(hamr_input.get_output_port(0), hamr.get_input_port(0))
builder.Connect(hamr.get_output_port(0), visualizer.get_input_port(0))
diagram = builder.Build()

# simulate
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(100)
simulator.set_publish_every_time_step(True)
hamr.set_state_vector(simulator.get_mutable_context(),
                      np.zeros(hamr.get_num_states()))
simulator.StepTo(50)
