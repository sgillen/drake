 import os.path

from pydrake.all import (
    DiagramBuilder, FloatingBaseType, RigidBodyPlant, RigidBodyTree, Simulator,
    VectorSystem, ConstantVectorSource, CompliantMaterial,
    CompliantContactModelParameters, DrakeVisualizer, AddFlatTerrainToWorld,
    SignalLogger, PiecewisePolynomial, TrajectorySource,
    AddModelInstanceFromUrdfFile)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType,
                                               RigidBodyFrame, RigidBodyTree)

from pydrake.lcm import DrakeLcm
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from hamr import *
from hamr_actuators import *
from hamr_with_actuators import *
from hamr_leg_control import *

builder = DiagramBuilder()

tree = RigidBodyTree()
world_frame = RigidBodyFrame("world_frame", tree.world(), [0, 0, 0], [0, 0, 0])
AddModelInstanceFromUrdfFile("hamrfull/models/urdf/hamr_scaled.urdf",
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

# np.set_printoptions(suppress=True, precision=4)
# # simulation params
# NCYC = 2
# builder = DiagramBuilder()

# # robot parameters
# urdf_type = HamrURDFType.kStandardHamr
# base_type = FloatingBaseType.kFixed
# dt = 0.001  # ms
# ustatic = 0.6

# # actuator parameters
# nact = 8
# Vmax = 225 * np.ones((nact, 1))
# Vmin = np.zeros((nact, 1))
# orien = np.array([-1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0])
# act_type = [HamrActuatorType.kOnePly] * nact

# # add hamr with actuators diagram
# hamr_w_act = builder.AddSystem(
#     HamrWithActuators(urdf_type, base_type, dt, ustatic, act_type, orien, Vmax,
#                       Vmin))

# # plant
# hamr = hamr_w_act.HamrP()

# # initial state
# x0 = hamr.HamrInitialState()

# # setup visualizer
# lcm = DrakeLcm()
# visualizer = builder.AddSystem(
#     DrakeVisualizer(
#         tree=hamr.get_rigid_body_tree(), lcm=lcm, enable_playback=True))

# # setup inputs
# freq = 0.002
# T = NCYC / freq
# amp = 0 * np.ones((nact, 1))
# offset = (Vmax / 2)
# tknot = np.arange(0, T, dt)

# fknot = hamr_w_act.HamrSinusoidalInput(freq, tknot[np.newaxis, :], amp, offset,
#                                        HamrGaitType.kTrot)
# ftraj = PiecewisePolynomial.FirstOrderHold(tknot, fknot)
# hamr_input = builder.AddSystem(TrajectorySource(ftraj, 0))

# # setup signal loggers
# hamr_state_logger = builder.AddSystem(
#     SignalLogger(hamr_w_act.get_output_port_state().size()))

# #connections
# builder.Connect(
#     hamr_input.get_output_port(0), hamr_w_act.get_input_port_voltage())

# builder.Connect(hamr_w_act.get_output_port_state(),
#                 hamr_state_logger.get_input_port(0))
# builder.Connect(hamr_w_act.get_output_port_state(),
#                 visualizer.get_input_port(0))

# # build
# diagram = builder.Build()
# simulator = Simulator(diagram)
# simulator.set_publish_every_time_step(True)
# simulator.set_target_realtime_rate(100)

# # simulate
# hamr.set_state_vector(simulator.get_mutable_context(), x0)
# simulator.Initialize()

# t0 = time.time()
# simulator.StepTo(T)
# telapsed = time.time() - t0
# print ("elapsed time: ", telapsed)
# print ("Fraction real_time: ", T / (telapsed * 1000))

# # data
# tt = hamr_state_logger.sample_times()
# xx = hamr_state_logger.data()
# N = tt.shape[0]

# # compute leg positons
# leg_positions = np.ndarray(shape=(3, 4, N))
# for i, x in enumerate(xx.T):
#     #print x.shape
#     q = x[0:hamr.get_num_positions()]
#     v = x[-hamr.get_num_velocities():]
#     leg_positions[:, :, i] = hamr.CalcHamrFootPosition(q, v, 0)

# dleg_pos = np.amax(leg_positions, 2) - np.amin(leg_positions, 2), 1
# # print dleg_pos

# plt.plot(tt, xx[2,:])
# print(xx[2, 0] - xx[2, -1])
# fig1, axs1 = plt.subplots(2, 2, figsize=(6, 6))
# axs1[0, 0].plot(leg_positions[0, 0, :] - np.mean(leg_positions[0, 0, :]),
#                 leg_positions[2, 0, :] - np.mean(leg_positions[2, 0, :]))
# axs1[0, 0].set_title('Front Left Leg')

# axs1[0, 1].plot(leg_positions[0, 1, :] - np.mean(leg_positions[0, 1, :]),
#                 leg_positions[2, 1, :] - np.mean(leg_positions[2, 1, :]))
# axs1[0, 1].set_title('Rear Left Leg')

# axs1[1, 0].plot(leg_positions[0, 2, :] - np.mean(leg_positions[0, 2, :]),
#                 leg_positions[2, 2, :] - np.mean(leg_positions[2, 2, :]))
# axs1[1, 0].set_title('Front Right Leg')

# axs1[1, 1].plot(leg_positions[0, 3, :] - np.mean(leg_positions[0, 3, :]),
#                 leg_positions[2, 3, :] - np.mean(leg_positions[2, 3, :]))
# axs1[1, 1].set_title('Rear Right Leg')

# print xx[:,-1]
# np.save('/home/nddoshi/Documents/python3-dev/hamr-dev-cpp/hamrfull/linearizations/models/approx_xfp_gnd', xx[:,-1])

plt.show()
