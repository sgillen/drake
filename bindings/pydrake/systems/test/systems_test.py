#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import numpy as np

# Emulate: //systems/framework:diagram_test, ExampleDiagram

from pydrake.systems import (
    # framework
    BasicVector,
    Diagram,
    DiagramBuilder,
    # WitnessFunctionDirection
    # primitives
    Adder,
    Integrator,
    # analysis
    Simulator,
    )

# from pydrake.systems.test.utilities import CustomLogger

size = 3

builder = DiagramBuilder()
adder0 = builder.AddSystem(Adder(2, size))
adder0.set_name("adder0")
adder1 = builder.AddSystem(Adder(2, size))
adder1.set_name("adder1")
adder2 = builder.AddSystem(Adder(2, size))
adder2.set_name("adder2")

# stateless = builder.AddSystem(
#     analysis_test.StatelessSystem(1.0, WitnessFunctionDirection.kCrossesZero))
# stateless.set_name("stateless")

integrator0 = builder.AddSystem(Integrator(size))
integrator0.set_name("integrator0")
integrator1 = builder.AddSystem(Integrator(size))
integrator1.set_name("integrator1")

builder.Connect(adder0.get_output_port(0), adder1.get_input_port(0))
builder.Connect(adder0.get_output_port(0), adder2.get_input_port(0))
builder.Connect(adder1.get_output_port(0), adder2.get_input_port(1))

builder.Connect(adder0.get_output_port(0), integrator0.get_input_port(0))
builder.Connect(integrator0.get_output_port(0), integrator1.get_input_port(0))

builder.ExportInput(adder0.get_input_port(0))
builder.ExportInput(adder0.get_input_port(1))
builder.ExportInput(adder1.get_input_port(1))
builder.ExportOutput(adder1.get_output_port(0))
builder.ExportOutput(adder2.get_output_port(0))
builder.ExportOutput(integrator1.get_output_port(0))

# builder.AddSystem(analysis_test.DoubleOnlySystem())

# # Add basic logging.
# data_points = []
# def log_callback(context):
#     global data_points
#     data_points.append(context.Clone())
# builder.AddSystem(CustomLogger(log_callback))

diagram = builder.Build()
diagram.set_name("test_diagram")  #"Unicode Snowman's Favorite Diagram!!1!☃!")
context = diagram.CreateDefaultContext()
output = diagram.AllocateOutput(context)

# Create and attach inputs.
input0 = BasicVector.Make(np.array([1, 2, 4]))
context.FixInputPort(0, input0)
input1 = BasicVector.Make([8, 16, 32])
context.FixInputPort(1, input1)
input2 = BasicVector.Make([64, 128, 256])
context.FixInputPort(2, input2)

# Initialize integrator states.
def get_mutable_continuous_state(system):
    return (diagram.GetMutableSubsystemState(system, context)
                   .get_mutable_continuous_state())

integrator0_xc = get_mutable_continuous_state(integrator0)
integrator0_xc.get_mutable_vector().SetFromVector([3, 9, 27])
integrator1_xc = get_mutable_continuous_state(integrator1)
integrator1_xc.get_mutable_vector().SetFromVector([81, 243, 729])

# https://stackoverflow.com/questions/4596962/display-graph-without-saving-using-pydot
print(diagram.GetGraphvizString())

# Simulate briefly.
simulator = Simulator(diagram, context)
simulator.StepTo(1)

assert context.get_time() == 1.

print(context)

# # Print stats.
# for pt in data_poitns:
#     print(pt)
