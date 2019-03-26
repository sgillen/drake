# https://gist.github.com/gizatt/c4d4822a82861d3ce311a9e0761010e4

from __future__ import print_function

from collections import namedtuple
import functools
import os
import math
import matplotlib.pyplot as plt
import numpy as np
import random
import sys
import time

import pydrake
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.eigen_geometry import Quaternion, AngleAxis, Isometry3
from pydrake.forwarddiff import gradient, jacobian
from pydrake.geometry import (
    Box,
    Sphere
)
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    MultibodyPlant
)
from pydrake.multibody.tree import (
    PrismaticJoint,
    SpatialInertia,
    UniformGravityFieldElement,
    UnitInertia
)
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.systems.framework import DiagramBuilder


def demonstrateGoodInvocation():
    builder = DiagramBuilder()
    mbp, _ = AddMultibodyPlantSceneGraph(
        builder, MultibodyPlant(time_step=0.01))

    world_body = mbp.world_body()
    ground_shape = Box(10., 10., 10.)
    ground_body = mbp.AddRigidBody("ground", SpatialInertia(
        mass=10.0, p_PScm_E=np.array([0., 0., 0.]),
        G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    mbp.WeldFrames(world_body.body_frame(), ground_body.body_frame(),
                   Isometry3(rotation=np.eye(3), translation=[0, 0, -5]))
    mbp.RegisterVisualGeometry(
        ground_body, Isometry3(), ground_shape, "ground_vis",
        np.array([0.5, 0.5, 0.5, 1.]))
    mbp.RegisterCollisionGeometry(
        ground_body, Isometry3(), ground_shape, "ground_col",
        CoulombFriction(0.9, 0.8))

    n_bodies = 1
    for k in range(n_bodies):
        body = mbp.AddRigidBody("body_{}".format(k), SpatialInertia(
            mass=1.0, p_PScm_E=np.array([0., 0., 0.]),
            G_SP_E=UnitInertia(0.1, 0.1, 0.1)))

        body_box = Box(1.0, 1.0, 1.0)
        mbp.RegisterVisualGeometry(
            body, Isometry3(), body_box, "body_{}_vis".format(k),
            np.array([1., 0.5, 0., 1.]))
        mbp.RegisterCollisionGeometry(
            body, Isometry3(), body_box, "body_{}_box".format(k),
            CoulombFriction(0.9, 0.8))

    mbp.AddForceElement(UniformGravityFieldElement())
    mbp.Finalize()

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    mbp_context = diagram.GetMutableSubsystemContext(
        mbp, diagram_context)
    q0 = mbp.GetPositions(mbp_context).copy()

    print("MBP 1: ", mbp)
    InverseKinematics(mbp)
    print("Successfully made IK with MBP built in the same scope.")


def build_mbp():
    builder = DiagramBuilder()
    mbp, _ = AddMultibodyPlantSceneGraph(
        builder, MultibodyPlant(time_step=0.01))

    world_body = mbp.world_body()
    ground_shape = Box(10., 10., 10.)
    ground_body = mbp.AddRigidBody("ground", SpatialInertia(
        mass=10.0, p_PScm_E=np.array([0., 0., 0.]),
        G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    mbp.WeldFrames(world_body.body_frame(), ground_body.body_frame(),
                   Isometry3(rotation=np.eye(3), translation=[0, 0, -5]))
    mbp.RegisterVisualGeometry(
        ground_body, Isometry3(), ground_shape, "ground_vis",
        np.array([0.5, 0.5, 0.5, 1.]))
    mbp.RegisterCollisionGeometry(
        ground_body, Isometry3(), ground_shape, "ground_col",
        CoulombFriction(0.9, 0.8))

    n_bodies = 1
    for k in range(n_bodies):
        body = mbp.AddRigidBody("body_{}".format(k), SpatialInertia(
            mass=1.0, p_PScm_E=np.array([0., 0., 0.]),
            G_SP_E=UnitInertia(0.1, 0.1, 0.1)))

        body_box = Box(1.0, 1.0, 1.0)
        mbp.RegisterVisualGeometry(
            body, Isometry3(), body_box, "body_{}_vis".format(k),
            np.array([1., 0.5, 0., 1.]))
        mbp.RegisterCollisionGeometry(
            body, Isometry3(), body_box, "body_{}_box".format(k),
            CoulombFriction(0.9, 0.8))

    mbp.AddForceElement(UniformGravityFieldElement())
    mbp.Finalize()

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    mbp_context = diagram.GetMutableSubsystemContext(
        mbp, diagram_context)
    q0 = mbp.GetPositions(mbp_context).copy()

    print("MBP 1: ", mbp)
    return q0, mbp, mbp_context


def use_mbp(q0, mbp, mbp_context):
    print("MBP 3:", mbp)
    print("MBP 3 default context: ", mbp.CreateDefaultContext())
    InverseKinematics(mbp)
    print("Made IK without context")
    InverseKinematics(mbp, mbp_context)
    print("Made IK without context")


def demonstrateBadInvocation():
    q0, mbp, mbp_context = build_mbp()
    print("MBP 2: ", mbp)
    use_mbp(q0, mbp, mbp_context)


def main():
    demonstrateGoodInvocation()
    demonstrateBadInvocation()


if __name__ == "__main__":
    # main()

    import sys, trace
    sys.stdout = sys.stderr
    tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
    tracer.runfunc(main)
