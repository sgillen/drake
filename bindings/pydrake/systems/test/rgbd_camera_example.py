import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.rigid_body_tree import (
    AddModelInstancesFromSdfString,
    FloatingBaseType,
    RigidBodyTree,
    RigidBodyFrame,
    )
from pydrake.systems.framework import (
    BasicVector,
    )
from pydrake.systems.sensors import (
    CameraInfo,
    RgbdCamera,
    )
from pydrake.util.eigen_geometry import Isometry3

sdf_path = FindResourceOrThrow(
    "drake/systems/sensors/test/models/box.sdf")
tree = RigidBodyTree()
with open(sdf_path) as f:
    sdf_string = f.read()
AddModelInstancesFromSdfString(
    sdf_string, FloatingBaseType.kFixed, None, tree)
frame = RigidBodyFrame(
    name="rgbd camera frame",
    body=tree.world(),
    xyz=[-20, 0, 0],
    rpy=[0, 0, 0])
tree.addFrame(frame)

camera = RgbdCamera(
    name="camera", tree=tree, frame=frame,
    z_near=0.5, z_far=5.0,
    fov_y=np.pi / 4, show_window=False)

print(camera.depth_camera_info().intrinsic_matrix())

import subprocess
subprocess.Popen(
    "export -p | sed 's# PWD=# OLD_PWD=#g' > tee /tmp/env.sh",
    shell=True)

# That we can access the state as images.
context = camera.CreateDefaultContext()
x0 = np.zeros(tree.get_num_positions() + tree.get_num_velocities())
context.FixInputPort(0, BasicVector(x0))
output = camera.AllocateOutput(context)
# camera.CalcOutput(context, output)
depth_index = camera.depth_image_output_port().get_index()
# depth_image = output.get_data(depth_index)
# print(depth_image)
