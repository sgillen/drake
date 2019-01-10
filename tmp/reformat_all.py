#!/usr/bin/python

import re
from common_scripts.text_processor import TextProcessor

from os.path import join, abspath, dirname, isfile

# find . -name '*.h' -o -name '*.cc' | xargs -n 2000 python tmp/reformat_all.py -i

# git diff HEAD~ --name-only
manifest = """
multibody/tree/internal/acceleration_kinematics_cache.cc
multibody/tree/internal/acceleration_kinematics_cache.h
multibody/tree/internal/articulated_body_inertia_cache.cc
multibody/tree/internal/articulated_body_inertia_cache.h
multibody/tree/internal/body_node.h
multibody/tree/internal/body_node_impl.cc
multibody/tree/internal/body_node_impl.h
multibody/tree/internal/body_node_welded.h
multibody/tree/internal/mobilizer.h
multibody/tree/internal/mobilizer_impl.cc
multibody/tree/internal/mobilizer_impl.h
multibody/tree/internal/multibody_tree.cc
multibody/tree/internal/multibody_tree.h
multibody/tree/internal/multibody_tree_context.cc
multibody/tree/internal/multibody_tree_context.h
multibody/tree/internal/multibody_tree_element.h
multibody/tree/internal/multibody_tree_forward_decl.h
multibody/tree/internal/multibody_tree_system.cc
multibody/tree/internal/multibody_tree_system.h
multibody/tree/internal/multibody_tree_topology.h
multibody/tree/internal/position_kinematics_cache.cc
multibody/tree/internal/position_kinematics_cache.h
multibody/tree/internal/prismatic_mobilizer.cc
multibody/tree/internal/prismatic_mobilizer.h
multibody/tree/internal/quaternion_floating_mobilizer.cc
multibody/tree/internal/quaternion_floating_mobilizer.h
multibody/tree/internal/revolute_mobilizer.cc
multibody/tree/internal/revolute_mobilizer.h
multibody/tree/internal/space_xyz_mobilizer.cc
multibody/tree/internal/space_xyz_mobilizer.h
multibody/tree/internal/velocity_kinematics_cache.cc
multibody/tree/internal/velocity_kinematics_cache.h
multibody/tree/internal/weld_mobilizer.cc
multibody/tree/internal/weld_mobilizer.h
""".strip().split()

template = """#pragma once

#include "drake/{new}"

#warning "This header is deprecated, and will be removed around 2019/03/01."
"""

workspace = join(dirname(abspath(__file__)), "..")

class Custom(TextProcessor):
    def __init__(self):
        TextProcessor.__init__(self, skipArgs = True, description = "Apply python regex to file")
        self.addArguments()

    def process(self, oldText):
        text = oldText
        for new in manifest:
            if not new.endswith(".h"):
                continue
            old = new.replace("multibody/tree/internal/", "multibody/tree/")
        return text

# Custom().main()

import os

for new in manifest:
    if not new.endswith(".h"):
        continue
    old = new.replace("multibody/tree/internal/", "multibody/tree/")
    p = join(workspace, old)
    if isfile(p):
        os.unlink(p)
    # with open(join(workspace, old), 'w') as f:
    #     f.write(template.format(new=new))
    print(old)
