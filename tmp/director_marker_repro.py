"""
To run:

    cd drake
    # Terminal 1
    bazel run //tools:drake_visualizer -- \
        --script ${PWD}/tmp/director_marker_repro.py

    # Terminal 2
    bazel run //manipulation/util:show_model -- \
        ${PWD}/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf
"""

from director import affordancemanager

manager = affordancemanager.AffordanceObjectModelManager(view)
obj = manager.newAffordanceFromDescription(dict(
    Name="test_affordance",
    classname="BoxAffordanceItem",
    # (xyz, q_wxyz)
    pose=((0, 0, 0), (1, 0, 0, 0)),
    # Asymmetric box.
    Dimensions=[0.01, 0.02, 0.03],
))
obj.getChildFrame().setProperty("Edit", True)
