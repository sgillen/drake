"""
To run:

    cd drake
    bazel run //tools:drake_visualizer -- \
        --script ${PWD}/tmp/director_marker_repro.py
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
