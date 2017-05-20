drake_cc_library(
    name = "manipulation_tracker",
    srcs = [
        "ManipulationTracker.cpp",
        glob("costs/*.cpp"),
    ],
    hdrs = [
        "ManipulationTracker.hpp",
        glob("costs/*.hpp"),
    ],
    deps = [
        # HACK: Since VTK has the necessary GLUT libraries.
        "@vtk//:vtk_libraries",
    ],
)

drake_cc_binary(
    name = "run_manipulation_tracker",
    srcs = [
        "runManipulationTracker.cpp",
    ],
    deps = [
        ":manipulation_tracker",
    ],
)

drake_cc_googletest(
    
)
