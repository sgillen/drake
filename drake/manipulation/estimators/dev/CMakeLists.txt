find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

set(drakeManipulationTracker_SRC_FILES
    ManipulationTracker.cpp
    costs/DynamicsCost.cpp
    costs/AttachedApriltagCost.cpp
    costs/GelsightCost.cpp
    costs/OptotrakMarkerCost.cpp
    costs/RobotStateCost.cpp
    costs/JointStateCost.cpp
    costs/NonpenetratingObjectCost.cpp
    costs/KinectFrameCost.cpp
    )

set(drakeManipulationTracker_HEADERS
    common/sdf_2d_functions.hpp
    common/common.hpp
    ManipulationTracker.hpp
    costs/DynamicsCost.hpp
    costs/NonpenetratingObjectCost.hpp
    costs/KinectFrameCost.hpp
    costs/OptotrakMarkerCost.hpp
    costs/JointStateCost.hpp
    costs/GelsightCost.hpp
    costs/ManipulationTrackerCost.hpp
    costs/RobotStateCost.hpp
    costs/AttachedApriltagCost.hpp
    )

set(drakeManipulationTracker_EXTERNAL_LIBRARIES
    lcm
    lcmtypes_drc_lcmtypes
    bot2-core
    bot2-lcmgl-client
    bot2-frames
    bot2-param-client
    # kinect-utils
    ${OpenCV_LIBS}
    )

set(drakeManipulationTracker_LIBRARIES
    ${drakeManipulationTracker_EXTERNAL_LIBRARIES}
    drake-convex-hull
    drakeCommon
    drakeJoints
    drakeRBM
    drakeRigidBodyPlant
    drakeSystemFramework
    drakeUtil
    drakeXMLUtil)  # How to replace this?

add_library_with_exports(LIB_NAME drakeManipulationTracker SOURCE_FILES
    ${drakeManipulationTracker_SRC_FILES})

target_link_libraries(drakeManipulationTracker
    ${drakeManipulationTracker_LIBRARIES})

drake_install_libraries(drakeManipulationTracker)
drake_install_headers(${drakeManipulationTracker_HEADERS})
drake_install_pkg_config_file(drake-sensors
    TARGET drakeManipulationTracker
    LIBS -ldrakeManipulationTracker)

add_executable(runManipulationTracker runManipulationTracker.cpp)
target_link_libraries(runManipulationTracker drakeManipulationTracker)
drake_add_test(
    NAME runManipulationTracker COMMAND runManipulationTracker --simulation_sec 0.01)

add_executable(runErrorEstimator runErrorEstimator.cpp)
target_link_libraries(runErrorEstimator runErrorEstimator)
drake_add_test(
    NAME runErrorEstimator COMMAND runErrorEstimator --simulation_sec 0.01)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
