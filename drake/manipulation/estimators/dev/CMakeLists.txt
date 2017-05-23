find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

set(drakeManipulationTracker_SRC_FILES
    ManipulationTracker.cpp
    costs/AttachedApriltagCost.cpp
    costs/DynamicsCost.cpp
    costs/GelsightCost.cpp
    costs/JointStateCost.cpp
#    costs/KinectFrameCost.cpp
    costs/NonpenetratingObjectCost.cpp
#    costs/OptotrakMarkerCost.cpp
    costs/RobotStateCost.cpp
    )

set(drakeManipulationTracker_HEADERS
    common/sdf_2d_functions.hpp
    common/common.hpp
    ManipulationTracker.hpp
    costs/AttachedApriltagCost.hpp
    costs/DynamicsCost.hpp
    costs/GelsightCost.hpp
    costs/JointStateCost.hpp
#    costs/KinectFrameCost.hpp
    costs/ManipulationTrackerCost.hpp
    costs/NonpenetratingObjectCost.hpp
#    costs/OptotrakMarkerCost.hpp
    costs/RobotStateCost.hpp
    )

set(drakeManipulationTracker_EXTERNAL_LIBRARIES
    lcm
    # lcmtypes_drc_lcmtypes
    lcmtypes_bot2-core-cpp
    # bot2-core
    bot2-lcmgl-client
    # <HACK>
    # -- Cannot set the library name?
    # bot2-frames
    ${CMAKE_INSTALL_RPATH}/libbot2-frames.so
    # bot2-param-client
    ${CMAKE_INSTALL_RPATH}/libbot2-param-client.so
    # </HACK>
    # kinect-utils
    ${OpenCV_LIBS}
    yaml-cpp
    )

set(drakeManipulationTracker_LIBRARIES
    ${drakeManipulationTracker_EXTERNAL_LIBRARIES}
    # drake-convex-hull
    drakeCommon
    drakeJoints
    drakeRBM
    drakeRigidBodyPlant
    drakeMultibodyParsers
    drakeSystemFramework
    # drakeUtil
    # drakeXMLUtil
    )  # How to replace this?

include_directories(.)

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

add_executable(runErrorCalculator runErrorCalculator.cpp)
target_link_libraries(runErrorCalculator drakeManipulationTracker)
drake_add_test(
    NAME runErrorCalculator COMMAND runErrorCalculator --simulation_sec 0.01)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
