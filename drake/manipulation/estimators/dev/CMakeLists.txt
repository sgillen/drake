# HACK: Must use custom installation of OpenCV.
# Drake superbuild somehow does not permit the same paths as a normal CMake project,
# so we must forcefully change its mind.
# Consider setting via command-line: -DOpenCV_DIR=/path/to/opencv
unset(OpenCV_DIR CACHE)
unset(OpenCV_INCLUDE_DIRS CACHE)
unset(OpenCV_LIBRARIES CACHE)
unset(OpenCV_LIBS CACHE)
unset(OpenCV_INSTALL_PATH CACHE)
unset(OpenCV_CONFIG_PATH CACHE)

find_package(OpenCV REQUIRED
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    HINTS $ENV{OPENCV_CUSTOM_DIR})

include_directories(${OpenCV_DIRS})

#add_definitions(-fopenmp)
# @ref https://stackoverflow.com/questions/12399422/how-to-set-linker-flags-for-openmp-in-cmakes-try-compile-function
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(drakeDartUtil_SRC_FILES
    dart_util.cc
    )
set(drakeDartUtil_HEADERS
    dart_util.h
    vector_slice.h
    )

set(drakeManipulationTracker_SRC_FILES
    dart.cc
    dart_objectives.cc
    dart_depth_image_icp_objective.cc
#    articulated_state_estimator.cc

#    ManipulationTracker.cpp
#    ManipulationTrackerLoader.cpp
##    costs/AttachedApriltagCost.cpp
#    costs/DynamicsCost.cpp
##    costs/GelsightCost.cpp
#    costs/JointStateCost.cpp
#    costs/KinectFrameCost.cpp
#    costs/NonpenetratingObjectCost.cpp
##    costs/OptotrakMarkerCost.cpp
#    costs/RobotStateCost.cpp
    )

set(drakeManipulationTracker_HEADERS
    dart.h
    dart_objectives.h
    dart_depth_image_icp_objective.h
#    articulated_state_estimator.h
    abstract_zoh.h
    vector_slice.h
    stamped_value.h
    common/sdf_2d_functions.hpp
#    common/common.hpp
#    ManipulationTracker.hpp
#    ManipulationTrackerLoader.hpp
##    costs/AttachedApriltagCost.hpp
#    costs/DynamicsCost.hpp
##    costs/GelsightCost.hpp
#    costs/JointStateCost.hpp
#    costs/KinectFrameCost.hpp
#    costs/ManipulationTrackerCost.hpp
#    costs/NonpenetratingObjectCost.hpp
##    costs/OptotrakMarkerCost.hpp
#    costs/RobotStateCost.hpp
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
    drakeLCMTypes
    drakeRBM
    drakeRigidBodyPlant
    drakeMultibodyParsers
    drakeSystemFramework
    drakeSensors
    drakeDartUtil
    # drakeUtil
    # drakeXMLUtil
    )  # How to replace this?
# For drakeLCMTypes
include_directories(${CMAKE_BINARY_DIR})

include_directories(.)



add_library_with_exports(LIB_NAME drakeDartUtil SOURCE_FILES
    ${drakeDartUtil_SRC_FILES})
target_link_libraries(drakeDartUtil
    drakeCommon
    drakeJoints
    drakeLCMTypes
    drakeRBM
    drakeRigidBodyPlant
    drakeMultibodyParsers
    )


add_library_with_exports(LIB_NAME drakeManipulationTracker SOURCE_FILES
    ${drakeManipulationTracker_SRC_FILES})
target_link_libraries(drakeManipulationTracker
    ${drakeManipulationTracker_LIBRARIES})

# drake_install_libraries(drakeManipulationTracker)
# drake_install_headers(${drakeManipulationTracker_HEADERS})
# drake_install_pkg_config_file(drake-sensors
#     TARGET drakeManipulationTracker
#     LIBS -ldrakeManipulationTracker)

add_executable(runManipulationTracker runManipulationTracker.cpp)
target_link_libraries(runManipulationTracker drakeManipulationTracker)
drake_add_test(
    NAME runManipulationTracker COMMAND runManipulationTracker --simulation_sec 0.01)

if(FALSE)
    add_executable(runErrorCalculator runErrorCalculator.cpp)
    target_link_libraries(runErrorCalculator drakeManipulationTracker)
    drake_add_test(
        NAME runErrorCalculator COMMAND runErrorCalculator --simulation_sec 0.01)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
