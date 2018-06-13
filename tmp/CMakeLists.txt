cmake_minimum_required(VERSION 3.5)

project(OSPRayPathTracer)

find_package(VTK 8.1 REQUIRED)
include(${VTK_USE_FILE})

add_executable(OSPRayPathTracer OSPRayPathTracer.cxx)
target_link_libraries(OSPRayPathTracer ${VTK_LIBRARIES})

add_executable(rgbd_ospray_minimal rgbd_ospray_minimal.cxx)
target_link_libraries(rgbd_ospray_minimal ${VTK_LIBRARIES})
