# https://cmake.org/cmake/help/v3.3/command/set.html
# Scope should be fine.

if(lcm_FOUND)
  set(cur_lib monolithicPickAndPlaceActionPrimitives)
  set(cur_pkg monolithic-pick-and-place-action-primitives)

  add_library_with_exports(LIB_NAME ${cur_lib} SOURCE_FILES
      action_primitive_base.cc
      gripper_action.cc
      iiwa_move.cc)
  target_link_libraries(${cur_lib}
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      drakeManipulationPlanner
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRigidBodyPlant
      drakeSystemFramework
      drakeMultibodyParsers
      drakeTrajectories
      drakeSchunkWsg
      pickAndPlaceCommon
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(${cur_lib})
  drake_install_headers(
        action_primitive_base.h
        action_primitives_common.h
        gripper_action.h
        iiwa_move.h)

  drake_install_pkg_config_file(${cur_pkg}
      TARGET ${cur_lib}
      LIBS -l${cur_lib}
      REQUIRES
        pick-and-place-common
        drake-schunk-wsg
        # and some other stuff
        drake-common
        drake-ik
        drake-kuka-iiwa-arm-common
        drake-lcm-system2
        drake-lcmtypes-cpp
        drake-lcm-util
        drake-rbm
        robotlocomotion-lcmtypes-cpp)
endif()
