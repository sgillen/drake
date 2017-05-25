if(lcm_FOUND)
  add_library_with_exports(LIB_NAME monolithicPickAndPlaceCommon SOURCE_FILES
      demo_diagram_builder.cc
      state_machine_system.cc
      synchronous_world_state.cc)
  target_link_libraries(monolithicPickAndPlaceCommon
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      monolithicPickAndPlaceActionPrimitives
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
  drake_install_libraries(monolithicPickAndPlaceCommon)
  drake_install_headers(
        demo_diagram_builder.h
        pick_and_place_common.h
        state_machine_system.h
        synchronous_world_state.h)

  drake_install_pkg_config_file(monolithic-pick-and-place-common
      TARGET monolithicPickAndPlaceCommon
      LIBS -lmonolithicPickAndPlaceCommon
      REQUIRES
        pick-and-place-common
        drake-schunk-wsg
        monolithic-pick-and-place-action-primitives
        # and some other stuff
        drake-common
        drake-ik
        drake-kuka-iiwa-arm-common
        drake-lcm-system2
        drake-lcmtypes-cpp
        drake-lcm-util
        drake-rbm
        robotlocomotion-lcmtypes-cpp)

  add_executable(monolithic_pick_and_place_demo
    monolithic_pick_and_place_demo.cc)
  target_link_libraries(monolithic_pick_and_place_demo
    monolithicPickAndPlaceCommon
    # drakeKukaIiwaArmCommon
    # drakeManipulationPlanner
    # drakeLCMTypes
    # drakeLCMUtil
    # drakeRigidBodyPlant
    # pickAndPlaceCommon
    # robotlocomotion-lcmtypes-cpp
    )

  add_subdirectory(action_primitives)
endif()
