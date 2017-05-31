if(lcm_FOUND)
  add_library_with_exports(LIB_NAME monolithicPickAndPlaceDevCommon SOURCE_FILES
      demo_diagram_builder.cc
      state_machine_system.cc)
  target_link_libraries(monolithicPickAndPlaceDevCommon
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
      pickAndPlaceDevCommon
      drakeSensors
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(monolithicPickAndPlaceDevCommon)
  drake_install_headers(
        demo_diagram_builder.h
        pick_and_place_common.h
        state_machine_system.h)

  drake_install_pkg_config_file(monolithic-pick-and-place-dev-common
      TARGET monolithicPickAndPlaceDevCommon
      LIBS -lmonolithicPickAndPlaceDevCommon
      REQUIRES
        pick-and-place-dev-common
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
    monolithicPickAndPlaceDevCommon
    # TODO(eric.cousineau): IWYU
    )

  add_subdirectory(action_primitives)
endif()
