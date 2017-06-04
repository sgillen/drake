if(lcm_FOUND)
  add_library_with_exports(LIB_NAME monolithicPickAndPlaceDevCommon SOURCE_FILES
      state_machine_system.cc)
  target_link_libraries(monolithicPickAndPlaceDevCommon
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      pickAndPlaceCommon
      pickAndPlaceStateMachine
      drakeManipulationPlanner
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRigidBodyPlant
      drakeSystemFramework
      drakeMultibodyParsers
      drakeTrajectories
      drakeSchunkWsg
      drakeSensors
      drakeManipulationTracker
      robotlocomotion-lcmtypes-cpp)

  add_executable(monolithic_pick_and_place_demo
    monolithic_pick_and_place_demo.cc
    perception_hack.cc)
  target_link_libraries(monolithic_pick_and_place_demo
    monolithicPickAndPlaceDevCommon
    # TODO(eric.cousineau): IWYU
    )
endif()
