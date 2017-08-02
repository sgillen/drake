if (lcm_FOUND)
    drake_add_cc_test(icp_test)
    target_link_libraries(icp_test
        drakeArticulatedIcp
        drakeCommon
        drakeRigidBodyPlant  # For drake_visualizer
        lcmtypes_bot2-core-cpp
        )

    drake_add_cc_test(articulated_icp_test)
    target_link_libraries(articulated_icp_test
        drakeArticulatedIcp
        drakeCommon
        drakeRBM
        drakeRigidBodyPlant  # For drake_visualizer
        drakeMultibodyParsers
        drakeOptimization
        lcmtypes_bot2-core-cpp
        )
endif()
