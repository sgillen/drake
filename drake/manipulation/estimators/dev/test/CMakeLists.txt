drake_add_cc_test(test_2D_sdf)
target_link_libraries(test_2D_sdf
    Eigen3::Eigen)

drake_add_cc_test(kinematic_state_portion_test)
target_link_libraries(kinematic_state_portion_test
    Eigen3::Eigen)
