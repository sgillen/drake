drake_add_cc_test(test_2D_sdf)
target_link_libraries(test_2D_sdf
    Eigen3::Eigen)

drake_add_cc_test(vector_slice_test)
target_link_libraries(vector_slice_test
    Eigen3::Eigen)

drake_add_cc_test(leaf_system_abstract_test)
target_link_libraries(leaf_system_abstract_test
  drakeSystemFramework
  drakeSystemAnalysis
  drakeSystemPrimitives)

# Meh. Ignore headers for now.
drake_add_cc_test(abstract_zoh_test)
target_link_libraries(abstract_zoh_test
  drakeSystemFramework
  drakeSystemAnalysis
  drakeSystemPrimitives)

drake_add_cc_test(pub_test)
target_link_libraries(pub_test
  drakeSystemFramework
  drakeSystemAnalysis
  drakeSystemPrimitives)
