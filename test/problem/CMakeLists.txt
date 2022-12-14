# Dynamics 
add_executable(dynamics_test
  dynamics_test.cpp
  dynamics_test.hpp
)
target_link_libraries(dynamics_test
  PRIVATE
  gtest_main
  problem
  common
  fmt::fmt
  utils
)
gtest_discover_tests(dynamics_test)

# Cost function test
add_executable(costfunction_test
  costfunction_test.cpp
)
target_link_libraries(costfunction_test
  PRIVATE
  gtest_main
  problem
  common
  fmt::fmt
  utils
)
gtest_discover_tests(costfunction_test)

# Triple integrator
add_executable(triple_integrator_test
  triple_integrator_test.cpp
)
target_link_libraries(triple_integrator_test
  PRIVATE
  gtest_main
  common
  dynamics_examples
  utils
)
gtest_discover_tests(triple_integrator_test)

# Unicycle
add_executable(unicycle_test
  unicycle_test.cpp
)
target_link_libraries(unicycle_test
  PRIVATE
  gtest_main
  dynamics_examples
  common
  utils
)
gtest_discover_tests(unicycle_test)

# Problem
add_executable(problem_test
  problem_test.cpp
)
target_link_libraries(problem_test
  PRIVATE
  constraints
  constraint_examples
  gtest_main
  problem
  dynamics_examples
  cost_function_examples
  utils
)
gtest_discover_tests(problem_test)

# Quadratic Cost 
add_executable(quadratic_cost_test
  quadratic_cost_test.cpp
)
target_link_libraries(quadratic_cost_test
  PRIVATE
  gtest_main
  cost_function_examples
  common
  utils
)
gtest_discover_tests(quadratic_cost_test)