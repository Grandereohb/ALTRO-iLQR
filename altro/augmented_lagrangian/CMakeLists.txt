add_library(augmented_lagrangian
  al_cost.hpp
  al_cost.cpp
  al_problem.hpp
  al_problem.cpp
  al_solver.hpp
  al_solver.cpp
)
target_link_libraries(augmented_lagrangian
  PUBLIC
  fmt::fmt
  Eigen3::Eigen
  Python3::Python
  Python3::NumPy

  PRIVATE
  constraints
  problem
  ilqr
  utils
)
target_include_directories(augmented_lagrangian PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})

export_library(augmented_lagrangian AltroCppTargets)