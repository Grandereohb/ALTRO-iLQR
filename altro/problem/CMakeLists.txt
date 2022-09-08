add_library(problem
  costfunction.cpp
  costfunction.hpp
  discretized_model.cpp
  discretized_model.hpp
  dynamics.cpp
  dynamics.hpp
  integration.cpp
  integration.hpp
  problem.cpp
  problem.hpp
)
target_link_libraries(problem
  PUBLIC
  Eigen3::Eigen
  fmt::fmt
  Python3::Python
  Python3::NumPy

  PRIVATE 
  common
  constraints
)
target_include_directories(problem PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
export_library(problem AltroCppTargets)
