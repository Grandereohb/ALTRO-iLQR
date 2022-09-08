add_library(ilqr
  cost_expansion.cpp
  cost_expansion.hpp
  dynamics_expansion.cpp
  dynamics_expansion.hpp
  knot_point_function_type.cpp
  knot_point_function_type.hpp
  ilqr.cpp
  ilqr.hpp
)
target_link_libraries(ilqr
  PUBLIC
  fmt::fmt
  Eigen3::Eigen
  threadpool
  Python3::Python
  Python3::NumPy

  PRIVATE
  common
  problem
  utils
)
target_include_directories(ilqr PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
export_library(ilqr AltroCppTargets)