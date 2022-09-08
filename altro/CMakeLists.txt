if (ALTRO_RUN_CLANG_TIDY AND (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
  message("Running clang-tidy on source directory.")
  set(CMAKE_CXX_CLANG_TIDY
      clang-tidy-8;
      -export-fixes=tidy-fixes.yaml;
      -header-filter=.;
      -checks="bugprone-*, clang-*, google-*, readability-*, modernize-*, performance-*, portability-*";)
endif()

add_subdirectory(common)
add_subdirectory(ilqr)
add_subdirectory(problem)
add_subdirectory(utils)
add_subdirectory(constraints)
add_subdirectory(augmented_lagrangian)

add_library(altro INTERFACE
)

target_link_libraries(altro 
  INTERFACE
  fmt::fmt
  Eigen3::Eigen
  Python3::Python
  Python3::NumPy

  augmented_lagrangian
  common
  constraints
  ilqr
  problem
  utils
  threadpool
)
target_include_directories(altro PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})

export_library(altro AltroCppTargets)
