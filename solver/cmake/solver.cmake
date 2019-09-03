#################################################
# additional configuration for real-time solver #
#################################################

set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
set(solver_DEFINITIONS "EIGEN_STACK_ALLOCATION_LIMIT=0")