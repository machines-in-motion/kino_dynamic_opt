#################################################
# additional configuration for real-time solver #
#################################################

set(solver_DEFINITIONS "EIGEN_STACK_ALLOCATION_LIMIT=0")
find_package(Boost REQUIRED COMPONENTS thread)