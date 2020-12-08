###############################
# flags for external packages #
###############################

if(GUROBI_FOUND)
  set(CMAKE_CXX_FLAGS "-DUSE_GUROBI ${CMAKE_CXX_FLAGS}")
endif()
