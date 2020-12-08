###############################
# flags for external packages #
###############################

find_package(IPOPT QUIET)
if(IPOPT_FOUND)
  set(CMAKE_CXX_FLAGS "-DUSE_IPOPT ${CMAKE_CXX_FLAGS}")
endif()
