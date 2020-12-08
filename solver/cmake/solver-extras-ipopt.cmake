###############################
# flags for external packages #
###############################

if(IPOPT_FOUND)
  set(CMAKE_CXX_FLAGS "-DUSE_IPOPT ${CMAKE_CXX_FLAGS}")
endif()
