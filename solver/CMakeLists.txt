cmake_minimum_required(VERSION 3.5.1)
project(solver)

#########
# flags #
#########

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#####################
# required packages #
#####################

set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  pybind11_catkin
  yaml_cpp_catkin
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_boost()
search_for_eigen()

#####################
# optional packages #
#####################

# To be fixed, the cmake crashes when using IPOPT
# find_package(IPOPT QUIET)
# find_package(GUROBI QUIET)

##################
# catkin package #
##################

set(EXPORTED_LIBRARIES ${EXPORTED_LIBRARIES} "solver")
set(EXPORTED_CFG_EXTRAS ${EXPORTED_CFG_EXTRAS} "solver.cmake")
set(EXPORTED_INCLUDE_DIRS ${EXPORTED_INCLUDE_DIRS} "include" ${EIGEN3_INCLUDE_DIR})

if(IPOPT_FOUND)
  set(EXPORTED_LIBRARIES ${EXPORTED_LIBRARIES} ${IPOPT_LIBRARIES})
  set(EXPORTED_CFG_EXTRAS ${EXPORTED_CFG_EXTRAS} "${PROJECT_NAME}-extras-ipopt.cmake")
  set(EXPORTED_INCLUDE_DIRS ${EXPORTED_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})
endif()

if(GUROBI_FOUND)
  set(EXPORTED_LIBRARIES ${EXPORTED_LIBRARIES} ${GUROBI_LIBRARIES})
  set(EXPORTED_CFG_EXTRAS ${EXPORTED_CFG_EXTRAS} "${PROJECT_NAME}-extras-gurobi.cmake")
  set(EXPORTED_INCLUDE_DIRS ${EXPORTED_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS})
endif()

catkin_package(
  CATKIN_DEPENDS ${CATKIN_PKGS}
  LIBRARIES ${EXPORTED_LIBRARIES}
  CFG_EXTRAS ${EXPORTED_CFG_EXTRAS}
  INCLUDE_DIRS ${EXPORTED_INCLUDE_DIRS}
)

###############
# definitions #
###############

get_filename_component(TEST_PATH tests ABSOLUTE)
get_filename_component(CONFIG_PATH config ABSOLUTE)

#######################
# include directories #
#######################

include_directories(
  tests
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYBIND11_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#####################
# optional includes #
#####################

if (IPOPT_FOUND)
  include_directories( ${IPOPT_INCLUDE_DIRS} )
  include(${PROJECT_SOURCE_DIR}/cmake/solver-extras-ipopt.cmake)
endif (IPOPT_FOUND)

if (GUROBI_FOUND)
  include_directories( ${GUROBI_INCLUDE_DIRS} )
  include(${PROJECT_SOURCE_DIR}/cmake/solver-extras-gurobi.cmake)
endif (GUROBI_FOUND)

################
# source files #
################

set(solver_SRC_FILES ${solver_SRC_FILES}
  src/solver/interface/Var.cpp
  src/solver/interface/Cone.cpp
  src/solver/interface/Exprs.cpp
  src/solver/interface/Model.cpp
  src/solver/interface/OptVar.cpp
  src/solver/interface/ConicProblem.cpp
  src/solver/interface/SolverSetting.cpp

  src/solver/optimizer/IPSolver.cpp
  src/solver/optimizer/LinSolver.cpp
  src/solver/optimizer/EqRoutine.cpp
  src/solver/optimizer/BnBSolver.cpp
  src/solver/optimizer/LbfgsSolver.cpp
  src/solver/optimizer/NcvxBnBSolver.cpp
  src/solver/optimizer/SparseCholesky.cpp
  src/solver/optimizer/CvxInfoPrinter.cpp
)

set(pysolver_SRC_FILES ${pysolver_SRC_FILES}
  srcpy/solver/PySolver.cpp
  srcpy/solver/interface/PySetting.cpp
)

#####################
# linking libraries #
#####################

set(solver_LIBS
  m
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(solver_LIBS ${solver_LIBS} rt)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

######################
# optional libraries #
######################

if (IPOPT_FOUND)
  set(solver_LIBS ${solver_LIBS} ${IPOPT_LIBRARIES})
endif (IPOPT_FOUND)

if (GUROBI_FOUND)
  set(solver_LIBS ${solver_LIBS} ${GUROBI_LIBRARIES})
endif (GUROBI_FOUND)

###################
# build libraries #
###################

add_library(solver ${solver_SRC_FILES})
target_link_libraries(solver ${solver_LIBS})
set_target_properties(solver PROPERTIES COMPILE_DEFINITIONS "${solver_DEFINITIONS}")

###################
# python bindings #
###################

pybind11_add_module(pysolver ${pysolver_SRC_FILES})
target_link_libraries(pysolver PRIVATE solver)
set_target_properties(pysolver PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(TARGETS pysolver DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

#########
# tests #
#########

set(GTEST_FILES ${GTEST_FILES}
  tests/GtestMain.cpp
  tests/TestLbfgs.cpp
  tests/TestSolver.cpp
  tests/TestRtSolver.cpp
)

if(IPOPT_FOUND)
  set(GTEST_FILES ${GTEST_FILES} tests/TestIpopt.cpp )
endif()

if(GUROBI_FOUND)
  set(GTEST_FILES ${GTEST_FILES} tests/TestGurobi.cpp )
endif()

catkin_add_gtest(solver_tests ${GTEST_FILES})
target_link_libraries(solver_tests solver ${catkin_LIBRARIES})
set_target_properties(solver_tests PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/yaml_config_files/")
