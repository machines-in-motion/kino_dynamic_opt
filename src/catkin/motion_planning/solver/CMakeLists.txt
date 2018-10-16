cmake_minimum_required(VERSION 2.8.3)
project(solver)

#########
# flags #
#########

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")

#####################
# required packages #
#####################

set(CATKIN_PKGS ${CATKIN_PKGS}
  yaml_cpp
  pybind11
)

find_package (Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)

##################
# catkin package #
##################

catkin_package(
  LIBRARIES solver
  CATKIN_DEPENDS ${CATKIN_PKGS}
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
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
  ${catkin_INCLUDE_DIRS}
)

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

###################
# build libraries #
###################

add_library(solver ${solver_SRC_FILES})
target_link_libraries(solver ${solver_LIBS})

###################
# python bindings #
###################

pybind11_add_module(pysolver ${pysolver_SRC_FILES})
target_link_libraries(pysolver solver)

#########
# tests #
#########

catkin_add_gtest(solver_tests
  tests/GtestMain.cpp
  tests/TestLbfgs.cpp
  tests/TestSolver.cpp
)
target_link_libraries(solver_tests solver)
set_target_properties(solver_tests PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/yaml_config_files/")
