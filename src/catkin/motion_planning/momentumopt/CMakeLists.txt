cmake_minimum_required(VERSION 2.8.3)
project(momentumopt)

#########
# flags #
#########

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -pedantic -O3 -funroll-loops -fPIC -Wno-return-type -Wno-reorder")

#####################
# required packages #
#####################

set(CATKIN_PKGS ${CATKIN_PKGS}
  solver
  yaml_cpp
  pybind11
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
find_package(Boost  REQUIRED COMPONENTS system program_options filesystem)

##################
# catkin package #
##################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES momentumopt
  CATKIN_DEPENDS ${CATKIN_PKGS}
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
  include
  ${GTEST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

################
# source files #
################

set(momentumopt_SRC_FILES ${momentumopt_SRC_FILES}
  # files for contacts planning
  src/momentumopt/cntopt/ContactState.cpp
  src/momentumopt/cntopt/TerrainDescription.cpp
  src/momentumopt/cntopt/ContactPlanFromFile.cpp
  src/momentumopt/cntopt/ContactPlanInterface.cpp

  # files for dynamics planning
  src/momentumopt/dynopt/DynamicsState.cpp
  src/momentumopt/dynopt/DynamicsOptimizer.cpp

  # files for kinematics planning
  src/momentumopt/kinopt/KinematicsState.cpp
  src/momentumopt/kinopt/KinematicsInterface.cpp
  src/momentumopt/kinopt/KinematicsOptimizer.cpp

  # files for setting up environments
  src/momentumopt/setting/PlannerSetting.cpp
  
  # utilities
  src/momentumopt/utilities/OrientationUtils.cpp
  src/momentumopt/utilities/TrajectoryGenerator.cpp
)

set(pymomentumopt_SRC_FILES ${pymomentumopt_SRC_FILES}
  # files for contacts planning
  srcpy/momentumopt/cntopt/PyTerrain.cpp
  srcpy/momentumopt/cntopt/PyContacts.cpp
  
  # files for dynamics planning
  srcpy/momentumopt/dynopt/PyDynamics.cpp
  
  # files for kinematics planning
  srcpy/momentumopt/kinopt/PyKinematics.cpp
  
  # files for setting up environments
  srcpy/momentumopt/PyMomentumopt.cpp
  srcpy/momentumopt/setting/PyParams.cpp
  srcpy/momentumopt/setting/PySetting.cpp
)

#####################
# linking libraries #
#####################

set(momentumopt_LIBS
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(momentumopt_LIBS ${momentumopt_LIBS} rt)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

###################
# build libraries #
###################

add_library(momentumopt ${momentumopt_SRC_FILES} )
target_link_libraries(momentumopt ${momentumopt_LIBS})
set_target_properties(momentumopt PROPERTIES COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")

#########
# tests #
#########

catkin_add_gtest(momentumopt_tests
  tests/gtest_main.cpp
  tests/TestMomentumopt.cpp
)
target_link_libraries(momentumopt_tests momentumopt)
set_target_properties(momentumopt_tests PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/yaml_config_files/")

#########
# demos #
#########

add_executable(demo_momentumopt demos/DemoMomentumopt.cpp)
target_link_libraries(demo_momentumopt momentumopt)
set_target_properties(demo_momentumopt PROPERTIES COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")

###################
# python bindings #
###################

pybind11_add_module(pymomentum ${pymomentumopt_SRC_FILES})
target_link_libraries(pymomentum momentumopt)
