cmake_minimum_required(VERSION 2.8.3)
project(momentumopt)

#########
# flags #
#########

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -pedantic -O3 -funroll-loops -fPIC")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

#####################
# required packages #
#####################

set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  yaml_cpp_catkin
  pybind11_catkin
  # local include
  solver
  solverlqr
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_boost()
search_for_eigen()

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
  src/momentumopt/dynopt/DynamicsFeedback.cpp
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
set_target_properties(momentumopt PROPERTIES
    COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")

###################
# python bindings #
###################

pybind11_add_module(pymomentum ${pymomentumopt_SRC_FILES})
target_link_libraries(pymomentum PRIVATE momentumopt)
set_target_properties(pymomentum PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(TARGETS pymomentum DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

################
# python setup #
################

catkin_python_setup()


#########
# tests #
#########

catkin_add_gtest(test_momentumopt
  tests/gtest_main.cpp
  tests/test_momentumopt.cpp
)
target_link_libraries(test_momentumopt momentumopt)
target_compile_definitions(test_momentumopt PRIVATE
  CFG_SRC_PATH="${CONFIG_PATH}/"
  TEST_PATH="${TEST_PATH}/yaml_config_files/"
)

if(CATKIN_ENABLE_TESTING)
  CATKIN_ADD_NOSETESTS(tests)
endif()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_solo_generation.py)
endif()

#########
# demos #
#########
add_executable(demo_momentumopt demos/DemoMomentumopt.cpp)
target_link_libraries(demo_momentumopt momentumopt ${catkin_LIBRARIES})
set_target_properties(demo_momentumopt PROPERTIES
    COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")
##########################
# building documentation #
##########################
build_doxygen_documentation()