cmake_minimum_required(VERSION 2.8.3)
project(solverlqr)

#########
# flags #
#########

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -pedantic -O3 -funroll-loops -fPIC")

#####################
# required packages #
#####################

set(CATKIN_PKGS ${CATKIN_PKGS}
  # third party dependencies
  mpi_cmake_modules
  yaml_cpp_catkin
  pybind11_catkin
  # local dependencies
  solver
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_boost()
search_for_eigen()

#######################
# include directories #
#######################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_PKGS}
  LIBRARIES solverlqr
)

get_filename_component(TEST_PATH tests ABSOLUTE)
get_filename_component(CONFIG_PATH config ABSOLUTE)

include_directories(
  tests
  include
  ${GTEST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

################
# source files #
################

set(solverlqr_SRC_FILES ${solverlqr_SRC_FILES}
  src/solver_lqr/SolverLqr.cpp
  src/solver_lqr/ForwardPass.cpp
  src/solver_lqr/BackwardPass.cpp
  src/solver_lqr/LqrInfoPrinter.cpp
  src/solver_lqr/OcpDescription.cpp
  src/solver_lqr/SolverLqrSetting.cpp
  src/solver_lqr/FiniteDifferences.cpp
)

set(pysolverlqr_SRC_FILES ${pysolverlqr_SRC_FILES}
  srcpy/solver_lqr/PySolverLqr.cpp

  srcpy/solver_lqr/PyParamsLqr.cpp
  srcpy/solver_lqr/PySettingLqr.cpp
  srcpy/solver_lqr/PyAlgorithmLqr.cpp
  srcpy/solver_lqr/PyOcpDescriptionLqr.cpp
)

#####################
# linking libraries #
#####################

set(solverlqr_LIBS
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

###################
# build libraries #
###################

add_library(solverlqr ${solverlqr_SRC_FILES})
target_link_libraries(solverlqr ${solverlqr_LIBS})
set_target_properties(solverlqr PROPERTIES COMPILE_DEFINITIONS CFG_PATH="${CONFIG_PATH}/")

###################
# python bindings #
###################

pybind11_add_module(pysolverlqr ${pysolverlqr_SRC_FILES})
target_link_libraries(pysolverlqr PRIVATE solverlqr)
set_target_properties(pysolverlqr PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
install(TARGETS pysolverlqr DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

#########
# tests #
#########
catkin_add_gtest(solverlqr_tests
  tests/GtestMain.cpp
  tests/TestSolverLqr.cpp
  tests/TestSolverLqrHelpers.cpp
)
target_link_libraries(solverlqr_tests solverlqr)
set_target_properties(solverlqr_tests PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/yaml_config_files/")
