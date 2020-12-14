#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(solver_lqr)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")

#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# local deps
find_package(mpi_cmake_modules REQUIRED)
find_package(solver REQUIRED)

#
# Add the main library
#

# Source for main library
set(solver_lqr_SRC_FILES
    ${solver_lqr_SRC_FILES}
    src/solver_lqr/SolverLqr.cpp
    src/solver_lqr/ForwardPass.cpp
    src/solver_lqr/BackwardPass.cpp
    src/solver_lqr/LqrInfoPrinter.cpp
    src/solver_lqr/OcpDescription.cpp
    src/solver_lqr/SolverLqrSetting.cpp
    src/solver_lqr/FiniteDifferences.cpp)

# Add Library
add_library(solver_lqr SHARED ${solver_lqr_SRC_FILES})

# Linking.
target_link_libraries(solver_lqr yaml_utils::yaml_utils)
target_link_libraries(solver_lqr Eigen3::Eigen)
target_link_libraries(solver_lqr solver::solver)

# Includes. Add the include dependencies
target_include_directories(
  solver_lqr PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)

# Properties.
# we add config paths
get_filename_component(CONFIG_PATH config ABSOLUTE)
target_compile_definitions(
  solver_lqr
  PUBLIC "${solver_DEFINITIONS}"
  PRIVATE CFG_PATH="${CONFIG_PATH}/"
  INTERFACE CFG_PATH="share/${PROJECT_NAME}/resource")

  # we install files where they should be
  # first the targets
install(
    TARGETS solver_lqr
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include)
# then the include + config files
install(DIRECTORY include/ DESTINATION include)
install(FILES config/default_solver_env.yaml
        DESTINATION share/${PROJECT_NAME}/resource)
  
#
# Python bindings
#
# Source for Python bindings
set(pysolverlqr_SRC_FILES
    ${pysolverlqr_SRC_FILES}
    srcpy/solver_lqr/PySolverLqr.cpp
    srcpy/solver_lqr/PyParamsLqr.cpp
    srcpy/solver_lqr/PySettingLqr.cpp
    srcpy/solver_lqr/PyAlgorithmLqr.cpp
    srcpy/solver_lqr/PyOcpDescriptionLqr.cpp)

pybind11_add_module(pysolverlqr MODULE ${pysolverlqr_SRC_FILES})
target_link_libraries(pysolverlqr PRIVATE pybind11::module)
target_link_libraries(pysolverlqr PRIVATE solver_lqr)

get_python_install_dir()
install(TARGETS pysolverlqr LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})


#
# tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  # C++ unit-tests
  add_executable(test_${PROJECT_NAME} 
                  tests/GtestMain.cpp
                  tests/TestSolverLqr.cpp
                  tests/TestSolverLqrHelpers.cpp)

  target_link_libraries(test_${PROJECT_NAME} solver_lqr)
  target_link_libraries(test_${PROJECT_NAME} GTest::GTest)
  target_include_directories(
    test_${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
  )
  get_filename_component(TEST_PATH tests ABSOLUTE)
  set_target_properties(
    test_${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS
                            TEST_PATH="${TEST_PATH}/yaml_config_files/")
  gtest_add_tests(TARGET test_${PROJECT_NAME})
endif()

#
# create the cmake package
#
generate_cmake_package(INSTALL_EXPORT)

# #
# # building documentation
# #
add_documentation()