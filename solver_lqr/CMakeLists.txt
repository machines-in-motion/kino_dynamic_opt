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
# it is important to find python after we search for pybind11
# otherwise this activates the new pybind way of compiling things
find_package(Python)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# local deps
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

# code to find installation path for python libs
set(_python_code
      "from distutils.sysconfig import get_python_lib"
      "import os"
      "print(os.path.relpath(get_python_lib(prefix='${CMAKE_INSTALL_PREFIX}'), start='${CMAKE_INSTALL_PREFIX}').replace(os.sep, '/'))"
)
execute_process(
  COMMAND
  "${PYTHON_EXECUTABLE}"
  "-c"
  "${_python_code}"
  OUTPUT_VARIABLE _output
  RESULT_VARIABLE _result
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT _result EQUAL 0)
  message(FATAL_ERROR
    "execute_process(${PYTHON_EXECUTABLE} -c '${_python_code}') returned "
    "error code ${_result}"
    )
endif()
set(PYTHON_INSTALL_DIR
      "${_output}"
      CACHE INTERNAL
      "The directory for Python library installation. This needs to be in PYTHONPATH when 'setup.py install' is called."
)

install(TARGETS pysolverlqr LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})


#
# tests
#
find_package(GTest QUIET)
if(${GTEST_FOUND})
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
else()
  message(WARNING "GTest NOT FOUND")
endif()

#
# create the cmake package
#
# we need this to create a nice exportable package
include(CMakePackageConfigHelpers)

# generate the necessary cmake file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion )

# we copy the cmake files we would need to configure the project
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION share/${PROJECT_NAME}/cmake
)


# we install the cmake package
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION share/${PROJECT_NAME}/cmake
)

# #
# # building documentation
# #

# add_documentation()