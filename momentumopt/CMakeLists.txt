#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(momentumopt)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC")

#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
# it is important to find python after we search for pybind11
# otherwise this activates the new pybind way of compiling things
find_package(Python)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

#local depends
find_package(solver REQUIRED)
find_package(solver_lqr REQUIRED)

#
# Build main library and binaries
#

# variable to store list of targets
set(all_targets)

# library source
set(momentumopt_SRC_FILES
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

# Add Library.
add_library(momentumopt SHARED ${momentumopt_SRC_FILES})
list(APPEND all_targets momentumopt)

# Linking.
target_link_libraries(momentumopt solver::solver)
target_link_libraries(momentumopt solver_lqr::solver_lqr)
target_link_libraries(momentumopt yaml_utils::yaml_utils)
target_link_libraries(momentumopt Eigen3::Eigen)

# Properties.
get_filename_component(CONFIG_PATH config ABSOLUTE)
target_compile_definitions(
  momentumopt
  PRIVATE CFG_SRC_PATH="${CONFIG_PATH}/"
  INTERFACE CFG_SRC_PATH="../share/${PROJECT_NAME}/config/")

# Includes. Add the include dependencies
target_include_directories(
  momentumopt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)

#
# Demos. (binaries)
#
add_executable(demo_momentumopt demos/demo_momentumopt.cpp)
target_link_libraries(demo_momentumopt momentumopt)
list(APPEND all_targets demo_momentumopt)

# command to install the library and binaries
install(
  TARGETS ${all_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES 
  DESTINATION include
)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

# the config folder
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/resource/)

function(install_scripts)
    cmake_parse_arguments(
        PARSE_ARGV
        0
        _args
        ""  # options without arguments
        "DESTINATION"  # options with single argument
        ""  # options with multiple arguments
    )

    # based on https://stackoverflow.com/a/10321017
    foreach(file ${_args_UNPARSED_ARGUMENTS})
        get_filename_component(name_without_extension ${file} NAME_WE)
        install(
            PROGRAMS ${file}
            DESTINATION ${_args_DESTINATION}
            RENAME ${name_without_extension}
        )
    endforeach()
endfunction()

# Install the demo python files
install_scripts(
  demos/demo_momentumopt_biped_from_python.py
  demos/demo_momentumopt_solo12_jump.py
  demos/demo_momentumopt_solo_jump_no_lqr.py
  demos/demo_momentumopt_solo_jump.py
  DESTINATION
  bin)



#
# Python bindings
#
set(pymomentumopt_SRC_FILES
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
    srcpy/momentumopt/setting/PySetting.cpp)
pybind11_add_module(pymomentum MODULE ${pymomentumopt_SRC_FILES})
target_link_libraries(pymomentum PRIVATE pybind11::module)
target_link_libraries(pymomentum PRIVATE momentumopt)

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

install(TARGETS pymomentum DESTINATION ${PYTHON_INSTALL_DIR})

# install the python package too
install(
    DIRECTORY python/${PROJECT_NAME}
    DESTINATION "${PYTHON_INSTALL_DIR}"
    PATTERN "*.pyc" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

#
# Tests.
#

find_package(GTest QUIET)
if(${GTEST_FOUND})
  include(GoogleTest)
  # C++ unit-tests
  add_executable(test_${PROJECT_NAME} 
                    tests/gtest_main.cpp
                    tests/test_momentumopt.cpp)
  target_link_libraries(test_${PROJECT_NAME} momentumopt)
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

# if(BUILD_TESTING)

#   # C++ unit-tests
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_momentumopt tests/gtest_main.cpp
#                   tests/test_momentumopt.cpp TIMEOUT 500)
#   target_link_libraries(test_momentumopt momentumopt)
#   target_include_directories(
#     test_momentumopt
#     PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
#            $<INSTALL_INTERFACE:include>)
#   get_filename_component(TEST_PATH tests ABSOLUTE)
#   target_compile_definitions(
#     test_momentumopt
#     PRIVATE TEST_PATH="${TEST_PATH}/yaml_config_files/")

#   # python unit tests
#   find_package(ament_cmake_pytest REQUIRED)
#   ament_add_pytest_test(test_${PROJECT_NAME}_python tests)

# endif()



# Python Native code.
# ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

#
# building documentation
#
# add_documentation()

