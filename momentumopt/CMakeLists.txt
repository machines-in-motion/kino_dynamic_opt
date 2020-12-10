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

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

# depend on ament macros
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(solver REQUIRED COMPONENTS thread)
find_package(solver_lqr REQUIRED COMPONENTS thread)
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# export de dependencies
ament_export_dependencies(
  ament_cmake
  ament_cmake_python
  mpi_cmake_modules
  solver
  solver_lqr
  pybind11
  yaml_utils
  Eigen3
  Boost)

# prepare the final export
set(all_targets)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC")

#
# Definitions.
#
get_filename_component(TEST_PATH tests ABSOLUTE)
get_filename_component(CONFIG_PATH config ABSOLUTE)

#
# Build main library.
#

# source
set(momentumopt_SRC_FILES
    ${momentumopt_SRC_FILES}
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
    src/momentumopt/utilities/TrajectoryGenerator.cpp)
# Add Library.
add_library(momentumopt SHARED ${momentumopt_SRC_FILES})
# Includes. Add the include dependencies
target_include_directories(
  momentumopt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
# Linking.
target_link_libraries(momentumopt solver::solver)
target_link_libraries(momentumopt solver_lqr::solver_lqr)
target_link_libraries(momentumopt yaml_utils::yaml_utils)
target_link_libraries(momentumopt Eigen3::Eigen)
# Properties.
target_compile_definitions(
  momentumopt
  PRIVATE CFG_SRC_PATH="${CONFIG_PATH}/"
  INTERFACE CFG_SRC_PATH="share/${PROJECT_NAME}/resource/")
# Exporting.
list(APPEND all_targets momentumopt)

#
# Python wrappers.
#
set(pymomentumopt_SRC_FILES
    ${pymomentumopt_SRC_FILES}
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
add_library(pymomentum MODULE ${pymomentumopt_SRC_FILES})
target_link_libraries(pymomentum PRIVATE pybind11::module)
target_link_libraries(pymomentum PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(pymomentum PRIVATE momentumopt)
set_target_properties(pymomentum
                      PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  pymomentum
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS pymomentum DESTINATION ${PYTHON_INSTALL_DIR})
list(APPEND all_targets pymomentum)

# #
# ##############################################################################
# # linking libraries # #
# ##############################################################################

# set(momentumopt_LIBS ${Boost_LIBRARIES} ${catkin_LIBRARIES})
# if(${CMAKE_SYSTEM_NAME} MATCHES "Linux") set(momentumopt_LIBS
# ${momentumopt_LIBS} rt) endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

#
# Demos.
#
add_executable(demo_momentumopt demos/demo_momentumopt.cpp)
target_link_libraries(demo_momentumopt momentumopt)
list(APPEND all_targets demo_momentumopt)

#
# Tests.
#

if(BUILD_TESTING)

  # C++ unit-tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_momentumopt tests/gtest_main.cpp
                  tests/test_momentumopt.cpp)
  target_link_libraries(test_momentumopt momentumopt)
  target_include_directories(
    test_momentumopt
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
           $<INSTALL_INTERFACE:include>)
  get_filename_component(TEST_PATH tests ABSOLUTE)
  target_compile_definitions(
    test_momentumopt
    PRIVATE TEST_PATH="${TEST_PATH}/yaml_config_files/")

  # python unit tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_${PROJECT_NAME}_python tests)

endif()

#
# Install everything.
#
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# the config folder
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install the demo python files
install_scripts(
  demos/demo_momentumopt_biped_from_python.py
  demos/demo_momentumopt_solo12_jump.py
  demos/demo_momentumopt_solo_jump_no_lqr.py
  demos/demo_momentumopt_solo_jump.py
  DESTINATION
  bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/resource/)

# Python Native code.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

#
# building documentation
#
add_documentation()

#
# export the package as a ament package
#
ament_package()
