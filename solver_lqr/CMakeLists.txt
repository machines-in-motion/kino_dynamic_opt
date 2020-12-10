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
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS thread)


# local find package.
set(CMAKE_MODULE_PATH cmake ${CMAKE_MODULE_PATH})
find_package(solver REQUIRED)

# export dependencies
ament_export_dependencies(ament_cmake mpi_cmake_modules pybind11 yaml_utils
                          Eigen3 Boost solver)

# prepare the final export
set(all_targets)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")


#
# Add the main library
#

# Source for main library
set(solver_lqr_SRC_FILES ${solver_lqr_SRC_FILES}
  src/solver_lqr/SolverLqr.cpp
  src/solver_lqr/ForwardPass.cpp
  src/solver_lqr/BackwardPass.cpp
  src/solver_lqr/LqrInfoPrinter.cpp
  src/solver_lqr/OcpDescription.cpp
  src/solver_lqr/SolverLqrSetting.cpp
  src/solver_lqr/FiniteDifferences.cpp
)

# Source for Python bindings
set(pysolverlqr_SRC_FILES ${pysolverlqr_SRC_FILES}
  srcpy/solver_lqr/PySolverLqr.cpp

  srcpy/solver_lqr/PyParamsLqr.cpp
  srcpy/solver_lqr/PySettingLqr.cpp
  srcpy/solver_lqr/PyAlgorithmLqr.cpp
  srcpy/solver_lqr/PyOcpDescriptionLqr.cpp
)

# Add Library
add_library(solver_lqr SHARED ${solver_lqr_SRC_FILES})

# Includes. Add the include dependencies
target_include_directories(
  solver_lqr PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>)

# Linking.
target_link_libraries(solver_lqr yaml_utils::yaml_utils)
target_link_libraries(solver_lqr Eigen3::Eigen)
target_link_libraries(solver_lqr solver::solver)

# Properties.
set_target_properties(solver_lqr PROPERTIES COMPILE_DEFINITIONS
                                        "${solver_DEFINITIONS}")
set_target_properties(solver_lqr PROPERTIES COMPILE_DEFINITIONS CFG_PATH="${CONFIG_PATH}/")

# Exporting.
list(APPEND all_targets solver_lqr)

#
# Python wrappers.
#
add_library(pysolverlqr MODULE ${pysolverlqr_SRC_FILES})
target_link_libraries(pysolverlqr PRIVATE pybind11::module)
target_link_libraries(pysolverlqr PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(pysolverlqr PRIVATE solver_lqr)
set_target_properties(pysolverlqr PROPERTIES PREFIX ""
                                          SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  pysolverlqr
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS pysolverlqr DESTINATION ${PYTHON_INSTALL_DIR})
list(APPEND all_targets pysolverlqr)


#
# tests
#
if(BUILD_TESTING)

  # C++ unit-tests
  find_package(ament_cmake_gtest REQUIRED)
  set(GTEST_FILES ${GTEST_FILES} 
      tests/GtestMain.cpp
      tests/TestSolverLqr.cpp
      tests/TestSolverLqrHelpers.cpp)
  ament_add_gtest(solver_lqr_tests ${GTEST_FILES})
  target_link_libraries(solver_lqr_tests solver_lqr)
  target_include_directories(
    solver_lqr_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
                        $<INSTALL_INTERFACE:include>)
  get_filename_component(TEST_PATH tests ABSOLUTE)
  set_target_properties(
    solver_lqr_tests PROPERTIES COMPILE_DEFINITIONS
                            TEST_PATH="${TEST_PATH}/yaml_config_files/")

  # python unit tests
#   find_package(ament_cmake_pytest REQUIRED)
#   ament_add_pytest_test(test_${PROJECT_NAME}_python tests)

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

#
# export the package as a ament package
#
ament_package()