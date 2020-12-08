#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(solver)

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
find_package(Boost REQUIRED COMPONENTS thread)

# local find package.
set(CMAKE_MODULE_PATH cmake ${CMAKE_MODULE_PATH})
find_package(GUROBI QUIET)
find_package(IPOPT QUIET)

# export de dependencies
ament_export_dependencies(ament_cmake mpi_cmake_modules pybind11 yaml_utils
                          Eigen3)

# prepare the final export
set(all_targets)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC")

#
# Add the main library
#

# Source.
set(solver_SRC_FILES
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
    src/solver/optimizer/CvxInfoPrinter.cpp)
# Add Library.
add_library(solver ${solver_SRC_FILES})
# Includes. Add the include dependencies
target_include_directories(
  solver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>)
# Linking.
target_link_libraries(solver yaml_utils::yaml_utils)
target_link_libraries(solver Eigen3::Eigen)
target_link_libraries(solver Boost::boost)
target_link_libraries(solver Boost::thread)
if(IPOPT_FOUND)
  include(cmake/solver-extras-ipopt.cmake)
  target_link_libraries(solver IPOPT::IPOPT)
endif()
if(GUROBI_FOUND)
  include(cmake/solver-extras-gurobi.cmake)
  target_link_libraries(solver GUROBI::GUROBI)
endif()
# Properties.
set_target_properties(solver PROPERTIES COMPILE_DEFINITIONS
                                        "${solver_DEFINITIONS}")
# Exporting.
list(APPEND all_targets solver)

#
# Python wrappers.
#
add_library(pysolver MODULE srcpy/solver/PySolver.cpp
                            srcpy/solver/interface/PySetting.cpp)
target_link_libraries(pysolver PRIVATE pybind11::module)
target_link_libraries(pysolver PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(pysolver PRIVATE solver)
set_target_properties(pysolver PROPERTIES PREFIX ""
                                          SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  pysolver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS pysolver DESTINATION ${PYTHON_INSTALL_DIR})
list(APPEND all_targets pysolver)

#
# tests
#

if(BUILD_TESTING)

  # C++ unit-tests
  find_package(ament_cmake_gtest REQUIRED)
  set(GTEST_FILES ${GTEST_FILES} tests/gtest_main.cpp tests/test_lbfgs.cpp
                  tests/test_solver.cpp tests/test_rt_solver.cpp)
  if(IPOPT_FOUND)
    set(GTEST_FILES ${GTEST_FILES} tests/test_ipopt.cpp)
  endif()
  if(GUROBI_FOUND)
    set(GTEST_FILES ${GTEST_FILES} tests/test_gurobi.cpp)
  endif()
  ament_add_gtest(solver_tests ${GTEST_FILES})
  target_link_libraries(solver_tests solver)
  target_include_directories(
    solver_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
                        $<INSTALL_INTERFACE:include>)
  get_filename_component(TEST_PATH tests ABSOLUTE)
  set_target_properties(
    solver_tests PROPERTIES COMPILE_DEFINITIONS
                            TEST_PATH="${TEST_PATH}/yaml_config_files/")

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

# install the CMake find modules at the good place
macro(SUBDIRLIST result curdir)
  file(
    GLOB children
    RELATIVE ${curdir}
    ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist cmake/${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()
subdirlist(cmake_dirs ${PROJECT_SOURCE_DIR}/cmake)
install(DIRECTORY ${cmake_dirs} DESTINATION share/${PROJECT_NAME}/cmake)

#
# building documentation
#
add_documentation()

#
# export the package as a ament package
#
file(
  GLOB cmake_files
  RELATIVE ${PROJECT_SOURCE_DIR}
  cmake/*.cmake)
ament_package(CONFIG_EXTRAS ${cmake_files})
