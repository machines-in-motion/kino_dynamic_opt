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
find_package(Boost REQUIRED COMPONENTS thread)


#
# the solver library MAIN libs
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
    src/solver/optimizer/CvxInfoPrinter.cpp
)

# ignore warning for external imported files
set_source_files_properties(
src/solver/optimizer/IPSolver.cpp
src/solver/optimizer/LinSolver.cpp
src/solver/optimizer/EqRoutine.cpp
src/solver/optimizer/BnBSolver.cpp
src/solver/optimizer/LbfgsSolver.cpp
src/solver/optimizer/NcvxBnBSolver.cpp
src/solver/optimizer/SparseCholesky.cpp
src/solver/optimizer/CvxInfoPrinter.cpp
PROPERTIES
COMPILE_FLAGS "-w"
)

# Add Library.
add_library(solver SHARED ${solver_SRC_FILES})

# Linking.
target_link_libraries(solver yaml_utils::yaml_utils)
target_link_libraries(solver Eigen3::Eigen)
target_link_libraries(solver Boost::boost)
target_link_libraries(solver Boost::thread)

# Properties.
# for realtime solver - not used
#set(solver_DEFINITIONS "EIGEN_STACK_ALLOCATION_LIMIT=0")
# set_target_properties(solver PROPERTIES COMPILE_DEFINITIONS
#                                         "${solver_DEFINITIONS}")


# we tell cmake which include files need to be used for this library 
# first when we build it and also after it is installed
target_include_directories(
  solver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>)


# command to install the library
install(
  TARGETS solver
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES 
  DESTINATION include
  )
# we also need to install the header files
# here we just copy the whole directory
# we might want to only open a few for other packages (replace DIRECTORY by FILES)
install(DIRECTORY include/ DESTINATION include)

#
# Python bindings
#
pybind11_add_module(pysolver MODULE srcpy/solver/PySolver.cpp
                            srcpy/solver/interface/PySetting.cpp)
target_link_libraries(pysolver PRIVATE pybind11::module)
target_link_libraries(pysolver PRIVATE solver)
# target_include_directories(
#   pysolver
#   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:include>
#   PUBLIC ${PYTHON_INCLUDE_DIRS})

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

install(TARGETS pysolver LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})

#
# Tests
#

find_package(GTest QUIET)
if(${GTEST_FOUND})
  include(GoogleTest)
  # C++ unit-tests
  add_executable(solver_tests 
                tests/gtest_main.cpp 
                tests/test_lbfgs.cpp
                tests/test_solver.cpp
                tests/test_rt_solver.cpp)
  # if(IPOPT_FOUND)
  #   set(GTEST_FILES ${GTEST_FILES} tests/test_ipopt.cpp)
  # endif()
  # if(GUROBI_FOUND)
  #   set(GTEST_FILES ${GTEST_FILES} tests/test_gurobi.cpp)
  # endif()
  target_link_libraries(solver_tests solver)
  target_link_libraries(solver_tests GTest::GTest)
  target_include_directories(
    solver_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
  )
  get_filename_component(TEST_PATH tests ABSOLUTE)
  set_target_properties(
    solver_tests PROPERTIES COMPILE_DEFINITIONS
                            TEST_PATH="${TEST_PATH}/yaml_config_files/")
  gtest_add_tests(TARGET solver_tests)
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
