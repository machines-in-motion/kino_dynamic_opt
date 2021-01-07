#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(solver VERSION 1.0.0)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")


#
# Dependencies
#

unset(PYBIND11_CPP_STANDARD CACHE)
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# local deps
find_package(mpi_cmake_modules REQUIRED)

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

get_python_install_dir(python_install_dir)
install(TARGETS pysolver LIBRARY DESTINATION ${python_install_dir})

#
# Tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(GTest CONFIG REQUIRED)
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
  target_link_libraries(solver_tests GTest::gtest)
  target_include_directories(
    solver_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/>
  )
  get_filename_component(TEST_PATH tests ABSOLUTE)
  set_target_properties(
    solver_tests PROPERTIES COMPILE_DEFINITIONS
                            TEST_PATH="${TEST_PATH}/yaml_config_files/")
  gtest_add_tests(TARGET solver_tests)
endif()


#
# create the cmake package
#
generate_cmake_package()


#
# building documentation
#
add_documentation()
