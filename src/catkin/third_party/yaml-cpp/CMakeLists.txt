cmake_minimum_required(VERSION 2.8.3)
project(yaml_cpp)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_BUILD_TYPE Release)

# find dependencies using catkin
find_package (Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})

# required packages
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES yaml_cpp
)

include_directories(
  src
  tests
  include
  include/node
  include/contrib
  include/node/detail
  ${catkin_INCLUDE_DIRS}
)

# set source files
set (yaml_cpp_SRC
  src/binary.cpp
  src/convert.cpp
  src/directives.cpp
  src/emit.cpp
  src/emitfromevents.cpp
  src/emitter.cpp
  src/emitterstate.cpp
  src/emitterutils.cpp
  src/exp.cpp
  src/memory.cpp
  src/node_data.cpp
  src/node.cpp
  src/nodebuilder.cpp
  src/nodeevents.cpp
  src/null.cpp
  src/ostream_wrapper.cpp
  src/parse.cpp
  src/parser.cpp
  src/regex_yaml.cpp
  src/scanner.cpp
  src/scanscalar.cpp
  src/scantag.cpp
  src/scantoken.cpp
  src/simplekey.cpp
  src/singledocparser.cpp
  src/stream.cpp
  src/tag.cpp
  src/contrib/graphbuilder.cpp
  src/contrib/graphbuilderadapter.cpp
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(yaml_cpp ${yaml_cpp_SRC})