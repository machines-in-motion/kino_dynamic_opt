cmake_minimum_required(VERSION 2.8.3)
project(Eigen)

# find dependencies using catkin
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
)
