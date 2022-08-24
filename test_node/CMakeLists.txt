cmake_minimum_required(VERSION 3.0.2)
project(test_node)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
)

catkin_package(
  INCLUDE_DIRS  include
  CATKIN_DEPENDS geometry_msgs std_msgs gazebo_msgs
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
  include
  include/network_operator/lib/include
  ${catkin_INCLUDE_DIRS}
)

set(BUILD_TESTS OFF CACHE INTERNAL "Don't build Gtests")
set(TO_CATKIN_WS ON CACHE INTERNAL "")

add_subdirectory(
  include/network_operator
)

add_executable(
  test_node 
  src/test_node.cpp
)

target_link_libraries(
  test_node 
  ${catkin_LIBRARIES} 
  nop_cpp
)

install(
  TARGETS test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
