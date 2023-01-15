cmake_minimum_required(VERSION 3.0.2)

set(This rosbot_nop_controller)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

project(${This} C CXX)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  nav_core
  # pluginlib
  std_msgs
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

add_library(astar_planner src/AStarPlanner.cpp)

add_executable(
  ${This} 
  src/rosbot_controller.cpp
  src/rosbot_controller_runner.cpp
)

add_executable(
  Follower 
  src/follower.cpp
)

target_link_libraries(
  ${This} 
  ${catkin_LIBRARIES} 
  nop_cpp
)

target_link_libraries(
  Follower 
  ${catkin_LIBRARIES} 
  nop_cpp
  astar_planner
)


install(
  TARGETS ${This}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS Follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


