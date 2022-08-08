cmake_minimum_required(VERSION 3.0.2)
project(test_node)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs std_msgs gazebo_msgs
)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(test_node src/model.cpp src/baseFunctions.cpp src/nop.cpp src/controller.cpp  src/test_node.cpp)
  target_link_libraries(test_node ${catkin_LIBRARIES})
  install(TARGETS test_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



    # include_directories(${catkin_INCLUDE_DIRS})
    # add_executable(test_node test_node.cpp)
    # target_link_libraries(test_node ${catkin_LIBRARIES})
    # install(TARGETS test_node
    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
