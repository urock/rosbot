cmake_minimum_required(VERSION 2.8.3)
project(rosbot_controller)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

