cmake_minimum_required(VERSION 2.8.3)

set(This nop_cpp)

project(${This} C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


include_directories(
    include
)

set(LibSources
    src/baseFunctions.cpp
    src/nop.cpp
)

add_library(${This} STATIC ${LibSources})

install(
    TARGETS nop_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
