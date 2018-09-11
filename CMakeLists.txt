cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_ulc_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  can_msgs
  dataspeed_dbw_msgs
)

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelet.cpp
  src/UlcNode.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

### Install Rules
# TODO

### Tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
