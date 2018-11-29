### Tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}_test test_ulc_node.test test_ulc_node.cpp)
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
