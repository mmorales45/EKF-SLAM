find_package(rostest REQUIRED)

catch_add_rostest_node(turtle_interface_test turtle_interface_test.cpp)
target_link_libraries(turtle_interface_test ${catkin_LIBRARIES})


catch_add_rostest_node(turtle_odom_test_node turtle_odom_test_node.cpp)
target_link_libraries(turtle_odom_test_node ${catkin_LIBRARIES})
