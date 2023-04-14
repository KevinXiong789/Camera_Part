cmake_minimum_required(VERSION 3.8)
project(pointcloud_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(std_msgs REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#min_distance.msg part
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#pointcloud_min_distance.cpp part
add_executable(pointcloud_test_sub
  src/pointcloud_min_distance.cpp
)
ament_target_dependencies(pointcloud_test_sub
  rclcpp 
  sensor_msgs
  
)
target_link_libraries(pointcloud_test_sub ${PCL_LIBRARIES})

#min_distance_get_pub.cpp part
add_executable(min_distance_get
  src/min_distance_get_pub.cpp
)
ament_target_dependencies(min_distance_get
  rclcpp 
  sensor_msgs
  std_msgs
)
target_link_libraries(min_distance_get ${PCL_LIBRARIES})

#handover_tool_in_hand.cpp part
add_executable(handover_tool_in_hand
  src/handover_tool_in_hand_detector.cpp
)
ament_target_dependencies(handover_tool_in_hand
  rclcpp 
  sensor_msgs
  std_msgs
  
)
target_link_libraries(handover_tool_in_hand ${PCL_LIBRARIES})


#Don't forget add executable name here
install(TARGETS
    pointcloud_test_sub
    min_distance_get
    handover_tool_in_hand
    DESTINATION lib/${PROJECT_NAME}) 



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
