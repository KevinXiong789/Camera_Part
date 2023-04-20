cmake_minimum_required(VERSION 3.8)
project(pointcloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(std_msgs REQUIRED)


#min_distance.cpp part
add_executable(min_distance src/min_distance.cpp)
ament_target_dependencies(min_distance
  rclcpp 
  sensor_msgs
  std_msgs
)

#handover_T_inhand_detector.cpp part
add_executable(handover_T_inhand_detector src/handover_T_inhand_detector.cpp)
ament_target_dependencies(handover_T_inhand_detector
  rclcpp 
  sensor_msgs
  std_msgs
)



#Don't forget add executable name here
install(TARGETS
    min_distance
    handover_T_inhand_detector
    DESTINATION lib/${PROJECT_NAME}) 





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
