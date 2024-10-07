cmake_minimum_required(VERSION 3.8)
project(d1_waypoints_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tsukutsuku2_msgs REQUIRED)



find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

#add_executable(nav2_send_goal src/nav2_send_goal.cpp)

ament_auto_add_executable(d1_waypoints_manager src/d1_waypoints_manager.cpp src/d1_waypoints_manager_node.cpp)
ament_target_dependencies(d1_waypoints_manager rclcpp rclcpp_action nav2_msgs nav_msgs tf2 tf2_geometry_msgs tsukutsuku2_msgs)

#ament_auto_add_executable(find_point_publisher src/find_point_publisher.cpp)
#ament_target_dependencies(find_point_publisher rclcpp std_msgs)


install(TARGETS
  d1_waypoints_manager
  #find_point_publisher
  #state_and_feedback_publisher
  DESTINATION lib/${PROJECT_NAME}/
)


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

#ament_auto_package(
#  INSTALL_TO_SHARE
#  launch
#)

ament_package()