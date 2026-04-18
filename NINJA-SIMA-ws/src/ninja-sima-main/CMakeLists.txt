cmake_minimum_required(VERSION 3.8)
project(ninja-sima-main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(THIS_PACKAGE_DEPS
  rclcpp
  std_msgs
  btcpp_ros2_interfaces
  rclcpp_action
  nav2_msgs
  opennav_docking_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
  ament_index_cpp
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ninja_sima_main_node 
  src/ninja-sima-main_node.cpp
  src/ninja-sima-main_nav_to_pose.cpp
)
target_link_libraries(ninja_sima_main_node yaml-cpp)
ament_target_dependencies(ninja_sima_main_node ${THIS_PACKAGE_DEPS})
install(TARGETS ninja_sima_main_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
