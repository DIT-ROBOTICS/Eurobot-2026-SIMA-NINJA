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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(THIS_PACKAGE_DEPS
  rclcpp
  std_msgs
  btcpp_ros2_interfaces
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ninja_sima_main_node src/ninja-sima-main_node.cpp)
ament_target_dependencies(ninja_sima_main_node ${THIS_PACKAGE_DEPS})
install(TARGETS ninja_sima_main_node
  DESTINATION lib/${PROJECT_NAME}
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
