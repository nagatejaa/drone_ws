cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include)

add_executable(takeoff_node src/takeoff_node.cpp)
ament_target_dependencies(takeoff_node rclcpp mavros_msgs)
install(TARGETS takeoff_node DESTINATION lib/${PROJECT_NAME})

add_executable(square src/square.cpp)
ament_target_dependencies(square rclcpp mavros_msgs)
install(TARGETS square DESTINATION lib/${PROJECT_NAME})

add_executable(takeoff_function src/takeoff_function.cpp)
ament_target_dependencies(takeoff_function rclcpp mavros_msgs)
install(TARGETS takeoff_function DESTINATION lib/${PROJECT_NAME})

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp mavros_msgs nav_msgs)
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  takeoff_node
  square
  takeoff_function
  test_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line on a copyrighted package
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
