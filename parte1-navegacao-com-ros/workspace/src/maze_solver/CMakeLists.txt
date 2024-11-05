cmake_minimum_required(VERSION 3.8)
project(maze_solver)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)  # Add dependency on cg_interfaces package

# Add executable for the maze_solver
add_executable(maze_solver src/maze_solver.cpp)

# Specify the dependencies for the maze_solver executable
ament_target_dependencies(maze_solver rclcpp std_msgs cg_interfaces)

# Install the executable
install(TARGETS
  maze_solver
  DESTINATION lib/${PROJECT_NAME})

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

