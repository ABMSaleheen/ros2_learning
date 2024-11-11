cmake_minimum_required(VERSION 3.8)
project(rover_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


# Add executables for each node

add_executable(linear_velocity_publisher src/linear_velocity_publisher.cpp)
ament_target_dependencies(linear_velocity_publisher rclcpp geometry_msgs sensor_msgs)

# install(TARGETS linear_velocity_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(obs_avoidance_node src/obs_avoidance_cpp.cpp)
ament_target_dependencies(obs_avoidance_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(follow_path_node src/follow_path_cpp.cpp)
ament_target_dependencies(follow_path_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

#----------------  Install each executable to lib ---------------------------------
install(TARGETS obs_avoidance_node 
                follow_path_node
                linear_velocity_publisher
        DESTINATION lib/${PROJECT_NAME}
      )

# Include directories
include_directories(include)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install World files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()

