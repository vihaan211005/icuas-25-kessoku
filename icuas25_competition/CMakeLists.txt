cmake_minimum_required(VERSION 3.8)
project(icuas25_competition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
#ament_target_dependencies(cpp_executable rclcpp)
# Install Cpp executables
#install(TARGETS
#  cpp_executable
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/aideck_streamer_tcp.py
  scripts/aideck_streamer_udp.py
  scripts/crazyflie_server_reconnect.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch and config files.
install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
