cmake_minimum_required(VERSION 3.8)
project(icuas25_competition)

# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(crazyflie_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(icuas25_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)
link_directories(
  ${OMPL_LIBRARIES}
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/TransformWorld2Odom.py
  scripts/charging.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  config
  launch
  worlds  
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(crazyflie_client src/crazyflie_client.cpp)
target_link_libraries(crazyflie_client 
  ompl
  fcl
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(crazyflie_client 
  rclcpp 
  crazyflie_interfaces 
  geometry_msgs
  sensor_msgs 
  visualization_msgs
  builtin_interfaces 
  ros2_aruco_interfaces 
  octomap_msgs 
  nav_msgs
  icuas25_msgs 
  octomap 
  Boost 
  Eigen3 
  ompl
)
target_compile_options(crazyflie_client PRIVATE -Wall -Wextra -Wpedantic -Ofast)
# target_compile_options(crazyflie_client PRIVATE -g -Wall -Wextra -Wpedantic -fsanitize=undefined)
# target_link_options(crazyflie_client PRIVATE -fsanitize=undefined)

# Install targets
install(TARGETS
  crazyflie_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
