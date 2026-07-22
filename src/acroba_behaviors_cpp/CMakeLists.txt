cmake_minimum_required(VERSION 3.8)
project(acroba_behaviors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(acroba_msgs REQUIRED)

# Avoid obstacle behavior node
add_executable(avoid_obstacle src/avoid_obstacle.cpp)
ament_target_dependencies(avoid_obstacle
  rclcpp rclcpp_action sensor_msgs geometry_msgs acroba_msgs)
target_include_directories(avoid_obstacle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(avoid_obstacle PUBLIC cxx_std_17)

# Follow wall behavior node
add_executable(follow_wall src/follow_wall.cpp)
ament_target_dependencies(follow_wall
  rclcpp rclcpp_action sensor_msgs geometry_msgs acroba_msgs)
target_include_directories(follow_wall PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(follow_wall PUBLIC cxx_std_17)

# Emergency stop behavior node
add_executable(emergency_stop src/emergency_stop.cpp)
ament_target_dependencies(emergency_stop
  rclcpp rclcpp_action sensor_msgs geometry_msgs acroba_msgs)
target_include_directories(emergency_stop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(emergency_stop PUBLIC cxx_std_17)

install(TARGETS
  avoid_obstacle
  follow_wall
  emergency_stop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
