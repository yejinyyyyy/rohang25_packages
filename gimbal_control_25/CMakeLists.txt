# CMakeLists.txt for gimbal_control_25

cmake_minimum_required(VERSION 3.8)
project(gimbal_control_25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)


# include directory
include_directories(include)

# add executable
add_executable(gimbal_control_node src/gimbal_control_node.cpp)
ament_target_dependencies(gimbal_control_node rclcpp std_msgs geometry_msgs px4_msgs)

# install
install(TARGETS
  gimbal_control_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

