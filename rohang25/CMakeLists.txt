cmake_minimum_required(VERSION 3.8)
project(rohang25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(include)

add_library(CONTROL SHARED src/control.cpp)
ament_target_dependencies(CONTROL px4_msgs rclcpp)
target_include_directories(CONTROL PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(LOG SHARED src/log.cpp)
ament_target_dependencies(LOG px4_msgs rclcpp mavros_msgs)
target_include_directories(LOG PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)



add_library(CUSTOMLIB SHARED
    src/algebra.cpp
    src/frame.cpp
    src/guidance.cpp
    
    
)


add_executable(offboard_control src/main_offboard.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs )
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})
target_link_libraries(offboard_control CUSTOMLIB)
target_link_libraries(offboard_control CONTROL)
target_link_libraries(offboard_control LOG)
target_include_directories(CUSTOMLIB PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(CONTROL PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(LOG PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Export information to downstream packages - add for custom libraries if needed
#ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

#ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

#ament_export_include_directories(include)
#ament_export_libraries(frame_transforms)

# Install launch files.
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


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




