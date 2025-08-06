cmake_minimum_required(VERSION 3.8)
project(rohang25_test)

# 1. C++17 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 필수 패키지 찾기
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(yolov11_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

# 3. 서비스(.srv) 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGuidanceParam.srv"
  DEPENDENCIES std_msgs
)
# 런타임에 필요할 의존성 export
ament_export_dependencies(rosidl_default_runtime)

# 4. 헤더(include) 경로 추가
#    - include/ 아래의 사용자 헤더
#    - 빌드 중 자동 생성된 srv 헤더(rosidl_generator_cpp)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 5. Custom 라이브러리 정의 & ROS2 라이브러리 의존성 연결
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

add_library(GUIDANCE SHARED src/guidance.cpp)
ament_target_dependencies(GUIDANCE geometry_msgs rclcpp)
target_include_directories(GUIDANCE PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(CUSTOMLIB SHARED
    src/algebra.cpp
    src/frame.cpp
)

install(
  TARGETS CONTROL LOG GUIDANCE CUSTOMLIB
  EXPORT export_${PROJECT_NAME}       # export the targets for downstream use
  ARCHIVE DESTINATION lib            # for static libs (if any)
  LIBRARY DESTINATION lib            # for shared libs
  RUNTIME DESTINATION bin            # for Windows DLLs or executables
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 6. 실행 파일 정의 / 빌드된 바이너리 설치
add_executable(offboard_control_test src/main_offboard.cpp)
ament_target_dependencies(offboard_control_test rclcpp px4_msgs geometry_msgs std_msgs yolov11_msgs)
install(TARGETS offboard_control_test DESTINATION lib/${PROJECT_NAME})
target_link_libraries(offboard_control_test CUSTOMLIB)
target_link_libraries(offboard_control_test CONTROL)
target_link_libraries(offboard_control_test LOG)
target_link_libraries(offboard_control_test GUIDANCE)
target_include_directories(CUSTOMLIB PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# 7. 생성된 서비스 타입 코드를 linkage
rosidl_target_interfaces(offboard_control_test
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 8. 헤더 설치
# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# 9. Launch 파일 설치
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
