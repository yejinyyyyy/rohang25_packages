cmake_minimum_required(VERSION 3.8)
project(geolocation_kf_25)

# 1. C++17 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 필수 패키지 찾기
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yolov11_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 4. 헤더(include) 경로 추가
#    - include/ 아래의 사용자 헤더
#    - 빌드 중 자동 생성된 srv 헤더(rosidl_generator_cpp)
include_directories(include)

# 5. Custom 라이브러리 정의 & ROS2 라이브러리 의존성 연결
add_library(KALMANFILTER SHARED src/kalmanfilter.cpp)
ament_target_dependencies(KALMANFILTER Eigen3)
target_include_directories(KALMANFILTER PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
)

install(
  TARGETS KALMANFILTER
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
# add_library(CUSTOMLIB SHARED
#     # src/algebra.cpp
#     # src/frame.cpp
#     # src/guidance.cpp
# )

# 6. 실행 파일 정의 / 빌드된 바이너리 설치
add_executable(geolocation src/geolocation_kf_node.cpp)
ament_target_dependencies(geolocation rclcpp px4_msgs geometry_msgs yolov11_msgs std_msgs Eigen3)
install(TARGETS geolocation DESTINATION lib/${PROJECT_NAME})
target_link_libraries(geolocation KALMANFILTER)
target_link_libraries(geolocation Eigen3::Eigen)
# target_link_libraries(geolocation CUSTOMLIB)
# target_include_directories(CUSTOMLIB PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
# 	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# 8. 헤더 설치
# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

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
