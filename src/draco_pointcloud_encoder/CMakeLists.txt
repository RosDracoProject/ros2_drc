cmake_minimum_required(VERSION 3.8)
project(draco_pointcloud_encoder)

# =========================
# C++ 표준 및 컴파일 옵션
# =========================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# ROS2 의존성
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# =========================
# Draco 라이브러리 설정
# =========================
# 사전 조건: ~/draco에서 빌드 완료 필요
set(DRACO_ROOT "$ENV{HOME}/draco" CACHE PATH "Draco source root")
set(DRACO_BUILD "${DRACO_ROOT}/build" CACHE PATH "Draco build directory")

# 헤더 경로 검증
if(NOT EXISTS "${DRACO_ROOT}/src")
  message(FATAL_ERROR "Draco headers not found at: ${DRACO_ROOT}/src")
endif()

# 라이브러리 파일 검색 (.so 우선, .a 대체)
set(_DRACO_SO "${DRACO_BUILD}/libdraco.so")
set(_DRACO_A  "${DRACO_BUILD}/libdraco.a")
if(EXISTS "${_DRACO_SO}")
  set(DRACO_LIB_PATH "${_DRACO_SO}")
elseif(EXISTS "${_DRACO_A}")
  set(DRACO_LIB_PATH "${_DRACO_A}")
else()
  message(FATAL_ERROR
    "Draco library not found. Expected at:\n  ${_DRACO_SO}\n  or\n  ${_DRACO_A}\n"
    "Build Draco first in ${DRACO_BUILD}")
endif()

# IMPORTED 라이브러리 정의
add_library(draco::draco UNKNOWN IMPORTED)
set_target_properties(draco::draco PROPERTIES
  IMPORTED_LOCATION "${DRACO_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${DRACO_ROOT}/src"
)

# =========================
# 실행 파일 빌드
# =========================
add_executable(encoder_node src/encoder_node.cpp)
add_executable(bag_processor_node src/bag_processor_node.cpp)
# add_executable(compression_analyzer src/compression_analyzer.cpp)  # 임시 주석 처리
# add_executable(bag_recorder src/bag_recorder.cpp)  # 임시 주석 처리
# add_executable(slam_integration_node src/slam_integration_node.cpp)  # 임시 주석 처리
# add_executable(tcp_encoder_node src/tcp_encoder_node.cpp)  # 임시 주석 처리

# ROS2 의존성 연결
ament_target_dependencies(encoder_node rclcpp sensor_msgs std_msgs)
ament_target_dependencies(bag_processor_node rclcpp sensor_msgs std_msgs rosbag2_cpp)
# ament_target_dependencies(compression_analyzer rclcpp sensor_msgs std_msgs)  # 임시 주석 처리
# ament_target_dependencies(bag_recorder rclcpp sensor_msgs std_msgs rosbag2_cpp)  # 임시 주석 처리
# ament_target_dependencies(slam_integration_node rclcpp sensor_msgs std_msgs tf2_ros tf2_geometry_msgs)  # 임시 주석 처리
# ament_target_dependencies(tcp_encoder_node rclcpp sensor_msgs std_msgs)  # 임시 주석 처리

# Draco 헤더 및 라이브러리 연결
target_include_directories(encoder_node PRIVATE "${DRACO_ROOT}/src")
target_link_libraries(encoder_node draco::draco)

target_include_directories(bag_processor_node PRIVATE "${DRACO_ROOT}/src")
target_link_libraries(bag_processor_node draco::draco)

# TCP 노드용 boost 라이브러리 연결 (임시 주석 처리)
# find_package(Boost REQUIRED COMPONENTS system)
# target_link_libraries(tcp_encoder_node PRIVATE Boost::system)

# 공유 라이브러리 사용 시 RPATH 설정
if(UNIX AND EXISTS "${_DRACO_SO}")
  set_target_properties(encoder_node PROPERTIES
    BUILD_RPATH "${DRACO_BUILD}"
    INSTALL_RPATH "${DRACO_BUILD}"
  )
endif()

# =========================
# 설치 설정
# =========================
install(TARGETS encoder_node bag_processor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS compression_analyzer  # 임시 주석 처리
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS bag_recorder slam_integration_node tcp_encoder_node  # 임시 주석 처리
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# 런치 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 설정 파일 설치 (임시 주석 처리)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# =========================
# 테스트 및 린팅
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스 헤더 준비 전 임시 스킵
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

