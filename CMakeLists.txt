cmake_minimum_required(VERSION 3.8)
project(key_event)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지 찾기 (ROS 2 관련)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ncurses 라이브러리 찾기
find_library(NCURSES_LIB ncurses)

set(TARGET_NAME key_event)

# 실행 파일 추가
add_executable(${TARGET_NAME} src/key_event.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

# ROS 2 종속성 추가 (ncurses는 제외)
ament_target_dependencies(${TARGET_NAME}
  rclcpp
  std_msgs
)

# ncurses 라이브러리 링크
if (NCURSES_LIB)
  target_link_libraries(${TARGET_NAME} ${NCURSES_LIB})
else()
  message(FATAL_ERROR "ncurses library not found! Install ncurses with: sudo apt install libncurses5-dev")
endif()

# 설치 설정
install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
