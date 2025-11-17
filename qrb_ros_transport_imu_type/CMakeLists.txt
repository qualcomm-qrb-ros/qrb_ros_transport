cmake_minimum_required(VERSION 3.8)
project(qrb_ros_transport_imu_type)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic --std=c++17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/imu.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(${PROJECT_NAME}_imu_test_pub test/imu_test.cpp)
  target_compile_definitions(${PROJECT_NAME}_imu_test_pub PRIVATE "-DTEST_PUBLISHER")

  ament_auto_add_gtest(${PROJECT_NAME}_imu_test_sub test/imu_test.cpp)
  target_compile_definitions(${PROJECT_NAME}_imu_test_sub PRIVATE "-DTEST_SUBSCRIBER")
endif()

ament_auto_package()
