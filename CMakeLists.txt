cmake_minimum_required(VERSION 3.8)
project(qrb_ros_transport)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dmabuf_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/type/image.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  dmabuf_transport
  OpenCV
  cv_bridge
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(${PROJECT_NAME}_image_test
    test/image_test.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_image_test
    rclcpp
    sensor_msgs
  )
  target_link_libraries(${PROJECT_NAME}_image_test
    dmabuf_transport
    ${PROJECT_NAME}
  )

  install(TARGETS
    ${PROJECT_NAME}_image_test DESTINATION lib
  )
endif()

ament_package()
