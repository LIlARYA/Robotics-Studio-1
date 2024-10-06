cmake_minimum_required(VERSION 3.8)
project(map_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(map_overlay src/main.cpp)
ament_target_dependencies(map_overlay rclcpp cv_bridge image_transport OpenCV)

install(TARGETS map_overlay
  DESTINATION lib/${PROJECT_NAME})
