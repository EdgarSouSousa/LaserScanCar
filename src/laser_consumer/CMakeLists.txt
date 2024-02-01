cmake_minimum_required(VERSION 3.8)
project(laser_consumer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(laser_consumer
  src/laser_consumer.cpp
  src/lshaped_fitting.cpp  # Add your new source file here
)

target_include_directories(laser_consumer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV headers
)

target_compile_features(laser_consumer PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(laser_consumer
  rclcpp
  sensor_msgs
  OpenCV  # Add OpenCV as a dependency
)

target_link_libraries(laser_consumer
  ${OpenCV_LIBS}  # Link against OpenCV libraries
)

install(TARGETS laser_consumer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
