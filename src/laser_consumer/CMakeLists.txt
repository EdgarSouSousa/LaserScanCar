cmake_minimum_required(VERSION 3.8)
project(laser_consumer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # This line should now work

add_executable(laser_consumer src/laser_consumer.cpp)
target_include_directories(laser_consumer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(laser_consumer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link against the required libraries including OpenCV
ament_target_dependencies(laser_consumer
  rclcpp
  sensor_msgs
)
target_link_libraries(laser_consumer
  ${OpenCV_LIBS}  # Change this line to use ${OpenCV_LIBS}
)

install(TARGETS laser_consumer
  DESTINATION lib/${PROJECT_NAME})

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

