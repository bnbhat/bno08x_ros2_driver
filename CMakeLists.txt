cmake_minimum_required(VERSION 3.8)
project(bno08x_ros2_driver)

# Check for compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall 
    -Wextra 
    -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter
    -Wno-unused-variable
    )
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-old-style-declaration")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories for main driver
include_directories(
  include
)

# Add the sh2 directory to build separately
add_subdirectory(include/sh2)

set(SOURCES
  src/bno08x.cpp
  src/bno08x_ros.cpp
  src/ros_node.cpp
)

# Create executable for the driver
add_executable(bno08x_driver ${SOURCES})

# Link include directories to the target
target_include_directories(bno08x_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set the required C and C++ standards
target_compile_features(bno08x_driver PUBLIC cxx_std_17)

# Link against sh2 library
target_link_libraries(bno08x_driver
  sh2_library
)

ament_target_dependencies(
  bno08x_driver
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

# Install the executable and the config files
install(TARGETS bno08x_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(test_uart_comm test/test_uart_interface.cpp)

  #target_include_directories(test_uart_comm PUBLIC
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #$<INSTALL_INTERFACE:include>
#)
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
