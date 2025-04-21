cmake_minimum_required(VERSION 3.8)
project(bno08x_driver)

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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gmock(test_i2c_mock test/i2c_interface_mock_test.cpp)
  if(TARGET test_i2c_mock)
    target_include_directories(test_i2c_mock PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  ament_add_gmock(test_watchdog test/watchdog_test.cpp)
  if(TARGET test_watchdog)
    target_include_directories(test_watchdog PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  # Linter checks
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
